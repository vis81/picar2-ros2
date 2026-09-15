#!/usr/bin/env python3
"""Turn a saved map into a benchmark scenario.

The scenarios written by hand are deliberately simple — one corner, one
doorway — so that a result points at one thing. This does the opposite: it
rebuilds a real place the robot has already driven, from the map it built
there, so a run in Gazebo can be compared against numbers measured on the
floor rather than only against other simulations.

    bench-from-map maps/F1.yaml --waypoints maps/F1.waypoints.yaml \\
                   --name f1_route --laps 3 -o src/.../scenarios

The occupancy grid becomes static boxes. Cells are merged into maximal
rectangles first: F1 is 2543 occupied cells, which as one box each would be a
world Gazebo has to collision-check 2543 times a step, and as rectangles is
409. The merge is greedy — extend right, then down while the full width stays
occupied — which is not optimal but is linear and close enough that the
remainder are genuinely irregular.

Coordinates are kept exactly as the map has them, so a waypoint file recorded
on the robot drops in unchanged and the sim's numbers line up with the real
ones. That is the whole point; re-centring the world would silently invalidate
the comparison.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import yaml

# 0.3 m: twice the LD19's scan plane (0.1485 m), so the lidar sees every
# wall, and low enough that the Gazebo GUI shows the robot and the route
# instead of a maze of 1 m boxes. A map's walls are walls either way.
WALL_HEIGHT = 0.3


def read_pgm(path: Path) -> np.ndarray:
    """Binary PGM (P5) as a height x width array. Written by hand because the
    map format is fixed and pulling in Pillow for four integers is not worth
    the dependency on a Pi."""
    data = path.read_bytes()
    fields: list[bytes] = []
    i = 0
    while len(fields) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':                     # comment to end of line
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        fields.append(data[i:j])
        i = j
    if fields[0] != b'P5':
        raise ValueError(f'{path}: expected a binary PGM (P5), got {fields[0]!r}')
    w, h = int(fields[1]), int(fields[2])
    i += 1                                            # single whitespace byte
    return np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)


def rectangles(occupied: np.ndarray) -> list[tuple[int, int, int, int]]:
    """Cover the occupied cells with disjoint rectangles, (r0, c0, r1, c1)."""
    free = occupied.copy()
    h, w = free.shape
    out = []
    for r in range(h):
        c = 0
        while c < w:
            if not free[r, c]:
                c += 1
                continue
            c1 = c
            while c1 + 1 < w and free[r, c1 + 1]:
                c1 += 1
            r1 = r
            while r1 + 1 < h and free[r1 + 1, c:c1 + 1].all():
                r1 += 1
            out.append((r, c, r1, c1))
            free[r:r1 + 1, c:c1 + 1] = False
            c = c1 + 1
    return out


def declutter(occ: np.ndarray, min_cells: int) -> tuple[np.ndarray, int, int]:
    """Drop occupied blobs smaller than min_cells.

    A saved map is full of specks — a chair leg seen once, a reflection, a
    person who walked past while mapping. Reproducing them makes a world that
    is slower to simulate and harder to reason about without being more like
    the place: what shapes a route is the corridor it drives, not the litter.
    Whole connected components go, rather than small rectangles, because a
    long wall decomposes into several small rectangles and dropping those
    would punch holes in it.
    """
    try:
        from scipy import ndimage
    except ImportError:                                # keep working without scipy
        return occ, 0, 0
    if min_cells <= 1:
        return occ, 0, 0
    labels, n = ndimage.label(occ)
    if n == 0:
        return occ, 0, 0
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0                                       # background
    keep = sizes >= min_cells
    cleaned = keep[labels]
    return cleaned, int((~keep[1:]).sum()), int(occ.sum() - cleaned.sum())


def coarsen(occ: np.ndarray, k: int) -> np.ndarray:
    """Merge k x k blocks, occupied if any cell in the block is.

    The point of this world is the corridor the route drives, not a faithful
    copy of every pixel the mapper recorded. Coarsening cuts the box count
    roughly in half per step and leaves the shape of the place intact.

    Occupied-if-any rather than majority: it can only thicken a wall, never
    open a gap through one. A world that is slightly tight is a harder test
    than the real place; a world with a hole in a wall is a different place.
    The cost is that corridors narrow by up to (k-1) cells on each side, which
    is why the generator reports waypoint clearances afterwards.

    Pad rather than crop, and pad at the top and right. The map's origin is its
    bottom-left corner, so that is the corner the block grid has to stay glued
    to: cropping the last row instead shifts every wall in the world down by
    one cell (5 cm for F1) against waypoints that did not move, which is a bias
    in precisely the comparison this world exists to make.
    """
    if k <= 1:
        return occ
    h, w = occ.shape
    ph, pw = (-h) % k, (-w) % k
    if ph or pw:
        occ = np.pad(occ, ((ph, 0), (0, pw)))   # top and right; bottom-left is the anchor
    h, w = occ.shape
    return occ.reshape(h // k, k, w // k, k).any(axis=(1, 3))


def boxes_from_map(map_yaml: Path, occupied_below: int = 1, min_cells: int = 12,
                   coarsen_by: int = 1
                   ) -> tuple[list[dict], tuple[float, float], dict]:
    """Static boxes for the map's walls, and the world size that contains them.

    occupied_below: a cell counts as occupied at or below this pixel value.
    The trinary maps written by map_saver use 0 for occupied, 205 unknown,
    254 free; unknown is left out deliberately — it is space the robot never
    saw, not space something is in.

    min_cells: blobs smaller than this are dropped. See declutter.
    """
    info = yaml.safe_load(map_yaml.read_text())
    res = float(info['resolution'])
    ox, oy = float(info['origin'][0]), float(info['origin'][1])
    img = read_pgm(map_yaml.parent / info['image'])
    h, w = img.shape
    occ = img <= occupied_below
    raw_cells = int(occ.sum())
    occ, dropped_blobs, dropped_cells = declutter(occ, min_cells)
    occ = coarsen(occ, coarsen_by)
    res *= coarsen_by
    h, w = occ.shape
    stats = {'raw_cells': raw_cells, 'dropped_blobs': dropped_blobs,
             'dropped_cells': dropped_cells, 'min_cells': min_cells,
             'coarsen': coarsen_by, 'cell_m': round(res, 3),
             'min_area_m2': round(min_cells * 0.05 * 0.05, 4)}

    out = []
    for r0, c0, r1, c1 in rectangles(occ):
        # Cell (r, c) spans x in [ox + c*res, ox + (c+1)*res); rows count from
        # the top of the image, which is the top of the world.
        x0 = ox + c0 * res
        x1 = ox + (c1 + 1) * res
        y0 = oy + (h - 1 - r1) * res
        y1 = oy + (h - r0) * res
        out.append({'x': round((x0 + x1) / 2, 4), 'y': round((y0 + y1) / 2, 4),
                    'sx': round(x1 - x0, 4), 'sy': round(y1 - y0, 4),
                    'sz': WALL_HEIGHT})

    # The scenario's own boundary walls are generated at +-size/2, so the world
    # has to be big enough to hold the map in the map's own coordinates. Adding
    # a margin keeps those walls off the map's edge, where they would show up
    # as obstacles the real place does not have.
    span_x = max(abs(ox), abs(ox + w * res)) + 1.0
    span_y = max(abs(oy), abs(oy + h * res)) + 1.0
    stats['boxes'] = len(out)
    return out, (round(2 * span_x, 1), round(2 * span_y, 1)), stats


def load_waypoints(path: Path) -> list[dict]:
    """The RViz Nav2 panel's Save Waypoints layout, orientation w-first."""
    doc = yaml.safe_load(path.read_text()) or {}
    wps = doc.get('waypoints') or {}
    out = []
    for key in sorted(wps, key=lambda k: int(''.join(c for c in k if c.isdigit()) or 0)):
        w = wps[key]
        x, y = float(w['pose'][0]), float(w['pose'][1])
        o = w['orientation']
        yaw = math.atan2(2.0 * float(o[0]) * float(o[3]),
                         1.0 - 2.0 * float(o[3]) ** 2)
        out.append({'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4)})
    return out


def build(map_yaml: Path, waypoints: Path | None, name: str, laps: int,
          loop: bool, start: tuple[float, float, float] | None,
          timeout_s: float, min_cells: int = 12,
          coarsen_by: int = 1) -> tuple[str, dict]:
    boxes, size, stats = boxes_from_map(map_yaml, min_cells=min_cells,
                                        coarsen_by=coarsen_by)
    wps = load_waypoints(waypoints) if waypoints else []

    if start is None:
        if not wps:
            raise SystemExit('--start is required when there are no waypoints')
        # Just behind the first waypoint, facing it, so the route starts the
        # way it does on the robot: already pointed down the first leg.
        if len(wps) > 1:
            bearing = math.atan2(wps[0]['y'] - wps[-1]['y'], wps[0]['x'] - wps[-1]['x'])
        else:
            bearing = wps[0]['yaw']
        start = (round(wps[0]['x'] - 0.8 * math.cos(bearing), 4),
                 round(wps[0]['y'] - 0.8 * math.sin(bearing), 4),
                 round(bearing, 4))

    lines = [
        f'name: {name}',
        'description: >',
        f'  Generated from {map_yaml.name} by bench-from-map — the real place the',
        '  robot drives, rebuilt from the map it built there, so a run here can be',
        '  compared against numbers measured on the floor rather than only against',
        '  other simulations.',
        '',
        f'  {len(boxes)} static boxes, merged from the occupancy grid. Blobs under',
        f'  {stats["min_cells"]} cells ({stats["min_area_m2"]} m2) were dropped first:'
        f' {stats["dropped_blobs"]} of them, which is',
        '  the litter a saved map collects — a chair leg seen once, a reflection,',
        '  someone who walked past while mapping. What shapes a route is the corridor',
        '  it drives. Coordinates are the map\'s own, so a waypoint file recorded on',
        '  the robot drops in unchanged and the lap and leg times line up.',
        '',
        '  Regenerate rather than edit: the geometry is derived, and a hand edit here',
        '  is lost the next time the map changes.',
        f'world: {{size: [{size[0]}, {size[1]}], rtf: 0.5, max_step: 0.001}}',
        'obstacles:',
    ]
    for b in boxes:
        lines.append(f"  - {{x: {b['x']}, y: {b['y']}, sx: {b['sx']}, "
                     f"sy: {b['sy']}, sz: {b['sz']}}}")
    lines += [
        f'start: {{x: {start[0]}, y: {start[1]}, yaw: {start[2]}}}',
        'goal:  {x: 0.0, y: 0.0, yaw: 0.0}     # unused on a route; kept for the loader',
        f'timeout_s: {timeout_s}',
    ]
    if wps:
        lines += [
            'route:',
            f'  loop: {str(bool(loop)).lower()}',
            f'  laps: {laps}',
            f'  timeout_s: {timeout_s}',
            '  capture_radius_m: 0.45',
            '  waypoints:',
        ]
        for w in wps:
            lines.append(f"    - {{x: {w['x']}, y: {w['y']}, yaw: {w['yaw']}}}")
    return '\n'.join(lines) + '\n', stats


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description='Generate a benchmark scenario from a saved map.')
    ap.add_argument('map_yaml', type=Path)
    ap.add_argument('--waypoints', type=Path,
                    help='a <map>.waypoints.yaml to drive as a route')
    ap.add_argument('--name', help='scenario name (default: the map name)')
    ap.add_argument('--laps', type=int, default=3)
    ap.add_argument('--no-loop', dest='loop', action='store_false')
    # Needs the = form for a negative x: argparse reads a leading - as a flag.
    ap.add_argument('--start', metavar='X,Y,YAW',
                    help='e.g. --start=-1.8,-1.3,1.28 . Default is just behind '
                         'waypoint 0, which can land near a wall — pass the pose '
                         'the robot actually started from when comparing runs.')
    ap.add_argument('--timeout-s', type=float, default=600.0)
    ap.add_argument('--coarsen', type=int, default=1,
                    help='merge NxN cells into one box cell. Default 1 = off, '
                         'because occupied-if-any thickens every wall by up to '
                         '(N-1) cells on each side and that comes straight out '
                         'of the corridor: on F1, --coarsen 2 halved the box '
                         'count but cut the real driven path\'s clearance from '
                         '0.094 m to 0.024 m, and the route then lost 10 s a lap '
                         'at 0.6 m/s fighting the one leg where it mattered. '
                         'Raise it only if the simulator cannot hold its rtf, '
                         'and re-check clearance when you do.')
    ap.add_argument('--min-cells', type=int, default=12,
                    help='drop occupied blobs smaller than this many cells '
                         '(default 12, about 0.03 m2 at 5 cm resolution). '
                         '1 keeps everything.')
    ap.add_argument('-o', '--out', type=Path, required=True,
                    help='directory to write <name>.yaml into')
    a = ap.parse_args(argv)

    name = a.name or a.map_yaml.stem
    start = None
    if a.start:
        start = tuple(float(v) for v in a.start.split(','))   # type: ignore[assignment]
    text, stats = build(a.map_yaml, a.waypoints, name, a.laps, a.loop, start,
                        a.timeout_s, a.min_cells, a.coarsen)
    a.out.mkdir(parents=True, exist_ok=True)
    dest = a.out / f'{name}.yaml'
    dest.write_text(text)

    # Load it back through the real loader: a scenario that does not validate
    # is worse than none, because it fails later inside a trial.
    from . import spec
    sc = spec.load(dest)
    print(f'{dest}')
    print(f'  {len(sc.obstacles)} obstacles, world {sc.size[0]} x {sc.size[1]} m')
    print(f'  {stats["raw_cells"]} occupied cells -> dropped {stats["dropped_blobs"]} '
          f'blob(s) under {stats["min_area_m2"]} m2, coarsened {stats["coarsen"]}x '
          f'to {stats["cell_m"]} m cells')
    if stats['coarsen'] > 1:
        grew = (stats['coarsen'] - 1) * stats['cell_m'] / stats['coarsen']
        print(f'  WARNING: coarsening thickens every wall by up to {grew:.2f} m '
              f'per side, straight out of the corridor width')
    # Coarsening thickens walls, so say how much room is left where it matters
    # rather than leaving it to be discovered by a robot that cannot fit.
    from .geometry import clearance
    tight = []
    for i, p in enumerate(sc.route_waypoints):
        c = clearance((p.x, p.y, p.yaw), sc.all_boxes)
        if c < 0.35:
            tight.append((i, c))
    sc_clear = clearance((sc.start.x, sc.start.y, sc.start.yaw), sc.all_boxes)
    print(f'  start clearance {sc_clear:.2f} m'
          + (f', TIGHT waypoints: ' + ', '.join(f'{i}={c:.2f}m' for i, c in tight)
             if tight else ', all waypoints clear'))
    if sc.route:
        print(f'  {len(sc.route_waypoints)} waypoints, '
              f'loop={sc.route.get("loop")} laps={sc.route.get("laps")}')
    return 0

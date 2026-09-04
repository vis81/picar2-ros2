"""Command-line entry points."""
from __future__ import annotations

import argparse
from pathlib import Path

from . import map_gen, spec, world_gen


def generate_main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Generate a benchmark world and map.')
    ap.add_argument('scenario', help='path to a scenario YAML')
    ap.add_argument('-o', '--out', default='/tmp/picar2_bench',
                    help='output directory (default: %(default)s)')
    a = ap.parse_args(argv)

    sc = spec.load(a.scenario)          # validates start/goal clearance
    out = Path(a.out) / sc.name
    out.mkdir(parents=True, exist_ok=True)
    world = out / f'{sc.name}.sdf'
    world.write_text(world_gen.to_sdf(sc))
    pgm, yml = map_gen.write(sc, out)

    from .geometry import clearance
    print(f'{sc.name}: {len(sc.obstacles)} obstacles + {len(sc.walls)} walls')
    print(f'  world {world}')
    print(f'  map   {yml}  ({pgm.name})')
    print(f'  start ({sc.start.x}, {sc.start.y}) clearance '
          f'{clearance((sc.start.x, sc.start.y, sc.start.yaw), sc.all_boxes):.3f} m')
    print(f'  goal  ({sc.goal.x}, {sc.goal.y}) clearance '
          f'{clearance((sc.goal.x, sc.goal.y, sc.goal.yaw), sc.all_boxes):.3f} m')
    return 0

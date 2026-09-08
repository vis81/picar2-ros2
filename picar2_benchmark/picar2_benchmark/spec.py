"""Scenario specification.

One YAML declares the obstacles once; they are then used for three separate
purposes — generating the Gazebo world, rasterising the static map, and
computing clearance analytically. Deriving all three from a single source is
what keeps the map, the simulated world and the metrics from drifting apart.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

# Robot footprint, copied from nav2.yaml. base_footprint sits at the rear axle
# and the chassis extends 40 mm behind it.
FOOTPRINT = [(-0.04, -0.095), (-0.04, 0.095), (0.30, 0.095), (0.30, -0.095)]

# Every generated world uses one fixed name so the ground-truth topic is a
# constant string rather than something templated per scenario.
WORLD_NAME = 'picar2_bench'


@dataclass(frozen=True)
class Box:
    """An axis-aligned obstacle, metres. Always static in the generated world:
    a box the robot can shove would desynchronise the world from the map and
    silently invalidate every clearance number."""
    x: float
    y: float
    sx: float
    sy: float

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        return (self.x - self.sx / 2, self.y - self.sy / 2,
                self.x + self.sx / 2, self.y + self.sy / 2)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class Scenario:
    name: str
    size: tuple[float, float]
    start: Pose
    goal: Pose
    obstacles: list[Box] = field(default_factory=list)
    timeout_s: float = 90.0
    rtf: float = 0.5
    max_step: float = 0.001
    wall_thickness: float = 0.2
    description: str = ''
    # Present on exploration scenarios, absent on navigation ones. Exploration
    # has no goal pose: the task is to map the space, so the run ends on
    # coverage or on the map ceasing to grow, not on arrival.
    explore: dict | None = None
    # Present on route scenarios. A route is a list of waypoints driven in
    # order, optionally looping. It is its own task because the thing being
    # measured is not arrival at one pose but whether every waypoint was
    # actually passed close to, in order — the failure mode a single goal
    # cannot express.
    route: dict | None = None

    @property
    def route_waypoints(self) -> list[Pose]:
        if not self.route:
            return []
        return [_pose(w) for w in self.route.get('waypoints', [])]

    @property
    def walls(self) -> list[Box]:
        """Boundary walls, generated rather than declared so every scenario is
        enclosed and no goal can sit outside the world."""
        w, h = self.size
        t = self.wall_thickness
        return [
            Box(0.0, (h + t) / 2, w + 2 * t, t),
            Box(0.0, -(h + t) / 2, w + 2 * t, t),
            Box((w + t) / 2, 0.0, t, h + 2 * t),
            Box(-(w + t) / 2, 0.0, t, h + 2 * t),
        ]

    @property
    def all_boxes(self) -> list[Box]:
        return self.walls + self.obstacles


# The simulated LD19 and cartographer both cap at 5.0 m (picar2.urdf.xacro
# <range><max>, cartographer.lua max_range). Rays that return nothing are
# dropped rather than inserted as free space, so the map covers only where the
# lidar got an actual return.
LIDAR_MAX_RANGE = 5.0


def slam_envelope(sc: 'Scenario', max_range: float = LIDAR_MAX_RANGE,
                  rays: int = 360) -> tuple[float, float, float, float]:
    """Bounding box of everything the lidar can see from the start pose.

    Under SLAM the global costmap is sized from cartographer's map, and that map
    extends only as far as the lidar gets returns — an unreturned ray contributes
    nothing. In an open world that makes the map far smaller than the sensor
    range suggests: measured on short_hop (14x8 world, 5 m range, walls 4 m to
    each side), only rays within 53 deg of the side walls returned at all, so the
    map reached just sqrt(5^2 - 4^2) = 3.0 m ahead of the robot and a goal 4 m
    away sat outside it. Predicted finite-ray fraction 0.408 against 147/360
    measured, which is what this model is built on.

    A goal outside this box cannot be planned to in slam mode however long the
    warm-up, so validate() rejects it instead of letting the trial report a
    misleading navigation failure.
    """
    from .geometry import ray_hit

    o = (sc.start.x, sc.start.y)
    xs: list[float] = []
    ys: list[float] = []
    for i in range(rays):
        a = 2.0 * math.pi * i / rays
        d = ray_hit(o, a, sc.all_boxes, max_range)
        if d is None:
            continue
        xs.append(o[0] + d * math.cos(a))
        ys.append(o[1] + d * math.sin(a))
    if not xs:
        return (o[0], o[1], o[0], o[1])
    return (min(xs), min(ys), max(xs), max(ys))


def pose_before(bt_events, pose_series, node: str = 'FollowPath',
                lead_s: float = 1.0):
    """The pose `lead_s` before `node` first reported FAILURE, or None.

    Kept here rather than on the Recorder so it can be tested without ROS: the
    path it feeds only runs when a trial fails, which is rare enough that it
    would otherwise ship unverified.
    """
    fails = [t for t, name, status in bt_events
             if name == node and status == 'FAILURE']
    if not fails or not pose_series:
        return None
    target = fails[0] - lead_s
    best = min(pose_series, key=lambda p: abs(p[0] - target))
    return best[1], best[2], best[3]


def write_repro(sc: 'Scenario', pose, out_dir) -> Path:
    """Write a scenario that starts where control broke down.

    A rosbag cannot reproduce a closed-loop failure: replaying it feeds the
    stack its old inputs while the robot no longer responds to the commands it
    emits, so the state diverges within a control cycle. Respawning at the pose
    where control broke down does reproduce it, and turns "run it until it
    fails again" into a single deterministic run - worth having when the
    corner_right thrash fires in only about 3 runs of 19.
    """
    x, y, yaw = pose
    lines = [f'name: {sc.name}_repro',
             'description: >',
             f'  Auto-generated from a failed {sc.name} run. Starts at the pose where',
             '  FollowPath first failed, so the breakdown is reproduced directly',
             '  rather than waiting for it to recur on its own.',
             f'world: {{size: [{sc.size[0]}, {sc.size[1]}], rtf: {sc.rtf}, '
             f'max_step: {sc.max_step}}}']
    if sc.obstacles:
        lines.append('obstacles:')
        lines += [f'  - {{x: {b.x}, y: {b.y}, sx: {b.sx}, sy: {b.sy}}}'
                  for b in sc.obstacles]
    else:
        lines.append('obstacles: []')
    lines += [f'start: {{x: {x:.3f}, y: {y:.3f}, yaw: {yaw:.4f}}}',
              f'goal:  {{x: {sc.goal.x}, y: {sc.goal.y}, yaw: {sc.goal.yaw}}}',
              f'timeout_s: {sc.timeout_s}']
    out = Path(out_dir) / 'repro.yaml'
    out.write_text('\n'.join(lines) + '\n')
    return out


def free_area_m2(sc: 'Scenario', resolution: float = 0.05) -> float:
    """True free area of the world, in square metres.

    The explorer only knows what it has mapped; this is what there was to map.
    Coverage measured against it is a number the robot cannot compute about
    itself, which is the point - the same reason ground truth is collected in
    every localisation mode.
    """
    from .map_gen import rasterise, FREE
    img, _ = rasterise(sc, resolution)
    return float((img == FREE).sum()) * resolution * resolution


def _validate_route(sc: 'Scenario') -> None:
    """Reject a route that cannot be driven, rather than reporting the refusal
    as a navigation result."""
    from .geometry import clearance

    r = sc.route
    wps = sc.route_waypoints
    w, h = sc.size
    if len(wps) < 2:
        raise ValueError(f'{sc.name}: a route needs at least two waypoints')
    if int(r.get('laps', 1)) < 1:
        raise ValueError(f'{sc.name}: route.laps must be at least 1')
    if r.get('laps', 1) > 1 and not r.get('loop', False):
        raise ValueError(f'{sc.name}: route.laps > 1 requires route.loop: true')

    for i, p in enumerate(wps):
        if abs(p.x) > w / 2 or abs(p.y) > h / 2:
            raise ValueError(
                f'{sc.name}: waypoint {i} ({p.x}, {p.y}) is outside the world')
        c = clearance((p.x, p.y, p.yaw), sc.all_boxes)
        if c <= 0.0:
            raise ValueError(f'{sc.name}: waypoint {i} overlaps an obstacle '
                             f'(clearance {c:.3f} m)')

    # Waypoints closer together than the capture radius cannot be told apart:
    # the robot would be inside both at once and the run could not say which
    # one it passed.
    cap = float(r.get('capture_radius_m', 0.45))
    ring = list(zip(wps, wps[1:] + ([wps[0]] if r.get('loop') else [])))
    for i, (a, b) in enumerate(ring):
        d = math.dist((a.x, a.y), (b.x, b.y))
        if d < 2 * cap:
            raise ValueError(
                f'{sc.name}: waypoints {i} and {(i + 1) % len(wps)} are {d:.2f} m '
                f'apart, closer than twice the {cap:.2f} m capture radius; a pass '
                f'through one cannot be distinguished from a pass through the other')

    # No slam_envelope check here, unlike a goal scenario. Route trials run in
    # ground_truth only — every metric is a distance between where the robot
    # really was and where a waypoint really is, and under slam the map frame is
    # anchored at the start pose, so those numbers would measure localisation
    # error rather than driving accuracy. The costmap comes from the static map,
    # which covers the whole world, so lidar visibility does not constrain where
    # a waypoint may sit.


def _pose(d: dict) -> Pose:
    return Pose(float(d['x']), float(d['y']), float(d.get('yaw', 0.0)))


def load(path: str | Path) -> Scenario:
    raw = yaml.safe_load(Path(path).read_text())
    world = raw.get('world', {})
    size = tuple(float(v) for v in world.get('size', [12.0, 8.0]))
    sc = Scenario(
        name=raw['name'],
        size=(size[0], size[1]),
        start=_pose(raw['start']),
        goal=_pose(raw['goal']),
        obstacles=[Box(float(o['x']), float(o['y']), float(o['sx']), float(o['sy']))
                   for o in raw.get('obstacles', [])],
        timeout_s=float(raw.get('timeout_s', 90.0)),
        explore=raw.get('explore'),
        route=raw.get('route'),
        rtf=float(world.get('rtf', 0.5)),
        max_step=float(world.get('max_step', 0.001)),
        description=raw.get('description', ''),
    )
    validate(sc)
    return sc


def validate(sc: Scenario) -> None:
    """Fail loudly at generation time rather than producing a scenario that can
    never succeed and then reading the failure as a Nav2 result."""
    from .geometry import clearance

    w, h = sc.size
    if sc.explore and sc.route:
        raise ValueError(f'{sc.name}: a scenario is either an exploration or a '
                         f'route, not both')
    if sc.explore or sc.route:
        checked = (('start', sc.start),)
    else:
        checked = (('start', sc.start), ('goal', sc.goal))
    for label, p in checked:
        if abs(p.x) > w / 2 or abs(p.y) > h / 2:
            raise ValueError(f'{sc.name}: {label} ({p.x}, {p.y}) is outside the world')
        c = clearance((p.x, p.y, p.yaw), sc.all_boxes)
        if c <= 0.0:
            raise ValueError(
                f'{sc.name}: {label} overlaps an obstacle (clearance {c:.3f} m)')
    if sc.explore:
        e = sc.explore
        if float(e.get('duration_s', 0)) <= 0:
            raise ValueError(f'{sc.name}: explore.duration_s must be positive')
        if not 0 < float(e.get('target_coverage', 0.95)) <= 1.0:
            raise ValueError(f'{sc.name}: explore.target_coverage must be in (0, 1]')
        return
    if sc.route:
        _validate_route(sc)
        return
    if math.dist((sc.start.x, sc.start.y), (sc.goal.x, sc.goal.y)) < 0.5:
        raise ValueError(f'{sc.name}: goal is within 0.5 m of start; nothing to measure')

    # Every scenario must be runnable in all three localisation modes, so that
    # ground_truth / slam / amcl are comparable on identical geometry. slam is
    # the binding constraint: see slam_envelope.
    x0, y0, x1, y1 = slam_envelope(sc)
    m = 0.30                       # keep the goal off the very edge of the map
    if not (x0 + m <= sc.goal.x <= x1 - m and y0 + m <= sc.goal.y <= y1 - m):
        raise ValueError(
            f'{sc.name}: goal ({sc.goal.x}, {sc.goal.y}) lies outside what the '
            f'lidar can observe from the start pose — x [{x0:.2f}, {x1:.2f}], '
            f'y [{y0:.2f}, {y1:.2f}]. Under slam the costmap never covers it, so '
            f'the scenario cannot run in every mode. Move a wall within '
            f'{LIDAR_MAX_RANGE} m behind the goal, or bring the goal closer.')

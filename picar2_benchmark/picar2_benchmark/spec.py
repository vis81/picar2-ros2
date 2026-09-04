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
    for label, p in (('start', sc.start), ('goal', sc.goal)):
        if abs(p.x) > w / 2 or abs(p.y) > h / 2:
            raise ValueError(f'{sc.name}: {label} ({p.x}, {p.y}) is outside the world')
        c = clearance((p.x, p.y, p.yaw), sc.all_boxes)
        if c <= 0.0:
            raise ValueError(
                f'{sc.name}: {label} overlaps an obstacle (clearance {c:.3f} m)')
    if math.dist((sc.start.x, sc.start.y), (sc.goal.x, sc.goal.y)) < 0.5:
        raise ValueError(f'{sc.name}: goal is within 0.5 m of start; nothing to measure')

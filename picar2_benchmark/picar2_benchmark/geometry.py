"""Footprint geometry and analytic clearance.

Clearance is computed from the known obstacle rectangles rather than from a
contact sensor: the scenario generator produced that geometry, and ground truth
gives the exact pose, so the distance is exact and needs no simulation support.

Caveat to carry into any report: this is *footprint* clearance in the plane. It
ignores the lidar mast and the 3D hull, so it is not a physical collision check.
"""
from __future__ import annotations

import math

from .spec import FOOTPRINT, Box


def transform(pose: tuple[float, float, float],
              poly: list[tuple[float, float]] = None) -> list[tuple[float, float]]:
    """Place the footprint polygon at an SE(2) pose."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + px * c - py * s, y + px * s + py * c)
            for px, py in (poly if poly is not None else FOOTPRINT)]


def _rect_corners(b: Box) -> list[tuple[float, float]]:
    x0, y0, x1, y1 = b.bounds
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def _seg_seg_distance(p1, p2, q1, q2) -> float:
    """Minimum distance between two segments."""
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def point_seg(p, a, b):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 == 0.0:
            return math.dist(p, a)
        t = clamp(((p[0] - ax) * dx + (p[1] - ay) * dy) / L2, 0.0, 1.0)
        return math.dist(p, (ax + t * dx, ay + t * dy))

    return min(point_seg(p1, q1, q2), point_seg(p2, q1, q2),
               point_seg(q1, p1, p2), point_seg(q2, p1, p2))


def _overlap(a: list, b: list) -> bool:
    """Separating-axis test for two convex polygons."""
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            nx, ny = -(y2 - y1), (x2 - x1)
            pa = [nx * px + ny * py for px, py in a]
            pb = [nx * px + ny * py for px, py in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def poly_box_distance(poly: list[tuple[float, float]], box: Box) -> float:
    """Signed distance: positive when clear, negative (penetration depth,
    approximated by the deepest vertex incursion) when overlapping."""
    rect = _rect_corners(box)
    if _overlap(poly, rect):
        x0, y0, x1, y1 = box.bounds
        worst = 0.0
        for px, py in poly:
            if x0 <= px <= x1 and y0 <= py <= y1:
                worst = max(worst, min(px - x0, x1 - px, py - y0, y1 - py))
        return -max(worst, 1e-6)
    best = math.inf
    for i in range(len(poly)):
        for j in range(len(rect)):
            best = min(best, _seg_seg_distance(
                poly[i], poly[(i + 1) % len(poly)],
                rect[j], rect[(j + 1) % len(rect)]))
    return best


def clearance(pose: tuple[float, float, float], boxes: list[Box]) -> float:
    """Smallest signed distance from the footprint at `pose` to any obstacle."""
    poly = transform(pose)
    return min((poly_box_distance(poly, b) for b in boxes), default=math.inf)

"""Map where the planner treats left and right turns differently.

Smac Hybrid returns a cusp-free path for some left turns and a cusp-containing
path for the exact mirror image, on a symmetric costmap. This sweeps the family
of turns the robot could actually execute and reports where that happens.

A turn is parameterised as an arc, not as a loose goal pose: from the origin
heading +x, an arc of radius R through angle theta ends at
(R sin t, R(1 - cos t)) heading t, and its mirror is the same with y and the
heading negated. Holding the goal position fixed while varying only the goal
heading - which an earlier ad-hoc sweep did - mixes two variables and cannot
say whether the angle or the displacement is what matters.

Every query is answered by the same planner instance against the same costmap
within a second or two of its mirror, so the two differ only in sign.
"""
from __future__ import annotations

import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def path_cusps(path) -> int:
    """Direction reversals in a path: consecutive segments that oppose."""
    n = 0
    for a, b, c in zip(path.poses, path.poses[1:], path.poses[2:]):
        v1 = (b.pose.position.x - a.pose.position.x,
              b.pose.position.y - a.pose.position.y)
        v2 = (c.pose.position.x - b.pose.position.x,
              c.pose.position.y - b.pose.position.y)
        if v1[0] * v2[0] + v1[1] * v2[1] < 0:
            n += 1
    return n


def path_length(path) -> float:
    return sum(math.dist((a.pose.position.x, a.pose.position.y),
                         (b.pose.position.x, b.pose.position.y))
               for a, b in zip(path.poses, path.poses[1:]))


def _pose(x, y, yaw, frame='map') -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def arc_endpoint(radius: float, theta: float, sign: int) -> tuple[float, float, float]:
    """Where an arc of this radius and angle ends, for a left (+1) or right turn."""
    return (radius * math.sin(theta),
            sign * radius * (1.0 - math.cos(theta)),
            sign * theta)


class Sweeper(Node):
    def __init__(self):
        super().__init__('bench_plan_sweep')
        self.cli = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def query(self, goal, start=(0.0, 0.0, 0.0)):
        g = ComputePathToPose.Goal()
        g.start = _pose(*start)
        g.goal = _pose(*goal)
        g.use_start = True
        f = self.cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=20.0)
        gh = f.result()
        if gh is None or not gh.accepted:
            return None, None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=25.0)
        res = rf.result()
        if res is None or not res.result.path.poses:
            return None, None
        return path_cusps(res.result.path), path_length(res.result.path)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description='Sweep mirrored turns through the planner.')
    ap.add_argument('--radii', default='0.6,0.8,1.2',
                    help='turn radii in metres (min_turning_radius is 0.5)')
    ap.add_argument('--angles', default='15,30,45,60,75,90,105,120,150,180',
                    help='turn angles in degrees')
    a = ap.parse_args(argv)
    radii = [float(v) for v in a.radii.split(',')]
    angles = [float(v) for v in a.angles.split(',')]

    rclpy.init()
    n = Sweeper()
    if not n.cli.wait_for_server(timeout_sec=30.0):
        print('  compute_path_to_pose not available - is a stack running?')
        return 1

    print(f"  start (0,0,0); goals are mirrored arc endpoints\n")
    print(f"  {'R':>5s} {'angle':>6s} {'goal x':>7s} {'|y|':>6s} "
          f"{'L cusp':>7s} {'R cusp':>7s} {'L len':>7s} {'R len':>7s}  verdict")
    asym = []
    for R in radii:
        for deg in angles:
            th = math.radians(deg)
            gl = arc_endpoint(R, th, +1)
            gr = arc_endpoint(R, th, -1)
            cl, ll = n.query(gl)
            cr, lr = n.query(gr)
            if cl is None or cr is None:
                print(f"  {R:5.2f} {deg:6.0f} {gl[0]:7.2f} {abs(gl[1]):6.2f} "
                      f"{'-':>7s} {'-':>7s} {'-':>7s} {'-':>7s}  no path")
                continue
            bad = cl != cr
            asym.append((R, deg, cl, cr)) if bad else None
            print(f"  {R:5.2f} {deg:6.0f} {gl[0]:7.2f} {abs(gl[1]):6.2f} "
                  f"{cl:7d} {cr:7d} {ll:7.2f} {lr:7.2f}"
                  f"{'  <-- ASYMMETRIC' if bad else ''}")
    print(f"\n  {len(asym)} asymmetric of {len(radii)*len(angles)} pairs")
    if asym:
        by_angle = sorted({d for _, d, _, _ in asym})
        by_radius = sorted({r for r, _, _, _ in asym})
        print(f"  angles affected: {by_angle}")
        print(f"  radii affected:  {by_radius}")
    rclpy.shutdown()
    return 0

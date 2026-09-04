"""Pre-trial gates.

Every one of these exists because the failure it catches has already happened
and was recorded as legitimate data. A trial that fails a gate is discarded, not
counted: a degraded simulator produces confident-looking numbers that mean
nothing, and averaging them in is worse than having no data.
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class GateFailure(Exception):
    """Raised when the simulator is not fit to measure. Never a Nav2 result."""


class GateContext(Node):
    def __init__(self):
        super().__init__('bench_gates', parameter_overrides=[
            # must be set at construction, or the clock binds to wall time and
            # every TF lookup compares sim stamps against wall-clock instants
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.gt: Odometry | None = None
        self.costmap: OccupancyGrid | None = None
        self.create_subscription(Odometry, '/gt/odom', self._on_gt, 20)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._on_costmap, LATCHED)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_gt(self, m):
        self.gt = m

    def _on_costmap(self, m):
        self.costmap = m

    def spin(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def gt_pose(self) -> tuple[float, float, float]:
        p = self.gt.pose.pose.position
        q = self.gt.pose.pose.orientation
        return (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z)))


def wait_for_ground_truth(ctx: GateContext, timeout: float = 60.0) -> None:
    end = time.time() + timeout
    while time.time() < end and ctx.gt is None:
        rclpy.spin_once(ctx, timeout_sec=0.1)
    if ctx.gt is None:
        raise GateFailure('no ground truth on /gt/odom')


def gate_spawn_pose(ctx: GateContext, start, xy_tol=0.02, yaw_tol=math.radians(2)) -> None:
    """The robot is where the scenario says it is.

    Also the detector for a model-frame offset: if the Gazebo model frame were
    base_link rather than base_footprint, every pose would be biased 0.116 m and
    every later number would be quietly wrong.
    """
    x, y, yaw = ctx.gt_pose()
    d = math.dist((x, y), (start.x, start.y))
    dyaw = abs(math.atan2(math.sin(yaw - start.yaw), math.cos(yaw - start.yaw)))
    if d > xy_tol or dyaw > yaw_tol:
        raise GateFailure(
            f'spawn pose off by {d*1000:.0f} mm / {math.degrees(dyaw):.1f} deg '
            f'(at {x:.3f},{y:.3f} yaw {math.degrees(yaw):.1f}, '
            f'expected {start.x},{start.y} yaw {math.degrees(start.yaw):.1f})')


def gate_settle(ctx: GateContext, timeout=20.0, still=1.0, eps=0.01) -> None:
    """Spawning drops the robot; do not start the clock during the bounce."""
    end = time.time() + timeout
    quiet_since = None
    while time.time() < end:
        rclpy.spin_once(ctx, timeout_sec=0.05)
        if ctx.gt is None:
            continue
        v = ctx.gt.twist.twist
        speed = math.hypot(v.linear.x, v.linear.y)
        if speed < eps:
            quiet_since = quiet_since or time.time()
            if time.time() - quiet_since >= still:
                return
        else:
            quiet_since = None
    raise GateFailure('robot never settled')


def gate_motion(ctx: GateContext, speed=0.25, seconds=4.0, min_move=0.05) -> None:
    """Command motion and require ground truth to move.

    `ros2 control list_controllers` is not sufficient: it has answered from a
    leftover controller_manager belonging to the previous simulation while the
    current robot could not move at all.
    """
    wait_for_ground_truth(ctx)
    x0, y0, _ = ctx.gt_pose()
    t = Twist()
    t.linear.x = speed
    end = time.time() + seconds
    while time.time() < end:
        ctx.cmd.publish(t)
        rclpy.spin_once(ctx, timeout_sec=0.02)
    t.linear.x = 0.0
    for _ in range(20):
        ctx.cmd.publish(t)
        rclpy.spin_once(ctx, timeout_sec=0.02)
    ctx.spin(1.0)
    x1, y1, _ = ctx.gt_pose()
    moved = math.dist((x0, y0), (x1, y1))
    if moved < min_move:
        raise GateFailure(
            f'robot does not respond to velocity commands (moved {moved*1000:.0f} mm '
            f'under {speed} m/s for {seconds}s)')


def gate_costmap(ctx: GateContext, sc, mode: str, timeout=90.0, min_free=400) -> None:
    """The global costmap is populated and actually covers the scenario.

    Without the coverage half, a goal can be sent while the costmap is still the
    5x5 m default at the origin — nav2.yaml declares no width/height/origin —
    and the resulting planner failure looks like a navigation result.
    """
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(ctx, timeout_sec=0.1)
        m = ctx.costmap
        if m is None:
            continue
        a = np.frombuffer(bytes(m.data), dtype=np.int8)
        if int(((a >= 0) & (a <= 50)).sum()) < min_free:
            continue
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        w = m.info.width * m.info.resolution
        h = m.info.height * m.info.resolution
        covered = all(ox <= p.x <= ox + w and oy <= p.y <= oy + h
                      for p in (sc.start, sc.goal))
        if covered:
            return
        if mode != 'slam':
            raise GateFailure(
                f'costmap {w:.1f}x{h:.1f} at ({ox:.1f},{oy:.1f}) does not cover '
                f'start/goal — static map not applied?')
        # under SLAM the map grows, so keep waiting for it to reach the goal
    raise GateFailure('global costmap never covered start and goal')


def gate_single_map_odom(ctx: GateContext, mode: str) -> None:
    """Exactly one publisher of map->odom. Two silently fight and the resulting
    pose is neither, which is indistinguishable from bad navigation."""
    ctx.spin(2.0)
    try:
        ctx.buf.lookup_transform('map', 'odom', rclpy.time.Time(),
                                 timeout=Duration(seconds=3.0))
    except Exception as e:                                   # noqa: BLE001
        raise GateFailure(f'no map->odom in mode={mode}: {e}') from e

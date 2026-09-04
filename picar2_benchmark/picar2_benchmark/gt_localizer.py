#!/usr/bin/env python3
"""Ground-truth pose plumbing for the benchmark.

Runs in every localisation mode, but does different work in each:

  ground_truth : publishes map->odom, so localisation error is zero by
                 construction and any failure is the planner or the controller.
  slam / amcl  : publishes no TF at all — cartographer or AMCL owns map->odom —
                 but still republishes ground truth so the *metrics* are
                 measured against reality rather than against what the robot
                 believes. The gap between the two is the localisation error,
                 which is the whole point of being able to run both arms.

In amcl mode it also seeds /initialpose once from ground truth, so AMCL starts
from the scenario's start pose reproducibly instead of needing RViz.

Ground truth arrives on /gt/odom from a gz-sim-odometry-publisher-system plugin
on the robot model (simulation only). That pose is world-absolute and its frames
are named explicitly, which the /world/<w>/pose/info route could not provide:
entity identity lives in pose.name there, while the ros_gz Pose_V->TFMessage
converter reads header.data, so every bridged transform came out frameless.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               TransformStamped)
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _compose(a: tuple, b: tuple) -> tuple:
    """SE(2) composition a . b."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, at + bt)


def _inverse(a: tuple) -> tuple:
    ax, ay, at = a
    c, s = math.cos(-at), math.sin(-at)
    return (-(c * ax - s * ay), -(s * ax + c * ay), -at)


class GtLocalizer(Node):
    def __init__(self):
        super().__init__('gt_localizer')
        self.declare_parameter('mode', 'ground_truth')       # ground_truth|slam|amcl
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('transform_tolerance', 0.3)   # matches nav2.yaml
        self.declare_parameter('lag', 0.10)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.mode = self.get_parameter('mode').value
        self.tol = float(self.get_parameter('transform_tolerance').value)
        self.lag = float(self.get_parameter('lag').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.owns_tf = (self.mode == 'ground_truth')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self) if self.owns_tf else None

        self._samples: list[tuple[float, tuple]] = []
        self._correction: tuple | None = None
        self._seeded = False

        self.create_subscription(Odometry, '/gt/odom', self._on_gt, 20)
        self.gt_pub = self.create_publisher(PoseStamped, '/gt/pose', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', latched)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'gt_localizer mode={self.mode} '
            f'({"publishing map->odom" if self.owns_tf else "metrics only, no TF"})')

    # ── ground truth in ─────────────────────────────────────────────────
    def _on_gt(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        gt = (p.x, p.y, _yaw(q))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._samples.append((t, gt))
        if len(self._samples) > 200:
            self._samples.pop(0)

        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = 'map'      # gt_odom origin is the world origin
        out.pose = msg.pose.pose
        self.gt_pub.publish(out)

        if self.mode == 'amcl' and not self._seeded:
            self._seed_amcl(msg)

    def _seed_amcl(self, msg: Odometry):
        """Start AMCL from the true pose, so the run is reproducible instead of
        depending on someone clicking '2D Pose Estimate'."""
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.pose = msg.pose.pose
        m.pose.covariance[0] = m.pose.covariance[7] = 0.01
        m.pose.covariance[35] = 0.02
        self.init_pub.publish(m)
        self._seeded = True
        self.get_logger().info('seeded /initialpose from ground truth')

    # ── map->odom out ───────────────────────────────────────────────────
    def _tick(self):
        if self.owns_tf:
            self._update_correction()
            self._broadcast()

    def _update_correction(self):
        """map->odom = GT(map->base) . (odom->base)^-1, evaluated on a lagged
        sample so the EKF transform for that stamp is already available."""
        if not self._samples:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        target = now - self.lag
        stamp, gt = min(self._samples, key=lambda s: abs(s[0] - target))
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception as e:                       # noqa: BLE001
            self.get_logger().warn(f'no {self.odom_frame}->{self.base_frame}: {e}',
                                   throttle_duration_sec=5.0)
            return
        t, q = tf.transform.translation, tf.transform.rotation
        self._correction = _compose(gt, _inverse((t.x, t.y, _yaw(q))))

    def _broadcast(self):
        """Publish on a timer and post-date the stamp, exactly as AMCL does.
        Publishing only when ground truth arrives would starve Nav2's lookups on
        any bridge hiccup, and an un-post-dated stamp gets rejected as stale."""
        if self._correction is None:
            return
        x, y, yaw = self._correction
        m = TransformStamped()
        stamp = self.get_clock().now() + Duration(seconds=self.tol)
        m.header.stamp = stamp.to_msg()
        m.header.frame_id = 'map'
        m.child_frame_id = self.odom_frame
        m.transform.translation.x = x
        m.transform.translation.y = y
        m.transform.rotation.z = math.sin(yaw / 2.0)
        m.transform.rotation.w = math.cos(yaw / 2.0)
        self.br.sendTransform(m)


def main():
    rclpy.init()
    node = GtLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()

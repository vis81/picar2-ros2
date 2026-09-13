#!/usr/bin/python3
"""Motion-compensate a LaserScan using odometry.

The LD19 takes ~100 ms per revolution and the driver stamps the whole scan
with the time the revolution finished. At 0.85 m/s the first beams are
therefore placed up to 8.5 cm from where they were really measured, and a
1 rad/s turn smears a 4 m wall by 40 cm. AMCL, slam_toolbox and the
costmaps all take the scan as instantaneous.

This node moves every beam to where the sensor was when that beam was
measured, using the odom -> laser transform (the EKF at 30 Hz), and
re-bins the result on the original angle grid at the stamp time. The
output is still a LaserScan in the laser frame, so nothing downstream
changes. If TF is not available for a scan it is passed through untouched:
a stale or missing scan is worse than a skewed one.

Beam timing (see ldlidar_component.cpp): the stamp is the end of the
revolution, time_increment = scan_time / (bins - 1), and with
rot_verse CCW the driver reverses the native (clockwise) order, so ROS
beam j was measured at stamp - j * time_increment. reverse_beam_time
covers the CW case.
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


class ScanDeskew(Node):
    def __init__(self):
        super().__init__('scan_deskew')
        self.declare_parameter('input', '/lidar_node/scan_raw')
        self.declare_parameter('output', '/lidar_node/scan')
        self.declare_parameter('fixed_frame', 'odom')
        # Poses are looked up at this many instants per scan and the beams
        # between them interpolated: 16 lookups at 10 Hz is nothing, 455
        # would be.
        self.declare_parameter('segments', 16)
        # True: beam j measured at stamp - j*dt (driver rot_verse CCW).
        # False: beam j measured at stamp - (n-1-j)*dt.
        self.declare_parameter('reverse_beam_time', True)
        # Beams newer than the newest transform are extrapolated; beyond
        # this the transform is simply stale (EKF hiccup) - pass through.
        self.declare_parameter('max_lead', 0.15)

        self.fixed = self.get_parameter('fixed_frame').value
        self.segments = int(self.get_parameter('segments').value)
        self.reverse = bool(self.get_parameter('reverse_beam_time').value)
        self.max_lead = float(self.get_parameter('max_lead').value)

        self.buf = Buffer(cache_time=Duration(seconds=5.0))
        # Own thread for /tf, so a slow scan callback never starves it.
        self.listener = TransformListener(self.buf, self, spin_thread=True)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(
            LaserScan, self.get_parameter('output').value, qos)
        self.create_subscription(
            LaserScan, self.get_parameter('input').value, self.on_scan, qos)
        self.n_pass = 0
        self.n_ok = 0
        self.create_timer(30.0, self.report)

    def report(self):
        if self.n_pass:
            self.get_logger().warn(
                f'{self.n_pass} of {self.n_pass + self.n_ok} scans passed '
                f'through without deskew (TF unavailable)')
        self.n_pass = self.n_ok = 0

    def pose_at(self, frame, t):
        tf = self.buf.lookup_transform(self.fixed, frame, t)
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n < 2 or msg.time_increment <= 0.0:
            self.pub.publish(msg)
            return
        dt = msg.time_increment
        t_end = Time.from_msg(msg.header.stamp)
        frame = msg.header.frame_id.lstrip('/')
        try:
            # The scan is stamped "now" by the driver while the EKF's
            # transform runs ~40 ms behind the clock, so the stamp itself is
            # never in the buffer yet. Reference the output to the newest
            # transform there is (a few tens of ms old - well inside
            # everyone's transform_tolerance) and extrapolate the handful
            # of beams newer than that from the last two poses.
            latest = self.buf.lookup_transform(self.fixed, frame, Time())
            t_ref = Time.from_msg(latest.header.stamp)
            if t_ref > t_end:
                t_ref = t_end
            lead = (t_end - t_ref).nanoseconds * 1e-9   # beams newer than t_ref
            if lead > self.max_lead:
                raise TransformException('transform too old')
            xe, ye, ye_yaw = self.pose_at(frame, t_ref)
            # Sample poses back through the revolution from t_ref.
            span = dt * (n - 1)
            offs = np.linspace(0.0, span, self.segments + 1)
            poses = []
            for o in offs:
                poses.append(self.pose_at(frame, t_ref - Duration(seconds=float(o))))
        except TransformException:
            self.n_pass += 1
            self.pub.publish(msg)
            return
        self.n_ok += 1
        poses = np.array(poses)                       # (S+1, 3): x, y, yaw
        # Unwrap yaw so interpolation does not jump across +-pi.
        poses[:, 2] = np.unwrap(poses[:, 2])

        r = np.asarray(msg.ranges, dtype=np.float64)
        j = np.arange(n)
        age = j * dt if self.reverse else (n - 1 - j) * dt  # s before stamp
        age = age - lead          # ...before t_ref; negative = after it
        if lead > 0.0:
            # Prepend an extrapolated pose at -lead so np.interp covers the
            # beams measured after t_ref (constant velocity over <= max_lead).
            vel = (poses[0] - poses[1]) / (offs[1] - offs[0])
            poses = np.vstack([poses[0] + vel * lead, poses])
            offs = np.concatenate([[-lead], offs])
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        # Sensor pose when each beam was measured, by interpolation.
        px = np.interp(age, offs, poses[:, 0])
        py = np.interp(age, offs, poses[:, 1])
        pyaw = np.interp(age, offs, poses[:, 2])
        ang = msg.angle_min + j * msg.angle_increment
        # Beam endpoint in the fixed frame...
        wx = px + r * np.cos(ang + pyaw)
        wy = py + r * np.sin(ang + pyaw)
        # ...seen from the sensor at the stamp time.
        dx = wx - xe
        dy = wy - ye
        c, s = math.cos(-ye_yaw), math.sin(-ye_yaw)
        lx = dx * c - dy * s
        ly = dx * s + dy * c
        nr = np.hypot(lx[ok], ly[ok])
        na = np.arctan2(ly[ok], lx[ok])
        na = np.mod(na - msg.angle_min, 2.0 * math.pi)
        nb = np.clip(np.rint(na / msg.angle_increment).astype(int), 0, n - 1)

        out = np.full(n, np.inf)
        # A bin that receives two beams keeps the nearer one, as the driver
        # does when two native beams land in one bin.
        np.minimum.at(out, nb, nr)
        res = LaserScan()
        res.header = msg.header
        res.header.stamp = t_ref.to_msg()
        res.angle_min = msg.angle_min
        res.angle_max = msg.angle_max
        res.angle_increment = msg.angle_increment
        res.time_increment = 0.0            # every beam now refers to stamp
        res.scan_time = msg.scan_time
        res.range_min = msg.range_min
        res.range_max = msg.range_max
        res.ranges = out.astype(np.float32).tolist()
        self.pub.publish(res)


def main():
    rclpy.init()
    node = ScanDeskew()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

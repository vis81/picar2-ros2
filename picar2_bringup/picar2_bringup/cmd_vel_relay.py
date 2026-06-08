#!/usr/bin/python3
"""Relay geometry_msgs/Twist from /cmd_vel to TwistStamped on the
ackermann_steering_controller reference topic."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


MAX_ANG_VEL  = 1.2   # rad/s — hardware max: v/r = 0.4/0.34 ≈ 1.16 rad/s at full lock
MAX_REV_VEL  = 0.25  # m/s  — cap reverse speed; RPPC uses same desired_linear_vel both ways


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(
            TwistStamped,
            '/ackermann_steering_controller/reference',
            10)
        self.create_subscription(Twist, 'cmd_vel', self._cb, 10)

    def _cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.twist = msg
        out.twist.angular.z = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, msg.angular.z))
        if msg.linear.x < -MAX_REV_VEL:
            out.twist.linear.x = -MAX_REV_VEL
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(CmdVelRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

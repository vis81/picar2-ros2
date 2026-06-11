#!/usr/bin/python3
"""Relay geometry_msgs/Twist from /cmd_vel to TwistStamped on the
ackermann_steering_controller reference topic."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('max_angular_vel', 1.2)
        self.declare_parameter('max_reverse_vel', 0.25)
        self.pub = self.create_publisher(
            TwistStamped,
            '/ackermann_steering_controller/reference',
            10)
        self.create_subscription(Twist, 'cmd_vel', self._cb, 10)

    def _cb(self, msg: Twist):
        max_ang = self.get_parameter('max_angular_vel').value
        max_rev = self.get_parameter('max_reverse_vel').value
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.twist = msg
        out.twist.angular.z = max(-max_ang, min(max_ang, msg.angular.z))
        if msg.linear.x < -max_rev:
            out.twist.linear.x = -max_rev
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(CmdVelRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

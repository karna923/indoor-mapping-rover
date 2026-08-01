#!/usr/bin/env python3
"""
Publishes a correctly time-stamped TwistStamped command for a fixed
duration, then stops cleanly. Used for the wheel-odom-vs-EKF heading
drift validation test, since `ros2 topic pub -r 20` sends every
message with a zero (unset) header stamp, which is worth avoiding
for a controlled test.

Usage:
    ros2 run rover_description spin_test.py --ros-args -p duration:=62.8 -p angular_z:=1.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class SpinTest(Node):
    def __init__(self):
        super().__init__('spin_test')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.declare_parameter('duration', 62.8)   # seconds, default = 10 rotations at 1 rad/s
        self.declare_parameter('angular_z', 1.0)    # rad/s
        self.declare_parameter('rate_hz', 20.0)

        self.duration = self.get_parameter('duration').value
        self.angular_z = self.get_parameter('angular_z').value
        rate_hz = self.get_parameter('rate_hz').value

        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.start_time = None
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        self.get_logger().info(
            f'spin_test: commanding angular_z={self.angular_z} rad/s for {self.duration}s'
        )

    def tick(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now

        elapsed = (now - self.start_time).nanoseconds * 1e-9

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_footprint'

        if elapsed < self.duration:
            msg.twist.angular.z = self.angular_z
        else:
            msg.twist.angular.z = 0.0
            self.pub.publish(msg)
            self.get_logger().info('spin_test: done, stop command sent')
            self.timer.cancel()
            rclpy.shutdown()
            return

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SpinTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

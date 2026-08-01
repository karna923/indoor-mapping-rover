#!/usr/bin/env python3
"""
Synthetic IMU publisher for indoor-mapping-rover.

Derives real angular velocity and linear acceleration from the robot's
validated /model/rover/odometry_ground_truth, adds gravity (rotated into the
body frame using the robot's real orientation) and Gaussian noise
matching the MPU6050's real spec sheet, then publishes sensor_msgs/Imu
on /imu.

This works around the same gz-sim Sensors-system rendering hang under
WSL2 that blocks the native LiDAR sensor (see docs/urdf_debugging_log.md
and fake_lidar_publisher.py) -- gz-sim's Sensors system gates ALL sensor
types (camera, LiDAR, IMU) behind the same render-thread init, even
sensors like IMU that don't conceptually need rendering.

Unlike the LiDAR workaround (which ray-casts against known geometry),
this publisher derives its signal directly from ground-truth physics
data Gazebo already computed via the validated diff-drive controller,
so it's a closer approximation to "what a real IMU would report" than
a from-scratch simulation would be.

Noise parameters match the values declared in the xacro's <imu> sensor
block, which were themselves derived from the MPU6050 datasheet:
  - gyro RMS noise:  ~0.05 deg/s  -> ~0.0009 rad/s stddev
  - accel noise:     ~400 ug/rtHz -> ~0.004 m/s^2 stddev (approximated)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np

IMU_FRAME = "imu_link"
PUBLISH_RATE_HZ = 100.0

GYRO_NOISE_STDDEV = 0.0009      # rad/s, matches xacro
ACCEL_NOISE_STDDEV = 0.004      # m/s^2, matches xacro
GRAVITY = 9.80665                # m/s^2


class FakeImuPublisher(Node):
    def __init__(self):
        super().__init__('fake_imu_publisher')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(Imu, '/imu', qos)
        self.sub = self.create_subscription(
            Odometry, '/model/rover/odometry_ground_truth', self.odom_callback, qos
        )

        self._last_odom = None
        self._last_time = None
        self._rng = np.random.default_rng()

        period = 1.0 / PUBLISH_RATE_HZ
        self.timer = self.create_timer(period, self.publish_imu)
        self.get_logger().info(
            f'fake_imu_publisher started: publishing /imu @ {PUBLISH_RATE_HZ} Hz '
            f'(gyro stddev={GYRO_NOISE_STDDEV} rad/s, accel stddev={ACCEL_NOISE_STDDEV} m/s^2)'
        )

    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now()

        if self._last_odom is not None and self._last_time is not None:
            dt = (now - self._last_time).nanoseconds * 1e-9
            if dt > 1e-4:
                # finite-difference linear acceleration from successive odom velocities
                vx0 = self._last_odom.twist.twist.linear.x
                vy0 = self._last_odom.twist.twist.linear.y
                vx1 = msg.twist.twist.linear.x
                vy1 = msg.twist.twist.linear.y
                self._ax = (vx1 - vx0) / dt
                self._ay = (vy1 - vy0) / dt
            else:
                self._ax = 0.0
                self._ay = 0.0
        else:
            self._ax = 0.0
            self._ay = 0.0

        self._last_odom = msg
        self._last_time = now

    def publish_imu(self):
        if self._last_odom is None:
            return

        q = self._last_odom.pose.pose.orientation
        wz = self._last_odom.twist.twist.angular.z

        # Rotate gravity (0,0,-g in world frame) into the body frame using
        # the robot's real orientation quaternion, so a level, stationary
        # robot correctly reads ~+g on its body Z axis, matching how a
        # real accelerometer responds to gravity (see MPU6050 tutorials).
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        # world gravity vector
        gx, gy, gz = 0.0, 0.0, -GRAVITY
        # rotate world->body using inverse quaternion rotation
        ax_g, ay_g, az_g = self._rotate_vector_by_quat_inverse(gx, gy, gz, qx, qy, qz, qw)
        # accelerometer reads the *reaction* to gravity (i.e. +g when level and still)
        ax_g, ay_g, az_g = -ax_g, -ay_g, -az_g

        ax = getattr(self, '_ax', 0.0) + ax_g
        ay = getattr(self, '_ay', 0.0) + ay_g
        az = az_g

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = IMU_FRAME

        msg.orientation = q
        msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        msg.angular_velocity.x = 0.0 + self._rng.normal(0.0, GYRO_NOISE_STDDEV)
        msg.angular_velocity.y = 0.0 + self._rng.normal(0.0, GYRO_NOISE_STDDEV)
        msg.angular_velocity.z = wz + self._rng.normal(0.0, GYRO_NOISE_STDDEV)
        msg.angular_velocity_covariance = [
            GYRO_NOISE_STDDEV ** 2, 0, 0,
            0, GYRO_NOISE_STDDEV ** 2, 0,
            0, 0, GYRO_NOISE_STDDEV ** 2,
        ]

        msg.linear_acceleration.x = ax + self._rng.normal(0.0, ACCEL_NOISE_STDDEV)
        msg.linear_acceleration.y = ay + self._rng.normal(0.0, ACCEL_NOISE_STDDEV)
        msg.linear_acceleration.z = az + self._rng.normal(0.0, ACCEL_NOISE_STDDEV)
        msg.linear_acceleration_covariance = [
            ACCEL_NOISE_STDDEV ** 2, 0, 0,
            0, ACCEL_NOISE_STDDEV ** 2, 0,
            0, 0, ACCEL_NOISE_STDDEV ** 2,
        ]

        self.pub.publish(msg)

    @staticmethod
    def _rotate_vector_by_quat_inverse(vx, vy, vz, qx, qy, qz, qw):
        """Rotate vector v from world frame into body frame using q^-1 * v * q."""
        # inverse of a unit quaternion is its conjugate
        iqx, iqy, iqz, iqw = -qx, -qy, -qz, qw

        # v as pure quaternion
        # t = 2 * cross(q.xyz, v)
        tx = 2.0 * (iqy * vz - iqz * vy)
        ty = 2.0 * (iqz * vx - iqx * vz)
        tz = 2.0 * (iqx * vy - iqy * vx)

        # v' = v + q.w * t + cross(q.xyz, t)
        rx = vx + iqw * tx + (iqy * tz - iqz * ty)
        ry = vy + iqw * ty + (iqz * tx - iqx * tz)
        rz = vz + iqw * tz + (iqx * ty - iqy * tx)
        return rx, ry, rz


def main():
    rclpy.init()
    node = FakeImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

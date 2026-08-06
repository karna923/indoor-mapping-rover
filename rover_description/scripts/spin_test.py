#!/usr/bin/env python3
"""
IMU/EKF validation: spin test.

Commands the rover to perform N_ROTATIONS full in-place rotations per trial,
for N_TRIALS trials, and logs the heading (yaw) reported by ground-truth
odometry vs the EKF-filtered odometry at the end of each trial.

Output: a CSV with one row per trial:
  trial, gt_start_yaw, gt_end_yaw, gt_delta, ekf_start_yaw, ekf_end_yaw, ekf_delta, error_deg
"""

import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

N_TRIALS = 10
N_ROTATIONS = 5
ANGULAR_VEL = 0.5  # rad/s
SETTLE_TIME = 2.0  # seconds to wait after stopping
INTER_TRIAL_PAUSE = 3.0  # seconds between trials

GT_TOPIC = "/model/rover/odometry_ground_truth"
EKF_TOPIC = "/odometry/filtered"
CMD_VEL_TOPIC = "/diff_drive_controller/cmd_vel"

OUTPUT_CSV = "spin_test_results.csv"


def yaw_from_quaternion(q):
    # standard yaw extraction from quaternion, z-axis rotation only (2D mode)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SpinTestNode(Node):
    def __init__(self):
        super().__init__("spin_test_node")

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.latest_gt_yaw = None
        self.latest_ekf_yaw = None
        self.gt_unwrapped = 0.0
        self.ekf_unwrapped = 0.0
        self._gt_prev_raw = None
        self._ekf_prev_raw = None
        self.tracking = False

        self.create_subscription(Odometry, GT_TOPIC, self._gt_cb, best_effort)
        self.create_subscription(Odometry, EKF_TOPIC, self._ekf_cb, best_effort)
        self.cmd_pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)

        self.get_logger().info("Waiting for odometry topics to publish...")

    def _unwrap(self, raw_yaw, prev_raw, unwrapped_accum):
        if prev_raw is None:
            return raw_yaw, raw_yaw
        diff = raw_yaw - prev_raw
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
        new_unwrapped = unwrapped_accum + diff
        return raw_yaw, new_unwrapped

    def _gt_cb(self, msg):
        raw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest_gt_yaw = raw
        if self.tracking:
            self._gt_prev_raw, self.gt_unwrapped = self._unwrap(
                raw, self._gt_prev_raw, self.gt_unwrapped
            )

    def _ekf_cb(self, msg):
        raw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest_ekf_yaw = raw
        if self.tracking:
            self._ekf_prev_raw, self.ekf_unwrapped = self._unwrap(
                raw, self._ekf_prev_raw, self.ekf_unwrapped
            )

    def wait_for_odom(self, timeout=15.0):
        start = time.time()
        while rclpy.ok() and (self.latest_gt_yaw is None or self.latest_ekf_yaw is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                raise RuntimeError(
                    f"Timed out waiting for odom on {GT_TOPIC} and {EKF_TOPIC}. "
                    "Check topic names match your actual setup."
                )

    def send_cmd(self, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def run_trial(self, trial_num):
        self.get_logger().info(f"--- Trial {trial_num+1}/{N_TRIALS} ---")

        # reset unwrap tracking, start fresh from current raw yaw
        rclpy.spin_once(self, timeout_sec=0.1)
        self._gt_prev_raw = self.latest_gt_yaw
        self._ekf_prev_raw = self.latest_ekf_yaw
        self.gt_unwrapped = self.latest_gt_yaw
        self.ekf_unwrapped = self.latest_ekf_yaw
        gt_start = self.latest_gt_yaw
        ekf_start = self.latest_ekf_yaw

        self.tracking = True

        target_rad = 2 * math.pi * N_ROTATIONS
        self.get_logger().info(
            f"Spinning at {ANGULAR_VEL} rad/s until ground truth reports "
            f"{N_ROTATIONS} full rotations ({math.degrees(target_rad):.0f} deg)..."
        )

        rate_hz = 20
        watchdog_start = time.time()
        watchdog_timeout_s = (target_rad / ANGULAR_VEL) * 20  # generous margin
        while rclpy.ok():
            self.send_cmd(ANGULAR_VEL)
            rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)

            rotated_so_far = self.gt_unwrapped - gt_start
            if rotated_so_far >= target_rad:
                break

            if time.time() - watchdog_start > watchdog_timeout_s:
                self.get_logger().warn(
                    "Watchdog timeout waiting for target rotation. "
                    "Check that cmd_vel is actually driving the robot."
                )
                break

        # stop
        self.send_cmd(0.0)
        self.get_logger().info(f"Stopped. Settling for {SETTLE_TIME}s...")
        settle_end = time.time() + SETTLE_TIME
        while time.time() < settle_end:
            self.send_cmd(0.0)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.tracking = False

        gt_end = self.latest_gt_yaw
        ekf_end = self.latest_ekf_yaw
        gt_delta_deg = math.degrees(self.gt_unwrapped - gt_start)
        ekf_delta_deg = math.degrees(self.ekf_unwrapped - ekf_start)

        commanded_deg = 360.0 * N_ROTATIONS
        # error of EKF's reported rotation vs ground truth's reported rotation
        # (ground truth vs commanded tells you about wheel slip / physics,
        error_deg = ekf_delta_deg - gt_delta_deg

        self.get_logger().info(
            f"GT delta: {gt_delta_deg:.2f} deg | EKF delta: {ekf_delta_deg:.2f} deg | "
            f"EKF error vs GT: {error_deg:.2f} deg | (commanded: {commanded_deg:.1f} deg)"
        )

        return {
            "trial": trial_num + 1,
            "gt_start_yaw": gt_start,
            "gt_end_yaw": gt_end,
            "gt_delta_deg": gt_delta_deg,
            "ekf_start_yaw": ekf_start,
            "ekf_end_yaw": ekf_end,
            "ekf_delta_deg": ekf_delta_deg,
            "error_deg": error_deg,
        }

    def run_all(self):
        self.wait_for_odom()
        results = []
        for i in range(N_TRIALS):
            result = self.run_trial(i)
            results.append(result)
            if i < N_TRIALS - 1:
                self.get_logger().info(f"Pausing {INTER_TRIAL_PAUSE}s before next trial...")
                pause_end = time.time() + INTER_TRIAL_PAUSE
                while time.time() < pause_end:
                    rclpy.spin_once(self, timeout_sec=0.1)
        return results


def write_csv(results, path):
    fieldnames = [
        "trial", "gt_start_yaw", "gt_end_yaw", "gt_delta_deg",
        "ekf_start_yaw", "ekf_end_yaw", "ekf_delta_deg", "error_deg",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)


def print_summary(results):
    errors = [r["error_deg"] for r in results]
    mean_err = sum(errors) / len(errors)
    variance = sum((e - mean_err) ** 2 for e in errors) / len(errors)
    stddev = math.sqrt(variance)
    print("\n=== SUMMARY ===")
    print(f"Trials: {len(errors)}")
    print(f"Mean EKF error vs ground truth: {mean_err:.2f} deg over {N_ROTATIONS} rotations")
    print(f"Std dev: {stddev:.2f} deg")
    print(f"Min / Max error: {min(errors):.2f} / {max(errors):.2f} deg")
    print(f"Per-rotation mean error: {mean_err / N_ROTATIONS:.3f} deg/rotation")


def main():
    rclpy.init()
    node = SpinTestNode()
    try:
        results = node.run_all()
        write_csv(results, OUTPUT_CSV)
        print_summary(results)
        print(f"\nFull results written to {OUTPUT_CSV}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
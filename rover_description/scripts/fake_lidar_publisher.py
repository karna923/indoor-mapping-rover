#!/usr/bin/env python3
"""
Synthetic LiDAR publisher for indoor-mapping-rover.

Ray-casts a simulated 2D scan against the known geometry of room.sdf
(4 walls + 1 box obstacle + 1 cylinder obstacle) using the robot's real
odom-frame pose. Publishes real sensor_msgs/LaserScan messages on /scan
so slam_toolbox can be tested/validated without depending on Gazebo's
sensor rendering pipeline (which is currently hung under WSL2, see
docs/urdf_debugging_log.md).

Assumes the robot spawns at approximately the room's origin/center, so
the odom frame is treated as equivalent to the room's world frame. If
the robot doesn't spawn at (0,0), adjust ROOM_ORIGIN_OFFSET below.

Room geometry (from room.sdf):
  - 4m x 4m room, walls 0.1m thick, centered at origin
  - wall_north: y=2,  wall_south: y=-2, wall_east: x=2, wall_west: x=-2
  - obstacle_box_1: center (0.8, 0.5), 0.4 x 0.4 (axis-aligned square)
  - obstacle_cylinder_1: center (-0.9, -0.7), radius 0.2
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

ANGLE_MIN = -math.pi
ANGLE_MAX = math.pi
ANGLE_INCREMENT = math.radians(0.5)
RANGE_MIN = 0.12
RANGE_MAX = 12.0
SCAN_RATE_HZ = 10.0

LIDAR_FRAME = "lidar_link"
ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"

ROOM_ORIGIN_OFFSET = (0.0, 0.0)

WALLS = [
    (-2.0, 2.0, 2.0, 2.0),
    (-2.0, -2.0, 2.0, -2.0),
    (2.0, -2.0, 2.0, 2.0),
    (-2.0, -2.0, -2.0, 2.0),
]

_bx, _by, _bs = 0.8, 0.5, 0.2
BOX_EDGES = [
    (_bx - _bs, _by - _bs, _bx + _bs, _by - _bs),
    (_bx + _bs, _by - _bs, _bx + _bs, _by + _bs),
    (_bx + _bs, _by + _bs, _bx - _bs, _by + _bs),
    (_bx - _bs, _by + _bs, _bx - _bs, _by - _bs),
]

CYLINDER = (-0.9, -0.7, 0.2)


def ray_segment_intersect(ox, oy, dx, dy, x1, y1, x2, y2):
    sx, sy = x2 - x1, y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-9:
        return None
    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def ray_circle_intersect(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    a = dx * dx + dy * dy
    b = 2 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    disc_sqrt = math.sqrt(disc)
    t1 = (-b - disc_sqrt) / (2 * a)
    t2 = (-b + disc_sqrt) / (2 * a)
    candidates = [t for t in (t1, t2) if t >= 0.0]
    return min(candidates) if candidates else None


class FakeLidarPublisher(Node):
    def __init__(self):
        super().__init__('fake_lidar_publisher')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(LaserScan, '/scan', qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.n_samples = int(round((ANGLE_MAX - ANGLE_MIN) / ANGLE_INCREMENT)) + 1

        period = 1.0 / SCAN_RATE_HZ
        self.timer = self.create_timer(period, self.publish_scan)
        self.get_logger().info(
            f'fake_lidar_publisher started: {self.n_samples} samples/scan @ {SCAN_RATE_HZ} Hz'
        )

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, BASE_FRAME, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x + ROOM_ORIGIN_OFFSET[0], t.y + ROOM_ORIGIN_OFFSET[1], yaw)

    def cast_ray(self, ox, oy, angle):
        dx, dy = math.cos(angle), math.sin(angle)
        best = RANGE_MAX

        for (x1, y1, x2, y2) in WALLS:
            t = ray_segment_intersect(ox, oy, dx, dy, x1, y1, x2, y2)
            if t is not None and RANGE_MIN <= t < best:
                best = t

        for (x1, y1, x2, y2) in BOX_EDGES:
            t = ray_segment_intersect(ox, oy, dx, dy, x1, y1, x2, y2)
            if t is not None and RANGE_MIN <= t < best:
                best = t

        cx, cy, r = CYLINDER
        t = ray_circle_intersect(ox, oy, dx, dy, cx, cy, r)
        if t is not None and RANGE_MIN <= t < best:
            best = t

        return best

    def publish_scan(self):
        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        ranges = []
        for i in range(self.n_samples):
            local_angle = ANGLE_MIN + i * ANGLE_INCREMENT
            world_angle = ryaw + local_angle
            r = self.cast_ray(rx, ry, world_angle)
            ranges.append(r if r < RANGE_MAX else float('inf'))

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = LIDAR_FRAME
        msg.angle_min = ANGLE_MIN
        msg.angle_max = ANGLE_MAX
        msg.angle_increment = ANGLE_INCREMENT
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / SCAN_RATE_HZ
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = ranges
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeLidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

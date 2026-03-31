#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower_node")
        self.get_logger().info("Path Follower Initialized")

        # State
        self.path_points = []
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.latest_scan = None

        # Tunable parameters
        self.lookahead_distance = 0.8
        self.avoid_distance = 1.2
        self.max_speed = 1.0
        self.min_speed = 0.5
        self.prev_steering_angle = 0.0
        self.max_scan_distance = 12.0
        self.depth_threshold = 1.5
        self.wheelbase = 0.25
        self.max_steering_angle = 0.5

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(Path, "/global_centerline", self.path_callback, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)

        # Publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)

    def path_callback(self, msg):
        self.path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.get_logger().info(
            f"Received path with {len(self.path_points)} points in frame {msg.header.frame_id}",
            throttle_duration_sec=1.0
        )

    def odom_callback(self, msg):
        # Keep this subscriber alive in case needed later, but do not use raw odom pose directly
        pass

    def scan_callback(self, msg):
        self.latest_scan = msg

    def update_pose_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time()
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            self.current_x = t.x
            self.current_y = t.y

            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

            return True

        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    def control_loop(self):
        # Update pose in map frame using TF
        if not self.update_pose_from_tf():
            self.publish_drive(0.0, 0.0)
            return

        if len(self.path_points) < 2:
            self.publish_drive(0.0, 0.0)
            return

        # Find closest path point
        closest_idx = 0
        best_dist2 = float("inf")

        for i, (px, py) in enumerate(self.path_points):
            dx = px - self.current_x
            dy = py - self.current_y
            d2 = dx * dx + dy * dy

            if d2 < best_dist2:
                best_dist2 = d2
                closest_idx = i

        # Walk forward to find lookahead point
        total = 0.0
        target_x, target_y = self.path_points[closest_idx]

        for j in range(len(self.path_points)):
            i1 = (closest_idx + j) % len(self.path_points)
            i2 = (closest_idx + j + 1) % len(self.path_points)

            x1, y1 = self.path_points[i1]
            x2, y2 = self.path_points[i2]

            seg = math.hypot(x2 - x1, y2 - y1)
            total += seg

            if total >= self.lookahead_distance:
                target_x, target_y = x2, y2
                break

        # Pure pursuit steering
        dx = target_x - self.current_x
        dy = target_y - self.current_y

        local_x = math.cos(self.current_yaw) * dx + math.sin(self.current_yaw) * dy
        local_y = -math.sin(self.current_yaw) * dx + math.cos(self.current_yaw) * dy

        Ld = math.hypot(local_x, local_y)
        if Ld < 1e-6:
            self.publish_drive(0.0, 0.0)
            return

        alpha_pp = math.atan2(local_y, local_x)
        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha_pp), Ld)

        # LiDAR override disabled
        if False and self.latest_scan is not None:
            ranges = np.array(self.latest_scan.ranges)
            ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_scan_distance, neginf=0.0)
            ranges = np.clip(ranges, 0, self.max_scan_distance)

            num_beams = len(ranges)
            start_idx = int(num_beams * 0.40)
            end_idx = int(num_beams * 0.60)
            front_window = ranges[start_idx:end_idx]

            if len(front_window) > 0 and np.min(front_window) < self.avoid_distance:
                start_idx = int(num_beams * 0.20)
                end_idx = int(num_beams * 0.80)
                front_ranges = ranges[start_idx:end_idx]

                mask = front_ranges > self.depth_threshold
                slices = np.ma.clump_unmasked(np.ma.masked_where(~mask, front_ranges))

                if slices:
                    best_slice = max(slices, key=lambda s: s.stop - s.start)
                    best_idx = (best_slice.start + best_slice.stop - 1) // 2
                    angle_offset = (start_idx + best_idx) * self.latest_scan.angle_increment
                    steering = self.latest_scan.angle_min + angle_offset

        # Smoothing
        smoothing = 0.25

        if abs(steering) < 0.03:
            steering = 0.0

        steering = smoothing * steering + (1.0 - smoothing) * self.prev_steering_angle
        steering = max(-self.max_steering_angle, min(self.max_steering_angle, steering))
        self.prev_steering_angle = steering

        # Speed control
        turn_scale = max(0.0, 1.0 - abs(steering) / self.max_steering_angle)
        speed = self.min_speed + (self.max_speed - self.min_speed) * turn_scale

        self.get_logger().info(
            f"car=({self.current_x:.2f},{self.current_y:.2f}) "
            f"target=({target_x:.2f},{target_y:.2f}) "
            f"steering={steering:.3f} speed={speed:.3f}",
            throttle_duration_sec=0.5
        )

        self.publish_drive(steering, speed)

    def publish_drive(self, steering_angle, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
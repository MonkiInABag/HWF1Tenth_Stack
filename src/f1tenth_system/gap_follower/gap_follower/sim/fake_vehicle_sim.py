#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class FakeVehicleSim(Node):
    def __init__(self):
        super().__init__("fake_vehicle_sim")

        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("raw_odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("marker_topic", "/fake_vehicle_marker")

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_map_to_odom_tf", True)
        self.declare_parameter("publish_marker", True)

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("path_topic", "/global_centerline")
        self.declare_parameter("use_map_raycast", False)
        self.declare_parameter("auto_start_on_path", False)
        self.declare_parameter("start_path_fraction", 0.0)
        self.declare_parameter("start_search_mode", "fraction")
        self.declare_parameter("start_reverse_direction", False)
        self.declare_parameter("auto_start_min_clearance", 2.0)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("raycast_step", 0.05)

        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("wheelbase", 0.25)
        self.declare_parameter("max_steering_angle", 0.5)
        self.declare_parameter("drive_timeout_sec", 0.5)
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("scan_rate_hz", 20.0)
        self.declare_parameter("vehicle_length", 0.55)
        self.declare_parameter("vehicle_width", 0.32)
        self.declare_parameter("vehicle_height", 0.15)
        self.declare_parameter("vehicle_marker_scale", 2.0)
        self.declare_parameter("publish_heading_marker", True)

        self.declare_parameter("scan_beams", 721)
        self.declare_parameter("scan_angle_min", -2.35)
        self.declare_parameter("scan_angle_max", 2.35)
        self.declare_parameter("scan_range_max", 10.0)
        self.declare_parameter("scan_range_min", 0.05)

        self.declare_parameter("enable_front_blocker", False)
        self.declare_parameter("blocker_start_sec", 8.0)
        self.declare_parameter("blocker_duration_sec", 4.0)
        self.declare_parameter("blocker_distance", 0.45)
        self.declare_parameter("blocker_width_degrees", 25.0)

        self.drive_topic = self.get_parameter("drive_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.raw_odom_topic = self.get_parameter("raw_odom_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value

        self.odom_frame = self.get_parameter("odom_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_map_to_odom_tf = bool(
            self.get_parameter("publish_map_to_odom_tf").value
        )
        self.publish_marker = bool(self.get_parameter("publish_marker").value)

        self.map_topic = self.get_parameter("map_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.use_map_raycast = bool(self.get_parameter("use_map_raycast").value)
        self.auto_start_on_path = bool(self.get_parameter("auto_start_on_path").value)
        self.start_path_fraction = float(self.get_parameter("start_path_fraction").value)
        self.start_search_mode = self.get_parameter("start_search_mode").value
        self.start_reverse_direction = bool(
            self.get_parameter("start_reverse_direction").value
        )
        self.auto_start_min_clearance = float(
            self.get_parameter("auto_start_min_clearance").value
        )
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.raycast_step = float(self.get_parameter("raycast_step").value)

        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.max_steering_angle = float(self.get_parameter("max_steering_angle").value)
        self.drive_timeout_sec = float(self.get_parameter("drive_timeout_sec").value)
        self.vehicle_length = float(self.get_parameter("vehicle_length").value)
        self.vehicle_width = float(self.get_parameter("vehicle_width").value)
        self.vehicle_height = float(self.get_parameter("vehicle_height").value)
        self.vehicle_marker_scale = float(self.get_parameter("vehicle_marker_scale").value)
        self.publish_heading_marker_enabled = bool(
            self.get_parameter("publish_heading_marker").value
        )

        self.scan_beams = int(self.get_parameter("scan_beams").value)
        self.scan_angle_min = float(self.get_parameter("scan_angle_min").value)
        self.scan_angle_max = float(self.get_parameter("scan_angle_max").value)
        self.scan_range_max = float(self.get_parameter("scan_range_max").value)
        self.scan_range_min = float(self.get_parameter("scan_range_min").value)

        self.enable_front_blocker = bool(self.get_parameter("enable_front_blocker").value)
        self.blocker_start_sec = float(self.get_parameter("blocker_start_sec").value)
        self.blocker_duration_sec = float(self.get_parameter("blocker_duration_sec").value)
        self.blocker_distance = float(self.get_parameter("blocker_distance").value)
        self.blocker_width = math.radians(
            float(self.get_parameter("blocker_width_degrees").value)
        )

        self.speed_cmd = 0.0
        self.steer_cmd = 0.0
        self.last_drive_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.start_time = self.get_clock().now()
        self.map_msg = None
        self.map_data = None
        self.started_on_path = False
        self.pending_path = None

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.raw_odom_pub = self.create_publisher(Odometry, self.raw_odom_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(AckermannDriveStamped, self.drive_topic, self.drive_callback, 10)
        if self.use_map_raycast:
            map_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
            self.create_subscription(Path, self.path_topic, self.path_callback, map_qos)

        update_rate = float(self.get_parameter("update_rate_hz").value)
        scan_rate = float(self.get_parameter("scan_rate_hz").value)
        self.create_timer(1.0 / update_rate, self.update)
        self.create_timer(1.0 / scan_rate, self.publish_scan)

        self.get_logger().info(
            f"Fake vehicle sim listening on {self.drive_topic}, publishing "
            f"{self.odom_topic}, {self.raw_odom_topic}, and {self.scan_topic}"
        )

    def map_callback(self, msg):
        self.map_msg = msg
        self.map_data = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        self.get_logger().info(
            f"Loaded raycast map {msg.info.width}x{msg.info.height} "
            f"from {self.map_topic}",
            throttle_duration_sec=2.0
        )
        if self.pending_path is not None:
            self.start_on_path(self.pending_path)

    def path_callback(self, msg):
        if not self.auto_start_on_path or self.started_on_path or len(msg.poses) < 2:
            return

        if self.use_map_raycast and self.map_msg is None:
            self.pending_path = msg
            return

        self.start_on_path(msg)

    def start_on_path(self, msg):
        if self.started_on_path or len(msg.poses) < 2:
            return

        idx, clearance = self.choose_start_index(msg)
        direction = -1 if self.start_reverse_direction else 1
        next_idx = (idx + direction) % len(msg.poses)

        pose = msg.poses[idx].pose.position
        next_pose = msg.poses[next_idx].pose.position

        self.x = float(pose.x)
        self.y = float(pose.y)
        self.yaw = math.atan2(next_pose.y - pose.y, next_pose.x - pose.x)
        self.speed_cmd = 0.0
        self.steer_cmd = 0.0
        self.started_on_path = True
        self.pending_path = None

        self.get_logger().info(
            f"Auto-started fake vehicle on {self.path_topic} at index {idx}: "
            f"x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}, "
            f"front_clearance={clearance:.2f}m, "
            f"direction={'reverse' if self.start_reverse_direction else 'forward'}"
        )

        if clearance < self.auto_start_min_clearance:
            self.get_logger().warn(
                "Best auto-start point is still close to an obstacle; "
                "try a different saved map or lower auto_start_min_clearance.",
                throttle_duration_sec=2.0
            )

    def choose_start_index(self, msg):
        start_fraction = max(0.0, min(1.0, self.start_path_fraction))
        start_idx = min(int(start_fraction * len(msg.poses)), len(msg.poses) - 2)

        if (
            self.start_search_mode == "fraction"
            or not self.use_map_raycast
            or self.map_msg is None
        ):
            clearance = (
                self.path_pose_front_clearance(msg, start_idx)
                if self.use_map_raycast and self.map_msg is not None
                else self.scan_range_max
            )
            return start_idx, clearance

        best_idx = start_idx
        best_clearance = -1.0

        for offset in range(len(msg.poses) - 1):
            idx = (start_idx + offset) % (len(msg.poses) - 1)
            clearance = self.path_pose_front_clearance(msg, idx)

            if clearance > best_clearance:
                best_clearance = clearance
                best_idx = idx

            if clearance >= self.auto_start_min_clearance:
                return idx, clearance

        return best_idx, best_clearance

    def path_pose_front_clearance(self, msg, idx):
        next_idx = (idx + 1) % len(msg.poses)
        pose = msg.poses[idx].pose.position
        next_pose = msg.poses[next_idx].pose.position
        yaw = math.atan2(next_pose.y - pose.y, next_pose.x - pose.x)
        laser_x = float(pose.x) + 0.27 * math.cos(yaw)
        laser_y = float(pose.y) + 0.27 * math.sin(yaw)

        clearances = [
            self.raycast_one(laser_x, laser_y, yaw + math.radians(angle_deg))
            for angle_deg in (-12.0, 0.0, 12.0)
        ]
        return min(clearances)

    def legacy_start_on_path_fraction(self, msg):
        fraction = max(0.0, min(1.0, self.start_path_fraction))
        idx = min(int(fraction * len(msg.poses)), len(msg.poses) - 2)
        next_idx = (idx + 1) % len(msg.poses)

        pose = msg.poses[idx].pose.position
        next_pose = msg.poses[next_idx].pose.position

        self.x = float(pose.x)
        self.y = float(pose.y)
        self.yaw = math.atan2(next_pose.y - pose.y, next_pose.x - pose.x)
        self.speed_cmd = 0.0
        self.steer_cmd = 0.0
        self.started_on_path = True

        self.get_logger().info(
            f"Auto-started fake vehicle on {self.path_topic} at index {idx}: "
            f"x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}"
        )

    def drive_callback(self, msg):
        self.speed_cmd = float(msg.drive.speed)
        self.steer_cmd = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, float(msg.drive.steering_angle)),
        )
        self.last_drive_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if dt <= 0.0:
            return

        drive_age = (now - self.last_drive_time).nanoseconds * 1e-9
        speed = self.speed_cmd if drive_age <= self.drive_timeout_sec else 0.0

        self.x += speed * math.cos(self.yaw) * dt
        self.y += speed * math.sin(self.yaw) * dt
        self.yaw += speed * math.tan(self.steer_cmd) / self.wheelbase * dt
        self.yaw = (self.yaw + math.pi) % (2.0 * math.pi) - math.pi

        self.publish_odom(now, speed)
        if self.publish_marker:
            self.publish_vehicle_marker(now)
            if self.publish_heading_marker_enabled:
                self.publish_heading_marker(now)

        if self.publish_tf:
            self.publish_transforms(now)

    def publish_odom(self, stamp, speed):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = yaw_to_quat(self.yaw)
        msg.twist.twist.linear.x = speed
        msg.twist.twist.angular.z = speed * math.tan(self.steer_cmd) / self.wheelbase

        self.odom_pub.publish(msg)
        self.raw_odom_pub.publish(msg)

    def publish_vehicle_marker(self, stamp):
        msg = Marker()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.ns = "fake_vehicle"
        msg.id = 0
        msg.type = Marker.CYLINDER
        msg.action = Marker.ADD
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.vehicle_height * 0.5
        msg.pose.orientation = yaw_to_quat(self.yaw)
        msg.scale.x = self.vehicle_length * self.vehicle_marker_scale
        msg.scale.y = self.vehicle_width * self.vehicle_marker_scale
        msg.scale.z = self.vehicle_height * self.vehicle_marker_scale
        msg.color.r = 1.0
        msg.color.g = 0.55
        msg.color.b = 0.05
        msg.color.a = 0.9
        self.marker_pub.publish(msg)

    def publish_heading_marker(self, stamp):
        msg = Marker()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.ns = "fake_vehicle"
        msg.id = 1
        msg.type = Marker.ARROW
        msg.action = Marker.ADD
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.vehicle_height + 0.15
        msg.pose.orientation = yaw_to_quat(self.yaw)
        msg.scale.x = self.vehicle_length * self.vehicle_marker_scale
        msg.scale.y = 0.12 * self.vehicle_marker_scale
        msg.scale.z = 0.12 * self.vehicle_marker_scale
        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 1.0
        msg.color.a = 1.0
        self.marker_pub.publish(msg)

    def publish_transforms(self, stamp):
        odom_tf = TransformStamped()
        odom_tf.header.stamp = stamp.to_msg()
        odom_tf.header.frame_id = self.odom_frame
        odom_tf.child_frame_id = self.base_frame
        odom_tf.transform.translation.x = self.x
        odom_tf.transform.translation.y = self.y
        odom_tf.transform.rotation = yaw_to_quat(self.yaw)

        laser_tf = TransformStamped()
        laser_tf.header.stamp = stamp.to_msg()
        laser_tf.header.frame_id = self.base_frame
        laser_tf.child_frame_id = self.laser_frame
        laser_tf.transform.translation.x = 0.27
        laser_tf.transform.translation.z = 0.11
        laser_tf.transform.rotation.w = 1.0

        transforms = []

        if self.publish_map_to_odom_tf:
            map_tf = TransformStamped()
            map_tf.header.stamp = stamp.to_msg()
            map_tf.header.frame_id = self.map_frame
            map_tf.child_frame_id = self.odom_frame
            map_tf.transform.rotation.w = 1.0
            transforms.append(map_tf)

        transforms.extend([odom_tf, laser_tf])
        self.tf_broadcaster.sendTransform(transforms)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.laser_frame
        msg.angle_min = self.scan_angle_min
        msg.angle_max = self.scan_angle_max
        msg.angle_increment = (self.scan_angle_max - self.scan_angle_min) / (self.scan_beams - 1)
        msg.time_increment = 0.0
        msg.scan_time = 0.05
        msg.range_min = self.scan_range_min
        msg.range_max = self.scan_range_max
        if self.use_map_raycast and self.map_msg is not None:
            msg.ranges = self.raycast_scan(msg)
        else:
            msg.ranges = [self.scan_range_max] * self.scan_beams

        if self.blocker_is_active():
            for i in range(self.scan_beams):
                angle = msg.angle_min + i * msg.angle_increment
                if abs(angle) <= self.blocker_width * 0.5:
                    msg.ranges[i] = self.blocker_distance

        self.scan_pub.publish(msg)

    def raycast_scan(self, scan_msg):
        ranges = []
        laser_x = self.x + 0.27 * math.cos(self.yaw)
        laser_y = self.y + 0.27 * math.sin(self.yaw)

        for i in range(self.scan_beams):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            ranges.append(self.raycast_one(laser_x, laser_y, self.yaw + angle))

        return ranges

    def raycast_one(self, start_x, start_y, angle):
        distance = self.scan_range_min

        while distance <= self.scan_range_max:
            x = start_x + distance * math.cos(angle)
            y = start_y + distance * math.sin(angle)

            cell = self.world_to_map(x, y)
            if cell is None:
                return distance

            mx, my = cell
            value = self.map_data[my, mx]
            if value >= self.occupied_threshold:
                return distance

            distance += self.raycast_step

        return self.scan_range_max

    def world_to_map(self, x, y):
        info = self.map_msg.info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        mx = int((x - origin_x) / info.resolution)
        my = int((y - origin_y) / info.resolution)

        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None

        return mx, my

    def blocker_is_active(self):
        if not self.enable_front_blocker:
            return False

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        return self.blocker_start_sec <= elapsed <= (
            self.blocker_start_sec + self.blocker_duration_sec
        )


def main(args=None):
    rclpy.init(args=args)
    node = FakeVehicleSim()

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

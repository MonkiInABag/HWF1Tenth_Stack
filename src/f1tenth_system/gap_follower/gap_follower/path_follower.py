#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower_node")
        self.get_logger().info("Path Follower Initialized")

        #State of car
        self.path_points = []        # global path [(x, y), ...]
        self.current_x = None        # car position
        self.current_y = None
        self.current_yaw = None      # car heading
        self.latest_scan = None      # latest LiDAR data

        #Turnable parameters       
        self.lookahead_distance = 0.7   # how far ahead on path to aim
        self.avoid_distance = 1.2       # distance to trigger avoidance
        self.max_speed = 2.5
        self.min_speed = 1.5
        self.prev_steering_angle = 0.0
        self.max_scan_distance = 12.0
        self.depth_threshold = 1.5      # used for gap detection

        #Subscribers
        self.create_subscription(Path, "/global_centerline", self.path_callback, 10)
        self.create_subscription(Odometry, "/ekf/odometry", self.odom_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)

        # Publisher for driving commands
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        # Run control loop at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    #Recieve data 
    def path_callback(self, msg):
        # Convert ROS Path → simple list of (x, y)
        self.path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_callback(self, msg):
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion → yaw (heading)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        # Store latest LiDAR scan
        self.latest_scan = msg

    #MAin
    def control_loop(self):

        # --- Safety check: do we have enough data? ---
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            self.publish_drive(0.0, 0.0)
            return

        if len(self.path_points) < 2:
            self.publish_drive(0.0, 0.0)
            return

        #Global path planner
        #Find closest point on path
        closest_idx = 0
        best_dist2 = float("inf")

        for i, (px, py) in enumerate(self.path_points):
            dx = px - self.current_x
            dy = py - self.current_y
            d2 = dx * dx + dy * dy

            if d2 < best_dist2:
                best_dist2 = d2
                closest_idx = i

        #Move forward along path to find lookahead point
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

        #Compute steering toward lookahead point
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        target_heading = math.atan2(dy, dx)

        steering = target_heading - self.current_yaw

        # Normalize angle to [-pi, pi]
        while steering > math.pi:
            steering -= 2.0 * math.pi
        while steering < -math.pi:
            steering += 2.0 * math.pi

        # 2. LIDAR SAFETY OVERRIDE (GAP FOLLOWER)

        #if self.latest_scan is not None:
        if  False:

            # Clean up LiDAR data
            ranges = np.array(self.latest_scan.ranges)
            ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_scan_distance, neginf=0.0)
            ranges = np.clip(ranges, 0, self.max_scan_distance)

            #Check front distance 
            num_beams = len(ranges)
            start_idx = int(num_beams * 0.40)
            end_idx = int(num_beams * 0.60)

            front_window = ranges[start_idx:end_idx]

            # If obstacle too close → override with gap followe
            if len(front_window) > 0 and np.min(front_window) < self.avoid_distance:

                # Focus on forward-facing region
                start_idx = int(num_beams * 0.20)
                end_idx = int(num_beams * 0.80)
                front_ranges = ranges[start_idx:end_idx]

                # Find safe regions (gaps)
                mask = front_ranges > self.depth_threshold
                slices = np.ma.clump_unmasked(np.ma.masked_where(~mask, front_ranges))

                if slices:
                    # Choose largest gap
                    best_slice = max(slices, key=lambda s: s.stop - s.start)

                    # Aim for center of gap
                    best_idx = (best_slice.start + best_slice.stop - 1) // 2
                    angle_offset = (start_idx + best_idx) * self.latest_scan.angle_increment
                    steering = self.latest_scan.angle_min + angle_offset

        #jitter reduce
        alpha = 0.18

        if abs(steering) < 0.10:
            steering = 0.0

        steering = alpha * steering + (1.0 - alpha) * self.prev_steering_angle
        self.prev_steering_angle = steering

        #Speed control
        # Slow down when turning more
        speed = max(self.min_speed, self.max_speed * (1.0 - abs(steering)))

        self.publish_drive(steering, speed)

    #car commands
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
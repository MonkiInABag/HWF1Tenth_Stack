#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class F1TenthEkfNode(Node):

    def __init__(self):
        super().__init__('f1tenth_ekf_node')
        self.get_logger().info("REAL EKF VERSION IS RUNNING")

        # Parameters
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('fused_odom_topic', '/odometry/filtered')
        self.declare_parameter('gyro_bias_z', 0.0)

        imu_topic = self.get_parameter('imu_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        fused_topic = self.get_parameter('fused_odom_topic').value
        self.gyro_bias_z = float(self.get_parameter('gyro_bias_z').value)

        self.get_logger().info(f"IMU topic: {imu_topic}")
        self.get_logger().info(f"Odom topic: {odom_topic}")
        self.get_logger().info(f"Fused topic: {fused_topic}")

        # Subscribers / Publishers
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.on_imu, 50)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.on_odom, 50)
        self.fused_pub = self.create_publisher(Odometry, fused_topic, 10)

        # TF broadcaster (IMPORTANT)
        self.tf_broadcaster = TransformBroadcaster(self)

        # EKF State: [x, y, yaw, v]
        self.x = np.zeros((4, 1), dtype=float)

        # Covariance
        self.P = np.diag([
            0.5**2,
            0.5**2,
            (10.0 * math.pi/180.0)**2,
            0.5**2
        ])

        # Process noise
        self.Q = np.diag([
            0.05**2,
            0.05**2,
            (1.5 * math.pi/180.0)**2,
            0.2**2
        ])

        # Speed measurement noise
        self.R_v = np.array([[0.3**2]])

        # Timing
        self.last_imu_stamp = None
        self.max_dt = 0.1
        self.omega_z = 0.0
        self.last_odom_header = None

        # Rate reporting
        self.imu_count = 0
        self.odom_count = 0
        self.create_timer(1.0, self.report_rates)


    # IMU → Predict
    def on_imu(self, msg: Imu):
        self.imu_count += 1

        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            t = self.get_clock().now().nanoseconds * 1e-9
        else:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_imu_stamp is None:
            self.last_imu_stamp = t
            return

        dt = t - self.last_imu_stamp
        self.last_imu_stamp = t

        if dt <= 0.0:
            return
        if dt > self.max_dt:
            dt = self.max_dt

        self.omega_z = float(msg.angular_velocity.z) - self.gyro_bias_z

        self.predict(dt, self.omega_z)

    # Odom → Update
    def on_odom(self, msg: Odometry):
        self.odom_count += 1
        self.last_odom_header = msg.header

        # Directly use odom pose
        self.x[0, 0] = float(msg.pose.pose.position.x)
        self.x[1, 0] = float(msg.pose.pose.position.y)

        # Extract yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_meas = math.atan2(siny_cosp, cosy_cosp)
        self.x[2, 0] = yaw_meas

        v_meas = float(msg.twist.twist.linear.x)
        self.update_speed(v_meas)

        self.publish_fused()


    # Predict Step
    def predict(self, dt, omega_z):
        px, py, yaw, v = self.x.flatten()

        yaw_new = yaw + omega_z * dt
        yaw_new = (yaw_new + math.pi) % (2 * math.pi) - math.pi

        px_new = px + v * math.cos(yaw) * dt
        py_new = py + v * math.sin(yaw) * dt
        v_new = v

        self.x = np.array([[px_new], [py_new], [yaw_new], [v_new]])

        F = np.eye(4)
        F[0, 2] = -v * math.sin(yaw) * dt
        F[0, 3] =  math.cos(yaw) * dt
        F[1, 2] =  v * math.cos(yaw) * dt
        F[1, 3] =  math.sin(yaw) * dt

        self.P = F @ self.P @ F.T + self.Q

    # Update Step
    def update_speed(self, v_meas):
        H = np.array([[0.0, 0.0, 0.0, 1.0]])
        z = np.array([[v_meas]])

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_v
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

    # Publish
    def publish_fused(self):
        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        px, py, yaw, v = self.x.flatten()

        msg.pose.pose.position.x = float(px)
        msg.pose.pose.position.y = float(py)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quat(float(yaw))

        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.angular.z = float(self.omega_z)

        # Covariances
        pose_cov = [0.0] * 36
        pose_cov[0] = float(self.P[0, 0])
        pose_cov[7] = float(self.P[1, 1])
        pose_cov[35] = float(self.P[2, 2])
        pose_cov[14] = 999.0
        pose_cov[21] = 999.0
        pose_cov[28] = 999.0
        msg.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = float(self.P[3, 3])
        twist_cov[35] = 0.25
        msg.twist.covariance = twist_cov

        self.fused_pub.publish(msg)

        # TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(px)
        t.transform.translation.y = float(py)
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quat(float(yaw))

        self.tf_broadcaster.sendTransform(t)
        
    # Debug
    def report_rates(self):
        self.get_logger().info(
            f"IMU: {self.imu_count}/s | "
            f"Odom: {self.odom_count}/s | "
            f"x={self.x[0,0]:.2f}, "
            f"y={self.x[1,0]:.2f}, "
            f"yaw={self.x[2,0]:.2f}, "
            f"v={self.x[3,0]:.2f}"
        )
        self.imu_count = 0
        self.odom_count = 0


def main():
    rclpy.init()
    node = F1TenthEkfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quat_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class F1TenthEkfNode(Node):

    def __init__(self):
        super().__init__('f1tenth_ekf_node')
        self.get_logger().info("SOFT-STOP EKF RUNNING")

        # Topics
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.fused_pub = self.create_publisher(Odometry, '/ekf/odometry', 10)

        # State: [x, y, yaw, v]
        self.x = np.zeros((4, 1), dtype=float)

        # Covariance
        self.P = np.diag([
            0.5**2,
            0.5**2,
            (10.0 * math.pi / 180.0)**2,
            0.5**2
        ])

        # Process noise
        self.Q = np.diag([
            0.05**2,
            0.05**2,
            (1.5 * math.pi / 180.0)**2,
            1.0**2
        ])

        # Measurement noise
        self.R_v = np.array([[0.03**2]])
        self.R_yaw = np.array([[(5.0 * math.pi / 180.0)**2]])

        # Timing
        self.last_imu_stamp = None
        self.omega_z = 0.0

        # Initialisation flags
        self.initialized_speed = False
        self.initialized_yaw = False

        # Soft stop logic
        self.stop_counter = 0
        self.stop_threshold = 0.03
        self.stop_required_count = 3

        # Debug
        self.imu_count = 0
        self.odom_count = 0
        self.create_timer(1.0, self.report_rates)

    # ---------------- IMU ----------------
    def on_imu(self, msg: Imu):
        self.imu_count += 1

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_imu_stamp is None:
            self.last_imu_stamp = t

            if not self.initialized_yaw:
                yaw = quat_to_yaw(msg.orientation)
                self.x[2, 0] = yaw
                self.initialized_yaw = True
                self.get_logger().info(f"Initial yaw set from IMU: {yaw:.3f} rad")

            return

        dt = t - self.last_imu_stamp
        self.last_imu_stamp = t

        if dt <= 0.0 or dt > 0.2:
            return

        self.omega_z = float(msg.angular_velocity.z)

        self.predict(dt)

        yaw_meas = quat_to_yaw(msg.orientation)
        self.update_yaw(yaw_meas)

    # ---------------- ODOM ----------------
    def on_odom(self, msg: Odometry):
        self.odom_count += 1

        v_meas = float(msg.twist.twist.linear.x)

        if not self.initialized_speed:
            self.x[3, 0] = v_meas
            self.initialized_speed = True
            self.get_logger().info(f"Initial speed set from odom: {v_meas:.3f} m/s")

        # Soft stop logic
        if abs(v_meas) < self.stop_threshold:
            self.stop_counter += 1
        else:
            self.stop_counter = 0

        if self.stop_counter >= self.stop_required_count:
            self.x[3, 0] = 0.0
            self.P[3, 3] = 0.02
        else:
            self.update_speed(v_meas)

        self.publish()

    # ---------------- PREDICT ----------------
    def predict(self, dt: float):
        px, py, yaw, v = self.x.flatten()

        yaw_pred = wrap_angle(yaw + self.omega_z * dt)
        px_pred = px + v * math.cos(yaw_pred) * dt
        py_pred = py + v * math.sin(yaw_pred) * dt
        v_pred = v

        self.x = np.array([[px_pred], [py_pred], [yaw_pred], [v_pred]], dtype=float)

        F = np.eye(4)
        F[0, 2] = -v * math.sin(yaw_pred) * dt
        F[0, 3] = math.cos(yaw_pred) * dt
        F[1, 2] = v * math.cos(yaw_pred) * dt
        F[1, 3] = math.sin(yaw_pred) * dt

        self.P = F @ self.P @ F.T + self.Q * dt

    # ---------------- UPDATE SPEED ----------------
    def update_speed(self, v_meas: float):
        H = np.array([[0.0, 0.0, 0.0, 1.0]])
        z = np.array([[v_meas]])

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_v
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y

        I = np.eye(4)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ self.R_v @ K.T

        # Keep velocity uncertainty from collapsing too much
        self.P[3, 3] = max(self.P[3, 3], 0.02)

    # ---------------- UPDATE YAW ----------------
    def update_yaw(self, yaw_meas: float):
        H = np.array([[0.0, 0.0, 1.0, 0.0]])

        innovation = wrap_angle(yaw_meas - self.x[2, 0])
        y = np.array([[innovation]])

        S = H @ self.P @ H.T + self.R_yaw
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2, 0] = wrap_angle(self.x[2, 0])

        I = np.eye(4)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ self.R_yaw @ K.T

    # ---------------- PUBLISH ----------------
    def publish(self):
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
        twist_cov[35] = float(self.R_yaw[0, 0])
        msg.twist.covariance = twist_cov

        self.fused_pub.publish(msg)

    # ---------------- DEBUG ----------------
    def report_rates(self):
        self.get_logger().info(
            f"IMU: {self.imu_count}/s | "
            f"Odom: {self.odom_count}/s | "
            f"x={self.x[0, 0]:.2f}, "
            f"y={self.x[1, 0]:.2f}, "
            f"yaw={self.x[2, 0]:.2f}, "
            f"v={self.x[3, 0]:.2f}, "
            f"stop_counter={self.stop_counter}"
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
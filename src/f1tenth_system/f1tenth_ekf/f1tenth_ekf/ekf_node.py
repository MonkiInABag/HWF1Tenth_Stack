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
        self.get_logger().info("FAST RESPONSE EKF RUNNING")

        # Topics
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.fused_pub = self.create_publisher(Odometry, '/ekf/odometry', 10)

        # State: [x, y, yaw, v]
        self.x = np.zeros((4, 1))

        # Covariance
        self.P = np.diag([
            0.5**2,
            0.5**2,
            (10.0 * math.pi/180.0)**2,
            0.5**2
        ])

        # 🔥 UPDATED: higher velocity uncertainty
        self.Q = np.diag([
            0.05**2,
            0.05**2,
            (1.5 * math.pi/180.0)**2,
            1.0**2   # BIG increase
        ])

        # 🔥 UPDATED: trust odom more
        self.R_v = np.array([[0.05**2]])

        # Yaw correction
        self.R_yaw = np.array([[(5.0 * math.pi / 180.0)**2]])

        # Timing
        self.last_imu_stamp = None
        self.omega_z = 0.0

        # Init flags
        self.initialized_speed = False
        self.initialized_yaw = False

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

            return

        dt = t - self.last_imu_stamp
        self.last_imu_stamp = t

        # 🔥 UPDATED: NO fake clamping
        if dt <= 0.0 or dt > 0.2:
            return

        self.omega_z = msg.angular_velocity.z

        self.predict(dt)

        # yaw correction
        yaw_meas = quat_to_yaw(msg.orientation)
        self.update_yaw(yaw_meas)

    # ---------------- ODOM ----------------
    def on_odom(self, msg: Odometry):
        self.odom_count += 1

        v_meas = msg.twist.twist.linear.x

        # initialise
        if not self.initialized_speed:
            self.x[3, 0] = v_meas
            self.initialized_speed = True

        # 🔥 UPDATED: HARD STOP FIX
        if abs(v_meas) < 0.03:
            self.x[3, 0] = 0.0
            self.P[3, 3] = min(self.P[3, 3], 0.01)
        else:
            self.update_speed(v_meas)

        self.publish()

    # ---------------- PREDICT ----------------
    def predict(self, dt):
        px, py, yaw, v = self.x.flatten()

        yaw = wrap_angle(yaw + self.omega_z * dt)

        px += v * math.cos(yaw) * dt
        py += v * math.sin(yaw) * dt

        self.x = np.array([[px], [py], [yaw], [v]])

        F = np.eye(4)
        F[0, 2] = -v * math.sin(yaw) * dt
        F[0, 3] = math.cos(yaw) * dt
        F[1, 2] = v * math.cos(yaw) * dt
        F[1, 3] = math.sin(yaw) * dt

        # 🔥 UPDATED: Q scaled by dt
        self.P = F @ self.P @ F.T + self.Q * dt

    # ---------------- UPDATE SPEED ----------------
    def update_speed(self, v_meas):
        H = np.array([[0, 0, 0, 1]])
        z = np.array([[v_meas]])

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_v
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y

        I = np.eye(4)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ self.R_v @ K.T

    # ---------------- UPDATE YAW ----------------
    def update_yaw(self, yaw_meas):
        H = np.array([[0, 0, 1, 0]])

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
        msg.pose.pose.orientation = yaw_to_quat(yaw)

        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.angular.z = float(self.omega_z)

        self.fused_pub.publish(msg)

    # ---------------- DEBUG ----------------
    def report_rates(self):
        self.get_logger().info(
            f"IMU: {self.imu_count}/s | "
            f"Odom: {self.odom_count}/s | "
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
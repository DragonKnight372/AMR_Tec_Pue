#!/usr/bin/env python3
"""
=========================================================
AMH19 - ROS2 LiDAR Listener and Polar Visualization Tool
---------------------------------------------------------
Author: Abraham Moro-Hernandez (AMH19)
=========================================================
Description:
    This node subscribes to a LaserScan topic from the QCar platform
    and visualizes the scan in real time using a polar plot. It runs 
    at 10 Hz using a timer-based update, while the latest received 
    scan is stored asynchronously via a ROS2 subscription callback.

Features:
    - QoS configured for non-critical LiDAR data (Best Effort)
    - Consumes /qcar/scan messages
    - Real-time 10 Hz polar visualization using matplotlib
    - Reports basic metrics (minimum and mean range)
    - Fully consistent with ROS2 sensor pipeline
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt


class LidarListener(Node):
    """ROS2 node that listens to a LaserScan topic and visualizes LiDAR data."""

    def __init__(self):
        super().__init__('lidar_listener_node')

        # ===========================================
        # QoS Configuration for LiDAR Sensor Data
        # ===========================================
        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # ===========================================
        # LiDAR Topic Subscription
        # ===========================================
        self.create_subscription(
            LaserScan,
            '/qcar/scan',
            self.lidar_callback,
            qos_profile=qos_lidar
        )

        # ===========================================
        # Timer at 10 Hz for Visualization Updates
        # ===========================================
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.latest_scan = None

        # ===========================================
        # Matplotlib Real-Time Plot Configuration
        # ===========================================
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_title("QCar LiDAR View (10 Hz)", va='bottom')
        self.ax.set_rmax(6.0)
        self.scatter = None

        self.get_logger().info("LiDAR Listener node active and listening to /qcar/scan")

    # --------------------------------------------------
    # Subscription Callback: store latest LaserScan msg
    # --------------------------------------------------
    def lidar_callback(self, msg: LaserScan):
        self.latest_scan = msg

    # --------------------------------------------------
    # Timer Callback: triggered at 10 Hz
    # --------------------------------------------------
    def timer_callback(self):
        if self.latest_scan is not None:
            self.plot_scan(self.latest_scan)

    # --------------------------------------------------
    # Polar Plot Rendering of LiDAR Data
    # --------------------------------------------------
    def plot_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=float)
        valid = np.isfinite(ranges)

        if not np.any(valid):
            self.get_logger().warn("LiDAR produced no valid range data")
            return

        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        ranges = np.clip(ranges, msg.range_min, msg.range_max)

        self.ax.clear()
        self.ax.scatter(angles[valid], ranges[valid], s=5, c='orange')
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.set_rmax(6.0)
        self.ax.set_title("QCar LiDAR View (10 Hz)", va='bottom')
        plt.pause(0.001)

        d_min = np.min(ranges[valid])
        d_mean = np.mean(ranges[valid])

        self.get_logger().info(
            f"Minimum distance = {d_min:.2f} m | Mean distance = {d_mean:.2f} m"
        )


# --------------------------------------------------
# Main Entry Point
# --------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LidarListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
=======================================================================
 QCar Obstacle Detector Node - Smart Mobility 
 Author: Abraham Moro-Hernandez (AMH19)

-----------------------------------------------------------------------
 Determines whether an obstacle is present in front of the QCar using
 LiDAR data and a dynamic field of view that adjusts based on vehicle
 speed. Publishes a boolean alert signal when an obstacle is detected.
=======================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped
import numpy as np


class ObstacleDetector(Node):
    """Obstacle detection module for the QCar platform using LiDAR."""

    def __init__(self):
        super().__init__('obstacle_detector')

        # ----------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------
        self.distance_threshold = 0.35  # meters

        # Dynamic angular window based on vehicle velocity
        self.angle_range_low = 22.5     # deg  (velocity <= 1 m/s)
        self.angle_range_high = 30.0    # deg  (velocity > 1 m/s)
        self.velocity_threshold = 1.0   # m/s

        self.angle_range = self.angle_range_low  # default window
        self.current_velocity_x = 0.0            # longitudinal velocity

        # LiDAR mounting offset (QCar LiDAR faces 270° / 4.71 rad)
        self.front_angle_offset = 4.71

        # Debug mode
        self.debug_mode = True

        # ----------------------------------------------------------
        # Publishers
        # ----------------------------------------------------------
        self.alert_pub = self.create_publisher(Bool, '/qcar/obstacle_alert', 10)

        # ----------------------------------------------------------
        # Subscribers
        # ----------------------------------------------------------
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/qcar/scan',
            self.lidar_callback,
            10
        )

        self.velocity_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/velocity',
            self.velocity_callback,
            10
        )

        # ----------------------------------------------------------
        # Startup Info
        # ----------------------------------------------------------
        self.get_logger().info("QCar Obstacle Detector Node initialized")
        self.get_logger().info(f"  Distance threshold: {self.distance_threshold} m")
        self.get_logger().info(f"  Angle range (<= {self.velocity_threshold} m/s): ±{self.angle_range_low}°")
        self.get_logger().info(f"  Angle range (>  {self.velocity_threshold} m/s): ±{self.angle_range_high}°")
        self.get_logger().info(f"  LiDAR front offset: {np.degrees(self.front_angle_offset):.1f}°")

        if self.debug_mode:
            self.get_logger().info("  Debug Mode: ON")

    # --------------------------------------------------------------
    # Velocity Callback
    # --------------------------------------------------------------
    def velocity_callback(self, msg):
        """Updates velocity and adjusts the angular detection window."""
        self.current_velocity_x = abs(msg.vector.x)

        # Switch window depending on velocity
        if self.current_velocity_x > self.velocity_threshold:
            self.angle_range = self.angle_range_high
        else:
            self.angle_range = self.angle_range_low

        if self.debug_mode:
            self.get_logger().info(
                f"Velocity X: {self.current_velocity_x:.2f} m/s -> Cone: ±{self.angle_range}°"
            )

    # --------------------------------------------------------------
    # LiDAR Callback
    # --------------------------------------------------------------
    def lidar_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = np.array(msg.ranges)

        # Debug: Show nearest obstacle
        if self.debug_mode:
            min_dist = float('inf')
            min_angle = 0
            for i, d in enumerate(ranges):
                if msg.range_min < d < min_dist:
                    min_dist = d
                    min_angle = angle_min + i * angle_increment
            self.get_logger().info(
                f"Closest obstacle: {min_dist:.2f} m at {min_angle:.3f} rad ({np.degrees(min_angle):.1f}°)"
            )

        # Compute indices for forward detection window
        center_index = int((self.front_angle_offset - angle_min) / angle_increment)
        range_indices = int((self.angle_range * np.pi / 180) / angle_increment)

        start_idx = max(0, center_index - range_indices)
        end_idx = min(len(ranges) - 1, center_index + range_indices)

        # Detect obstacles in window
        obstacle_detected = False
        min_distance = float('inf')

        for i in range(start_idx, end_idx + 1):
            distance = ranges[i]
            if msg.range_min < distance < self.distance_threshold:
                obstacle_detected = True
                min_distance = min(min_distance, distance)

        # Publish alert
        alert_msg = Bool()
        alert_msg.data = obstacle_detected
        self.alert_pub.publish(alert_msg)

        if obstacle_detected:
            self.get_logger().warn(
                f"Obstacle detected at {min_distance:.2f} m "
                f"(velocity: {self.current_velocity_x:.2f} m/s, cone: ±{self.angle_range}°)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

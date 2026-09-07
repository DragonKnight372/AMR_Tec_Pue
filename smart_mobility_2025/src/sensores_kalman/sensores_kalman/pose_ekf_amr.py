#!/usr/bin/env python3
"""
=======================================================================
 AMR Pose Node - Smart Mobility 
 Author: Abraham Moro-Hernandez (AMH19)
-----------------------------------------------------------------------
 Pose Estimator for AMR Platform
 Integrates wheel encoder linear velocity with IMU yaw measurements
 to produce a dead-reckoning 2D pose estimate.
=======================================================================

Subscribed Topics:
    /amr/odom   (sm_interfaces/msg/Amr)
        Unified encoder + IMU sensor data.

Published Topics:
    /amr/pose   (geometry_msgs/msg/Vector3Stamped)
        Estimated pose [x, y, theta(radians)].
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sm_interfaces.msg import Amr
import math


class PoseEstimator(Node):
    """Computes robot 2D pose using velocity integration and IMU yaw."""

    def __init__(self):
        super().__init__('pose_estimator')

        # Initial state [x, y, theta]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Variables for integration
        self.last_time = None
        self.velocity = 0.0
        self.yaw_deg = 0.0
        self.theta_imu = 0.0

        # Yaw calibration offset
        self.theta_offset = None
        self.calibrated = False

        # Subscription to encoder + IMU unified data
        self.amr_sub = self.create_subscription(
            Amr,
            '/amr/odom',
            self.amr_callback,
            10
        )

        # Publisher for estimated pose
        self.pose_pub = self.create_publisher(
            Vector3Stamped,
            '/amr/pose',
            10
        )

        # Timer running at 20 Hz
        self.timer = self.create_timer(0.05, self.update_pose)

        self.get_logger().info("==================================================")
        self.get_logger().info(" AMR POSE ESTIMATOR STARTED")
        self.get_logger().info(" Subscribed: /amr/odom")
        self.get_logger().info(" Publishing: /amr/pose")
        self.get_logger().info("==================================================")

    def normalize_angle(self, angle):
        """Normalize angle to [0, 2*pi]."""
        while angle >= 2 * math.pi:
            angle -= 2 * math.pi
        while angle < 0:
            angle += 2 * math.pi
        return angle

    def amr_callback(self, msg: Amr):
        """Callback processing velocity and IMU yaw input."""
        self.velocity = msg.velocity
        self.yaw_deg = msg.yaw

        raw_theta = math.radians(self.yaw_deg)

        if not self.calibrated:
            self.theta_offset = raw_theta
            self.calibrated = True
            self.get_logger().info(
                f"IMU heading calibrated. Offset: {self.yaw_deg:.2f} degrees"
            )

        self.theta_imu = self.normalize_angle(raw_theta - self.theta_offset)

    def update_pose(self):
        """Integrates linear velocity to update pose estimate."""
        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0 or dt > 1.0:
            return

        # Use IMU heading directly
        self.theta = self.theta_imu

        # Dead-reckoning integration
        self.x += self.velocity * math.cos(self.theta) * dt
        self.y += self.velocity * math.sin(self.theta) * dt

        # Prepare outgoing message
        pose_msg = Vector3Stamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = "amr_pose"
        pose_msg.vector.x = self.x
        pose_msg.vector.y = self.y
        pose_msg.vector.z = self.theta

        # Publish pose
        self.pose_pub.publish(pose_msg)

        theta_deg = math.degrees(self.theta)
        print(
            f"\rPosition: X={self.x:.3f}  Y={self.y:.3f}  "
            f"Heading={theta_deg:.1f} deg  Vel={self.velocity:.3f} m/s   ",
            end=''
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n==================================================")
        print(" POSE ESTIMATOR STOPPED")
        print("==================================================")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

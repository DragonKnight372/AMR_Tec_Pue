#!/usr/bin/env python3
"""
=======================================================================
 Qcar Pose Node--IMU BNO055- Smart Mobility 
 Author: Abraham Moro-Hernandez (AMH19)
 Co-author: Ivan Valdez del Toro
-----------------------------------------------------------------------
 QCar Pose Estimator
 Combines encoder-based linear velocity with IMU yaw measurement to 
 compute a 2D pose estimate of the QCar platform.

 Subscribed Topics:
     /qcar/velocity   (Vector3Stamped)
         Linear velocity from wheel encoder (vector.x)

     /imu/accel_raw   (TwistStamped)
         IMU data containing yaw in twist.angular.z

 Published Topic:
     /qcar/pose       (Vector3Stamped)
         Estimated pose [x, y, theta], where theta is in radians.
=======================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, TwistStamped
import math


class PoseEstimator(Node):
    """ROS2 node that computes the QCar 2D pose using encoder velocity and IMU yaw."""

    def __init__(self):
        super().__init__('pose_estimator')

        # ----------------------------------------------------------
        # Initial pose state
        # ----------------------------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # heading in radians

        # Integration variables
        self.last_time = None
        self.velocity = 0.0
        self.theta_imu = 0.0

        # IMU yaw calibration offset
        self.theta_offset = None
        self.calibrated = False

        # ----------------------------------------------------------
        # Subscribers
        # ----------------------------------------------------------
        self.vel_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/velocity',
            self.velocity_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            TwistStamped,
            '/imu/accel_raw',
            self.imu_callback,
            10
        )

        # ----------------------------------------------------------
        # Publisher
        # ----------------------------------------------------------
        self.pose_pub = self.create_publisher(
            Vector3Stamped,
            '/qcar/pose',
            10
        )

        # 20 Hz timer
        self.timer = self.create_timer(0.05, self.update_pose)

        self.get_logger().info("=" * 50)
        self.get_logger().info(" QCAR POSE ESTIMATOR STARTED")
        self.get_logger().info("=" * 50)
        self.get_logger().info("Subscribed to:")
        self.get_logger().info("  /qcar/velocity (Vector3Stamped)")
        self.get_logger().info("  /imu/accel_raw (TwistStamped)")
        self.get_logger().info("Publishing:")
        self.get_logger().info("  /qcar/pose (Vector3Stamped)")
        self.get_logger().info("=" * 50)

    # --------------------------------------------------------------
    # Encoder callback
    # --------------------------------------------------------------
    def velocity_callback(self, msg: Vector3Stamped):
        """Reads linear velocity from encoder (vector.x)."""
        self.velocity = msg.vector.x

    # --------------------------------------------------------------
    # Angle normalization [0, 2*pi]
    # --------------------------------------------------------------
    def normalize_angle(self, angle):
        while angle >= 2 * math.pi:
            angle -= 2 * math.pi
        while angle < 0:
            angle += 2 * math.pi
        return angle

    # --------------------------------------------------------------
    # IMU callback
    # --------------------------------------------------------------
    def imu_callback(self, msg: TwistStamped):
        """Reads yaw (in radians) directly from IMU angular.z."""
        raw_theta = msg.twist.angular.z

        # Calibrate yaw offset on first measurement
        if not self.calibrated:
            self.theta_offset = raw_theta
            self.calibrated = True
            self.get_logger().info(
                f"IMU heading calibrated. Offset: {math.degrees(self.theta_offset):.2f} degrees"
            )

        # Apply offset and normalize
        self.theta_imu = self.normalize_angle(raw_theta - self.theta_offset)

    # --------------------------------------------------------------
    # Pose integration loop
    # --------------------------------------------------------------
    def update_pose(self):
        """Integrates velocity and IMU yaw to estimate position."""
        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        # Compute time delta
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Ignore invalid dt
        if dt <= 0 or dt > 1.0:
            return

        # Use IMU heading
        self.theta = self.theta_imu

        # Dead-reckoning position update
        self.x += self.velocity * math.cos(self.theta) * dt
        self.y += self.velocity * math.sin(self.theta) * dt

        # Publish pose
        pose_msg = Vector3Stamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.vector.x = self.x
        pose_msg.vector.y = self.y
        pose_msg.vector.z = self.theta

        self.pose_pub.publish(pose_msg)

        # Terminal output
        theta_deg = math.degrees(self.theta)
        print(
            f"\rPosition: X={self.x:.3f} Y={self.y:.3f} | "
            f"Heading={theta_deg:.1f} deg | Velocity={self.velocity:.3f} m/s   ",
            end=''
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n" + "=" * 50)
        print(" QCAR POSE ESTIMATOR STOPPED")
        print("=" * 50)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

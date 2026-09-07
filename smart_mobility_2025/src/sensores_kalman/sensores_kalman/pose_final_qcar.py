#!/usr/bin/env python3
"""
=======================================================================
 QCar Unified Pose Node - Smart Mobility 
 Author: Arturo Daniel Sosa-Ceron
 Co-authors: Marmanja, Abraham Moro-Hernandez, Ivan Valdez del Toro.
-----------------------------------------------------------------------
 Computes QCar orientation, trajectory, and full pose estimation using
 wheel velocities and steering commands under a bicycle kinematic model.
=======================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import math


class QCarPose(Node):
    """Unified 2D pose estimator for the QCar platform."""

    def __init__(self):
        super().__init__('qcar_pose')

        # ----------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------
        self.declare_parameter('wheelbase', 0.256)               # meters
        self.declare_parameter('max_steering_angle', 0.3)        # radians
        self.declare_parameter('velocity_threshold', 1e-12)      # m/s noise floor

        self.L = self.get_parameter('wheelbase').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value

        # ----------------------------------------------------------
        # Full robot state
        # ----------------------------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.velocity = 0.0
        self.steering = 0.0

        # Timing
        self.last_time = self.get_clock().now()

        # ----------------------------------------------------------
        # Subscribers
        # ----------------------------------------------------------
        self.velocity_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/velocity',
            self.velocity_callback,
            10
        )

        self.command_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/user_command',
            self.command_callback,
            10
        )

        # ----------------------------------------------------------
        # Publishers
        # ----------------------------------------------------------
        self.pose_pub = self.create_publisher(Vector3Stamped, '/qcar/pose', 10)
        self.orientation_pub = self.create_publisher(Vector3Stamped, '/qcar/orientation', 10)
        self.trajectory_pub = self.create_publisher(Vector3Stamped, '/qcar/trajectory', 10)

        # Integration loop at 50 Hz
        self.timer = self.create_timer(0.02, self.integrate)

        # Startup info
        self.get_logger().info("QCar Unified Pose Node started")
        self.get_logger().info(f"Wheelbase: {self.L} m")
        self.get_logger().info(f"Max steering: {math.degrees(self.max_steering):.1f}°")
        self.get_logger().info(f"Velocity threshold: {self.velocity_threshold}")
        self.get_logger().info("Initial pose: (0, 0, 0°)")

    # --------------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------------
    def velocity_callback(self, msg):
        """Reads linear velocity components vx, vy from encoder node."""
        self.vx = msg.vector.x
        self.vy = msg.vector.y

        raw_velocity = math.sqrt(self.vx**2 + self.vy**2)
        self.velocity = 0.0 if raw_velocity < self.velocity_threshold else raw_velocity

    def command_callback(self, msg):
        """Reads steering angle command (in radians)."""
        self.steering = msg.vector.y

    # --------------------------------------------------------------
    # Integration of bicycle model
    # --------------------------------------------------------------
    def integrate(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Clamp dt
        dt = max(0.001, min(dt, 0.1))

        # 1) Update heading (bicycle model)
        omega = (self.velocity / self.L) * math.sin(self.steering)
        self.theta += omega * dt

        # 2) Update position
        self.x += self.vx * dt
        self.y += self.vy * dt

        # ----------------------------------------------------------
        # Publish orientation
        # ----------------------------------------------------------
        orientation_msg = Vector3Stamped()
        orientation_msg.header.stamp = current_time.to_msg()
        orientation_msg.header.frame_id = 'base_link'
        orientation_msg.vector.z = self.theta
        self.orientation_pub.publish(orientation_msg)

        # ----------------------------------------------------------
        # Publish trajectory
        # ----------------------------------------------------------
        trajectory_msg = Vector3Stamped()
        trajectory_msg.header.stamp = current_time.to_msg()
        trajectory_msg.header.frame_id = 'odom'
        trajectory_msg.vector.x = self.x
        trajectory_msg.vector.y = self.y
        trajectory_msg.vector.z = self.theta
        self.trajectory_pub.publish(trajectory_msg)

        # ----------------------------------------------------------
        # Publish full pose
        # ----------------------------------------------------------
        pose_msg = Vector3Stamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.vector.x = self.x
        pose_msg.vector.y = self.y
        pose_msg.vector.z = self.theta
        self.pose_pub.publish(pose_msg)

        # Terminal logging
        theta_deg = math.degrees(self.theta)
        steering_deg = math.degrees(self.steering)
        self.get_logger().info(
            f"Pos: X={self.x:+.3f}m  Y={self.y:+.3f}m | "
            f"Theta={self.theta:.3f} rad ({theta_deg:+.2f}°) | "
            f"Velocity={self.velocity:.6f} m/s | Steering={steering_deg:+.2f}°"
        )

        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = QCarPose()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("Final Pose:")
        print(f"  X = {node.x:+.3f} m")
        print(f"  Y = {node.y:+.3f} m")
        print(f"  Theta = {node.theta:.3f} rad = {math.degrees(node.theta):+.2f}°")
        print(f"  Total distance: {math.sqrt(node.x**2 + node.y**2):.3f} m")
        print("=" * 60)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

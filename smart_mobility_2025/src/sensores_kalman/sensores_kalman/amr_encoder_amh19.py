#!/usr/bin/env python3
"""
=======================================================================
 AMR Encoder Node - Smart Mobility AMH19
 Author: Abraham Moro-Hernandez (AMH19)
-----------------------------------------------------------------------
 Node Name:
     amr_encoder_amh19

 Description:
     This ROS2 node reads encoder data from an ESP32 through a serial
     interface and publishes measurements using the standard message
     type geometry_msgs/Vector3Stamped.

 Published Topic:
     /amr/encoder       (Vector3Stamped)

 Vector3Stamped Layout:
     vector.x -- distance traveled (meters)
     vector.y -- velocity (m/s)
     vector.z -- total pulses

 Expected Serial JSON Format:
     {
       "pulsos": int,
       "dist": float,
       "vel": float
     }

 Frequency:
     20 Hz (every 50 ms)

 Purpose:
     Provide basic encoder feedback for AMR odometry and motion
     estimation in higher-level control nodes.
=======================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import serial
import json


class EncoderNode(Node):
    """ROS2 node handling serial encoder data and publishing Vector3Stamped."""

    def __init__(self):
        super().__init__('amr_encoder_amh19')

        # === SERIAL PORT CONFIGURATION ===
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise SystemExit

        # === PUBLISHER ===
        self.pub = self.create_publisher(Vector3Stamped, '/amr/encoder', 10)

        # Timer → 20 Hz (50 ms)
        self.timer = self.create_timer(0.05, self.read_serial)

        self.get_logger().info("Node /amr/encoder started — Reading AMH19 ESP32 Encoder")

    # ------------------------------------------------------------------
    #  Read Serial JSON → Publish ROS2 Vector3Stamped
    # ------------------------------------------------------------------
    def read_serial(self):
        try:
            raw = self.ser.readline().decode(errors="ignore").strip()

            if not raw.startswith("{"):
                return  # ignore incomplete fragments

            data = json.loads(raw)

            dist   = float(data["dist"])
            vel    = float(data["vel"])
            pulses = float(data["pulses"])

            # --- Build Message ---
            msg = Vector3Stamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "encoder_frame_amh19"

            msg.vector.x = dist
            msg.vector.y = vel
            msg.vector.z = pulses

            self.pub.publish(msg)

        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON received")
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

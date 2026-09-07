"""
=======================================================================
 Custom Interface Node - Smart Mobility Interface AMR Package (AMH19)
 Author: Abraham Moro-Hernandez (AMH19)
-----------------------------------------------------------------------
 Node Name:
     amr_odom

 Description:
     This node reads fused sensor data from an ESP32 (encoder + IMU)
     through a serial interface and publishes a custom ROS2 message
     "sm_interfaces/msg/Amr" into the topic /amr/odom.

 Published Data:
     • Pulses from encoder
     • Distance travelled (meters)
     • Velocity (m/s)
     • Acceleration (X, Y, Z axes)
     • Orientation (Roll, Pitch, Yaw) in degrees
     
 Purpose:
     Provide odometry data for the AMR1 autonomous platform by
     merging encoder-based linear motion with IMU orientation.
     
 Serial Input Format:
     {
       "pulsos": int,
       "dist": float,
       "vel": float,
       "ax": float,
       "ay": float,
       "az": float,
       "roll": float,
       "pitch": float,
       "yaw": float
     }

=======================================================================
"""



import rclpy
from rclpy.node import Node
from sm_interfaces.msg import Amr
import serial
import json

class EncoderNode(Node):
    def __init__(self):
        super().__init__('amr_odom_19')

        # === OPENING PYSERIAL TO GET DATA FROM ESP32 ===
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # === PUBLISHER ===
        self.pub = self.create_publisher(Amr, '/amr/odom', 10)

        # Timer 50 ms
        self.timer = self.create_timer(0.05, self.leer_serial)

        self.get_logger().info("Nodo /amr/odom iniciado leyendo ESP32 AMH19 Encoder-IMU")

    def leer_serial(self):
        try:
            line = self.ser.readline().decode().strip()

            if not line.startswith("{"):
                return

            data = json.loads(line)

            pulsos = int(data["pulsos"])
            dist   = float(data["dist"])
            vel    = float(data["vel"])
            ax     = float(data["ax"])
            ay     = float(data["ay"])
            az     = float(data["az"])
            roll   = float(data["roll"])
            pitch  = float(data["pitch"])
            yaw    = float(data["yaw"])

            msg = Amr()  
            msg.distance = dist
            msg.velocity = vel
            msg.pulses = pulsos
            msg.acceleration_x = ax
            msg.acceleration_y = ay
            msg.acceleration_z = az
            msg.roll = roll
            msg.pitch = pitch
            msg.yaw = yaw

         

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Error obtaining serial info: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

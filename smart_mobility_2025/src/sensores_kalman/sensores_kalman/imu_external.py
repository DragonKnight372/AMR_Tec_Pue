#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
import serial
import json
import math


class BNO055Publisher(Node):
    def __init__(self):
        super().__init__('imu_bno055_publisher')
        
        # Parámetro para el puerto serial
        self.declare_parameter('port', '/dev/ttyUSB0')
        port = self.get_parameter('port').get_parameter_value().string_value
        
        # Abrir puerto serial
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.get_logger().info(f"✓ Conectado a {port}")
        except Exception as e:
            self.get_logger().error(f"✗ Error abriendo {port}: {e}")
            raise
        
        # Publishers
        # Aceleración cruda (para tu filtro Kalman)
        self.pub_accel = self.create_publisher(
            TwistStamped, '/imu/accel_raw', 10)
        
        # IMU completo (estándar ROS2)
        self.pub_imu = self.create_publisher(
            Imu, '/imu/data', 10)
        
        # Timer para leer serial
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        
        self.get_logger().info("=== BNO055 Publisher iniciado ===")
        self.get_logger().info("Publicando en:")
        self.get_logger().info("  - /imu/accel_raw (TwistStamped)")
        self.get_logger().info("  - /imu/data (Imu)")

    def timer_callback(self):
        # Leer línea del serial
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    return
                
                # Parsear JSON
                data = json.loads(line)
                
                # Verificar que sea un mensaje de datos válido
                if "ax" not in data:
                    return
                
                timestamp = self.get_clock().now().to_msg()
                
                # ===== Publicar TwistStamped (para tu filtro Kalman) =====
                msg_twist = TwistStamped()
                msg_twist.header.stamp = timestamp
                msg_twist.header.frame_id = "imu_traxxas"
                
                # Aceleración lineal (m/s²)
                msg_twist.twist.linear.x = data["ax"]
                msg_twist.twist.linear.y = data["ay"]
                msg_twist.twist.linear.z = data["az"]
                
                # Orientación Euler (grados -> radianes)
                msg_twist.twist.angular.x = math.radians(data["roll"])
                msg_twist.twist.angular.y = math.radians(data["pitch"])
                msg_twist.twist.angular.z = math.radians(data["yaw"])
                
                self.pub_accel.publish(msg_twist)
                
                # ===== Publicar Imu estándar (opcional) =====
                msg_imu = Imu()
                msg_imu.header.stamp = timestamp
                msg_imu.header.frame_id = "imu_traxxas"
                
                # Aceleración lineal
                msg_imu.linear_acceleration.x = data["ax"]
                msg_imu.linear_acceleration.y = data["ay"]
                msg_imu.linear_acceleration.z = data["az"]
                
                # Velocidad angular (rad/s)
                msg_imu.angular_velocity.x = data["gx"]
                msg_imu.angular_velocity.y = data["gy"]
                msg_imu.angular_velocity.z = data["gz"]
                
                # Orientación (convertir Euler a Quaternion)
                # Simplificado: puedes mejorar esto con transformaciones
                roll = math.radians(data["roll"])
                pitch = math.radians(data["pitch"])
                yaw = math.radians(data["yaw"])
                
                # Conversión Euler -> Quaternion
                cy = math.cos(yaw * 0.5)
                sy = math.sin(yaw * 0.5)
                cp = math.cos(pitch * 0.5)
                sp = math.sin(pitch * 0.5)
                cr = math.cos(roll * 0.5)
                sr = math.sin(roll * 0.5)
                
                msg_imu.orientation.w = cr * cp * cy + sr * sp * sy
                msg_imu.orientation.x = sr * cp * cy - cr * sp * sy
                msg_imu.orientation.y = cr * sp * cy + sr * cp * sy
                msg_imu.orientation.z = cr * cp * sy - sr * sp * cy
                
                self.pub_imu.publish(msg_imu)
                
                # Log cada segundo
                if hasattr(self, 'log_counter'):
                    self.log_counter += 1
                else:
                    self.log_counter = 0
                
                if self.log_counter % 50 == 0:  # Cada 50 mensajes (~0.5s)
                    self.get_logger().info(
                        f"ax={data['ax']:.3f} ay={data['ay']:.3f} az={data['az']:.3f} | "
                        f"yaw={data['yaw']:.1f}° pitch={data['pitch']:.1f}° roll={data['roll']:.1f}°")
                
        except json.JSONDecodeError:
            pass  # Ignorar líneas no-JSON
        except KeyError as e:
            self.get_logger().warn(f"Clave faltante en JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
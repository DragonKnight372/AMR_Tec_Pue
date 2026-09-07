#!/usr/bin/env python3
"""
=======================================================================
 QCar Watchdog Safety Node - Smart Mobility 
 Author: Marmanja

-----------------------------------------------------------------------
 Ensures the physical QCar stops safely if user-command messages stop
 arriving within a defined timeout window. If no real commands are 
 received (i.e., not originating from the watchdog itself), a STOP 
 command is injected into /qcar/user_command to guarantee safe halt.
=======================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import math


class QCarWatchdog(Node):
    """Safety watchdog that enforces STOP when command publishing stops."""

    def __init__(self):
        super().__init__('qcar_watchdog')

        # ----------------------------------------------------------
        # Timeout parameter (no commands -> STOP)
        # ----------------------------------------------------------
        self.declare_parameter('timeout', 0.2)  
        self.timeout = (
            self.get_parameter('timeout')
            .get_parameter_value()
            .double_value
        )

        # ----------------------------------------------------------
        # Subscriber + Publisher (same topic)
        # ----------------------------------------------------------
        self.sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/user_command',
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            Vector3Stamped,
            '/qcar/user_command',
            10
        )

        # Timestamp of last real command
        self.last_real_cmd_time = self.get_clock().now()

        # Watchdog timer (20 Hz)
        self.timer = self.create_timer(0.05, self.watchdog_cb)

        self.get_logger().info(
            f"QCarWatchdog started. Timeout = {self.timeout:.3f} s"
        )

    # --------------------------------------------------------------
    # Command Callback
    # --------------------------------------------------------------
    def cmd_callback(self, msg: Vector3Stamped):
        """
        Receives ALL /qcar/user_command messages.
        
        - If the message comes from the controller:
              frame_id != "watchdog"
              => update last_real_cmd_time
        
        - If it comes from this node:
              frame_id == "watchdog"
              => ignore (otherwise infinite reset loop)
        """
        if msg.header.frame_id != "watchdog":
            self.last_real_cmd_time = self.get_clock().now()

    # --------------------------------------------------------------
    # Watchdog Logic
    # --------------------------------------------------------------
    def watchdog_cb(self):
        """
        Checks if the timeout has expired.
        If exceeded -> send STOP command via /qcar/user_command.
        """
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_real_cmd_time.nanoseconds) * 1e-9

        if dt > self.timeout:
            stop = Vector3Stamped()
            stop.header.stamp = now.to_msg()
            stop.header.frame_id = "watchdog"  # mark source
            
            stop.vector.x = 0.0   # velocity STOP
            stop.vector.y = 0.0   # steering neutral
            stop.vector.z = 0.0

            try:
                self.pub.publish(stop)
            except Exception:
                pass


# --------------------------------------------------------------
# MAIN
# --------------------------------------------------------------
def main(args=None):
    # marml 2025
    rclpy.init(args=args)
    node = QCarWatchdog()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

"""
Author: Sachin Kumar
Date: 2025-09-09
Program: Emergency Command Node for Obstacle Avoidance in ROS 2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

SAFETY_THRESHOLD = 0.3  # meters


# Try to import numpy for fast scan processing
try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    np = None
    _HAS_NUMPY = False

class EmergencyCmdNode(Node):
    __slots__ = [
        'cmd_vel_sub', 'emergency_pub', 'scan_sub', 'safety_threshold', 'obstacle_detected',
        '_stop_msg', '_last_obstacle_state', '_logger'
    ]

    def __init__(self):
        """
        Initialize the EmergencyCmdNode, set up subscriptions and publishers, and configure safety threshold.
        """
        super().__init__("emergency_cmd_node")
        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.emergency_pub = self.create_publisher(Twist, "emergency_cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.safety_threshold = SAFETY_THRESHOLD
        self.obstacle_detected = False
        self._last_obstacle_state = False
        self._stop_msg = Twist()  # Reuse stop message
        self._logger = self.get_logger()
        self._logger.info("Emergency Command Node has been started.")

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for cmd_vel topic. Publishes stop command if obstacle detected, otherwise forwards velocity command.
        """
        pub = self.emergency_pub
        if self.obstacle_detected:
            pub.publish(self._stop_msg)
            # Only log if state changed
            if not self._last_obstacle_state:
                self._logger.warn("Obstacle detected! Published STOP command.")
                self._last_obstacle_state = True
        else:
            emergency_msg = Twist()
            emergency_msg.linear.x = msg.linear.x
            emergency_msg.angular.z = msg.angular.z
            pub.publish(emergency_msg)
            # Only log if state changed
            if self._last_obstacle_state:
                self._logger.info(f"Obstacle cleared. Published emergency command: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
                self._last_obstacle_state = False

    def scan_callback(self, msg: LaserScan):
        """
        Callback for scan topic. Checks for obstacles within safety threshold using laser scan data.
        """
        threshold = self.safety_threshold
        ranges = msg.ranges
        detected = False
        if _HAS_NUMPY and np is not None:
            arr = np.array(ranges)
            # Ignore invalid (<=0) readings
            valid = arr[arr > 0]
            if valid.size > 0:
                if (valid < threshold).any():
                    detected = True
        else:
            for distance in ranges:
                if 0 < distance < threshold:
                    detected = True
                    break
        if detected != self.obstacle_detected:
            self.obstacle_detected = detected
            if detected:
                self._logger.warn("Obstacle within safety threshold detected!")
            else:
                self._logger.info("No obstacle within safety threshold.")

def main(args=None):
    """
    Entry point for the EmergencyCmdNode ROS2 node.
    """
    rclpy.init(args=args)
    node = EmergencyCmdNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

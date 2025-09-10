#!/usr/bin/env python3

"""
cmd_pid_controller

script to implement the pid controller

author: Sachin Kumar
date: 2025-08-19
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
from nav_swarm_marl.pid_controller import PidParams, TwistPidController



class CmdVelPIDNode(Node):
    def __init__(self):
        super().__init__("cmd_pid_controller")
        # PID parameters (can be made ROS2 params)
        self.declare_parameter("kp_linear", 1.0)
        self.declare_parameter("ki_linear", 0.1)
        self.declare_parameter("kd_linear", 0.01)
        self.declare_parameter("kp_angular", 1.0)
        self.declare_parameter("ki_angular", 0.1)
        self.declare_parameter("kd_angular", 0.01)

        kp_linear_param = self.get_parameter("kp_linear").value
        ki_linear_param = self.get_parameter("ki_linear").value
        kd_linear_param = self.get_parameter("kd_linear").value
        kp_angular_param = self.get_parameter("kp_angular").value
        ki_angular_param = self.get_parameter("ki_angular").value
        kd_angular_param = self.get_parameter("kd_angular").value

        kp_linear: float = (
            float(kp_linear_param) if kp_linear_param is not None else 1.0
        )
        ki_linear: float = (
            float(ki_linear_param) if ki_linear_param is not None else 0.1
        )
        kd_linear: float = (
            float(kd_linear_param) if kd_linear_param is not None else 0.0
        )
        kp_angular: float = (
            float(kp_angular_param) if kp_angular_param is not None else 1.0
        )
        ki_angular: float = (
            float(ki_angular_param) if ki_angular_param is not None else 0.1
        )
        kd_angular: float = (
            float(kd_angular_param) if kd_angular_param is not None else 0.0
        )

        # Initialize PID controllers with parameters
        l_pid_param = PidParams(kp=kp_linear, ki=ki_linear, kd=kd_linear, setpoint=0.0)
        a_pid_param = PidParams(
            kp=kp_angular, ki=ki_angular, kd=kd_angular, setpoint=0.0
        )

        self.twist_pid_controller = TwistPidController(l_pid_param, a_pid_param)

        self.subscription = self.create_subscription(
            Twist, "/tb2/cmd_vel_in", self.cmd_vel_callback, 10
        )
        self.publisher = self.create_publisher(Twist, "/tb2/cmd_vel", 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, "/tb2/odom", self.odom_callback, 10
        )
        self.out_msg = Twist()
        self.get_logger().info("CmdVelPIDNode has been initialized")
        self.get_logger().info(f"PID parameters set to: {l_pid_param}, {a_pid_param}")

        # Timer for PID compute and publish (e.g., 20 Hz)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.feedback_vel = (0, 0)

        # For dt calculation
        self.last_time = time.time()

    def cmd_vel_callback(self, msg: Twist):
        # Only update setpoints from incoming message
        self.twist_pid_controller.linear_setpoint = msg.linear.x
        self.twist_pid_controller.angular_setpoint = msg.angular.z

    def odom_callback(self, msg: Odometry):
        # Update the PID controller with the current odometry
        self.feedback_vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def timer_callback(self):
        # In real use, measurement should come from odometry or sensors
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        # Here, measurement is assumed zero (replace with real feedback in closed-loop)
        l_out, a_out = (
            self.twist_pid_controller.update(
                self.out_msg.linear.x, self.out_msg.angular.z, dt
            )
        )
        # l_out, a_out = (
        #     self.twist_pid_controller.update(
        #         self.feedback_vel[0], self.feedback_vel[1], dt
        #     )
        # )
        # l_out = l_out * dt
        # a_out = a_out * dt
        self.out_msg.linear.x += l_out * 0.1
        self.out_msg.angular.z += a_out * 0.1

        self.publisher.publish(self.out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

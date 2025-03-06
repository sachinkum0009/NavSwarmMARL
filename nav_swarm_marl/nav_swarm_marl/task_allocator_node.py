#!/usr/bin/env python3

"""
Task Allocator Node

This node will subscribe to the RViz goal pose and publish the task to the task allocator node.

Author: Sachin Kumar
Date: 2025/02/27

Description:
This script provides functionality to allocate tasks among multiple agents in a decentralized manner. 
It is part of the NavSwarmMARL project, which focuses on navigation and task allocation for multi-agent reinforcement learning systems.

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from typing import List
from nav_swarm_marl.models.decentralized_multi_agent_task_allocator import DecentralizedMATA
from nav_swarm_marl.datatypes.robot import Robot
from nav_swarm_marl.datatypes.task import Task
from std_msgs.msg import String

class TaskAllocatorNode(Node):
    def __init__(self):
        super().__init__('task_allocator_node')
        self.robot0_odom = Odometry()
        self.robot1_odom = Odometry()
        self.robot2_odom = Odometry()
        self.initialize_robots()
        self.pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.pose_callback, 10)
        self.task_pub = self.create_publisher(PoseStamped, '/task', 10)
        self.robot0_pose_pub = self.create_publisher(PoseStamped, '/tb3_0/pose', 10)
        self.robot1_pose_pub = self.create_publisher(PoseStamped, '/tb3_1/pose', 10)
        self.robot2_pose_pub = self.create_publisher(PoseStamped, '/tb3_2/pose', 10)
        self.robot0_odom_sub = self.create_subscription(Odometry, '/tb3_0/odom', self.robot0_odom_callback, 10)
        self.robot1_odom_sub = self.create_subscription(Odometry, '/tb3_1/odom', self.robot1_odom_callback, 10)
        self.robot2_odom_sub = self.create_subscription(Odometry, '/tb3_2/odom', self.robot2_odom_callback, 10)
        self.task_done_sub = self.create_subscription(String, '/success', self.task_done_cb, 10)
        self.task_allocator = DecentralizedMATA()
        self.get_logger().info("The task allocator node has just been created")

    def initialize_robots(self):
        self.robot0 = Robot(0, (0,0), False)
        self.robot1 = Robot(1, (0,0), False)
        self.robot2 = Robot(2, (0,0), False)
        self.get_logger().info("Robots initialized")

    def robot0_odom_callback(self, msg: Odometry):
        self.robot0.position = (msg.pose.pose.position.x, msg.pose.pose.position.y) 
    
    def robot1_odom_callback(self, msg: Odometry):
        self.robot1.position = (msg.pose.pose.position.x, msg.pose.pose.position.y) 

    def robot2_odom_callback(self, msg: Odometry):
        self.robot2.position = (msg.pose.pose.position.x, msg.pose.pose.position.y) 
    
    def task_done_cb(self, msg: String):
        if msg.data == "/tb3_0":
            self.robot0.status = False
        elif msg.data == "/tb3_1":
            self.robot1.status = False
        elif msg.data == "/tb3_2":
            self.robot2.status = False

    def pose_callback(self, msg: PoseStamped):
        task = Task(0, (msg.pose.position.x, msg.pose.position.y))
        robot_id = self.task_allocator.allocate_tasks([self.robot0, self.robot1, self.robot2], task)
        self.get_logger().info(f"Task allocated to robot with id: {robot_id}")
        # TODO: Publish the task to the robot with the given id
        if robot_id == -1:
            self.get_logger().error("No robot is available to handle the task")
            return
        self.send_task(robot_id, msg)
    
    def send_task(self, robot_id: int, pose: PoseStamped):
        if robot_id == 0:
            self.robot0.status = True
            self.robot0_pose_pub.publish(pose)
        elif robot_id == 1:
            self.robot1.status = True
            self.robot1_pose_pub.publish(pose)
        elif robot_id == 2:
            self.robot2.status = True
            self.robot2_pose_pub.publish(pose)
    
def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

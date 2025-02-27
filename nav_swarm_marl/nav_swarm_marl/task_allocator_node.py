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

class TaskAllocatorNode(Node):
    def __init__(self):
        super().__init__('task_allocator_node')
        self.robot1_odom = Odometry()
        self.robot2_odom = Odometry()
        self.pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.pose_callback, 1)
        self.task_pub = self.create_publisher(PoseStamped, '/task', 10)
        self.robot1_odom_sub = self.create_subscription(Odometry, '/robot1/odom', self.robot1_odom_callback, 10)
        self.robot2_odom_sub = self.create_subscription(Odometry, '/robot2/odom', self.robot2_odom_callback, 10)
        self.task_allocator = DecentralizedMATA()
        self.get_logger().info("The task allocator node has just been created")

    def initialize_robots(self):
        self.robot1 = Robot(1, (0,0), False)
        self.robot2 = Robot(2, (0,0), False)
        self.get_logger().info("Robots initialized")
    
    def robot1_odom_callback(self, msg: Odometry):
        self.robot1.position = (msg.pose.pose.position.x, msg.pose.pose.position.y) 

    def robot2_odom_callback(self, msg: Odometry):
        self.robot2.position = (msg.pose.pose.position.x, msg.pose.pose.position.y) 

    def pose_callback(self, msg: PoseStamped):
        task = Task(msg.pose.position.x, msg.pose.position.y)
        robot_id = self.task_allocator.allocate_tasks([self.robot1, self.robot2], task)
        self.get_logger().info(f"Task allocated to robot with id: {robot_id}")
        # TODO: Publish the task to the robot with the given id
    
def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

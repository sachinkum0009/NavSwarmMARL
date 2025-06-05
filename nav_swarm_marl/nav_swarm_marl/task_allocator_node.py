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
from rclpy.node import Node, Client
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from typing import List
from nav_swarm_marl.models.decentralized_multi_agent_task_allocator import (
    DecentralizedMATA,
)
from nav_swarm_marl.lib.utils import wait_for_service
from nav_swarm_marl.datatypes.robot import Robot
from nav_swarm_marl.datatypes.task import Task
from nav_swarm_marl.lib.utils import RobotID, wait_for_service
from NavSwarmMARL.nav_swarm_marl.nav_swarm_marl.scenarios.distance_calculator import DistanceCalculator
from std_msgs.msg import String

from nav_swarm_inter.srv import Goal
import numpy as np
import time
import asyncio
import random


class TaskAllocatorNode(Node):
    def __init__(self):
        super().__init__("task_allocator_node")
        self.robot0_odom = Odometry()
        self.robot1_odom = Odometry()
        self.robot2_odom = Odometry()

        self.distance_calculator = DistanceCalculator(3)

        reentrant_callback_group = ReentrantCallbackGroup()

        self.initialize_robots()
        self.pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.pose_callback, 10
        )
        self.scenario_server = self.create_service(
            Goal,
            "/scenario_goal",
            self.scenario_server_cb,
            callback_group=reentrant_callback_group,
        )
        self.tb3_0_task_client = self.create_client(Goal, "/tb3_0/task_server")
        self.tb3_1_task_client = self.create_client(Goal, "/tb3_1/task_server")
        self.tb3_2_task_client = self.create_client(Goal, "/tb3_2/task_server")
        wait_for_service(self, self.tb3_0_task_client)
        wait_for_service(self, self.tb3_1_task_client)
        wait_for_service(self, self.tb3_2_task_client)
        self.goal_req = Goal.Request()
        self.task_pub = self.create_publisher(PoseStamped, "/task", 10)
        self.robot0_pose_pub = self.create_publisher(PoseStamped, "/tb3_0/pose", 10)
        self.robot1_pose_pub = self.create_publisher(PoseStamped, "/tb3_1/pose", 10)
        self.robot2_pose_pub = self.create_publisher(PoseStamped, "/tb3_2/pose", 10)
        self.robot0_odom_sub = self.create_subscription(
            Odometry, "/tb3_0/odom", self.robot0_odom_callback, 10
        )
        self.robot1_odom_sub = self.create_subscription(
            Odometry, "/tb3_1/odom", self.robot1_odom_callback, 10
        )
        self.robot2_odom_sub = self.create_subscription(
            Odometry, "/tb3_2/odom", self.robot2_odom_callback, 10
        )
        self.task_done_sub = self.create_subscription(
            String, "/success", self.task_done_cb, 10
        )

        self.task_allocator = DecentralizedMATA()
        self.get_logger().info("The task allocator node has just been created")

    def initialize_robots(self):
        self.robot0 = Robot(0, (0, 0), False)
        self.robot1 = Robot(1, (0, 0), False)
        self.robot2 = Robot(2, (0, 0), True)
        self.get_logger().info("Robots initialized")

    def robot0_odom_callback(self, msg: Odometry):
        self.robot0.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.distance_calculator.update_distance(0, msg.pose.pose.position.x, msg.pose.pose.position.y)

    def robot1_odom_callback(self, msg: Odometry):
        self.robot1.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.distance_calculator.update_distance(1, msg.pose.pose.position.x, msg.pose.pose.position.y)

    def robot2_odom_callback(self, msg: Odometry):
        self.robot2.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.distance_calculator.update_distance(2, msg.pose.pose.position.x, msg.pose.pose.position.y)

    def task_done_cb(self, msg: String):
        if msg.data == "/tb3_0":
            self.robot0.status = False
        elif msg.data == "/tb3_1":
            self.robot1.status = False
        elif msg.data == "/tb3_2":
            self.robot2.status = False

    def scenario_server_cb(self, req: Goal.Request, res: Goal.Response):
        print(f"Received request: {req}")
        task = Task(req.scenario_id, (req.x, req.y))
        robot_id = self.task_allocator.allocate_tasks(
            [self.robot0, self.robot1, self.robot2], task
        )
        self.get_logger().info(f"Task allocated to robot with id: {robot_id}")
        if robot_id == -1:
            self.get_logger().error("No robot is available to handle the task")
            res.success = False
            return res
        result = self.send_task_xy(robot_id, req.x, req.y)
        if isinstance(result, bool):
            res.success = False
        elif isinstance(result, Goal.Response):
            res.success = result.success
            res.message = result.message
        return res

    def pose_callback(self, msg: PoseStamped):
        task = Task(0, (msg.pose.position.x, msg.pose.position.y))
        robot_id = self.task_allocator.allocate_tasks(
            [self.robot0, self.robot1, self.robot2], task
        )
        self.get_logger().info(f"Task allocated to robot with id: {robot_id}")
        # TODO: Publish the task to the robot with the given id
        if robot_id == -1:
            self.get_logger().error("No robot is available to handle the task")
            return
        self.send_task(robot_id, msg)

    def _response_callback(self, future):
        if future.result() is not None:
            self.get_logger().info(f"Received response: {future.result()}")
            res: Goal.Response = future.result()
            if res.message == "0":
                self.robot0.status = False
            elif res.message == "1":
                self.robot1.status = False
            elif res.message == "2":
                self.robot2.status = False
        else:
            self.get_logger().error("Service call failed")

    def send_task(self, robot_id: int, pose: PoseStamped) -> None:
        # self.goal_req.scenario_id = robot_id
        # self.goal_req.x = pose.pose.position.x
        # self.goal_req.y = pose.pose.position.y
        # self.goal_req.theta = 2 * np.atan2(
        #     pose.pose.orientation.z, pose.pose.orientation.w
        # )
        # if robot_id == 0:
        #     self.robot0.status = True
        #     future = self.tb3_0_task_client.call_async(self.goal_req)
        #     future.add_done_callback(self._response_callback)
        # elif robot_id == 1:
        #     self.robot1.status = True
        #     future = self.tb3_1_task_client.call_async(self.goal_req)
        #     future.add_done_callback(self._response_callback)
        # elif robot_id == 2:
        #     self.robot2.status = True
        #     future = self.tb3_2_task_client.call_async(self.goal_req)
        #     future.add_done_callback(self._response_callback)
        # else:
        #     self.get_logger().info("no robot id found")
        return None

    def send_task_xy(self, robot_id: int, x: float, y: float) -> bool | Goal.Response:
        future = Future()
        self.goal_req = Goal.Request()
        self.goal_req.scenario_id = robot_id
        self.goal_req.x = x
        self.goal_req.y = y
        self.goal_req.theta = 0.0
        if robot_id == 0:
            self.robot0.status = True
            if self.tb3_0_task_client.service_is_ready():
                self.get_logger().info("tb3_0 task client is ready")
                response = self.tb3_0_task_client.call(self.goal_req)
                self.robot0.status = False
                return response
                # future = self.tb3_0_task_client.call_async(self.goal_req)
            else:
                return False
            # future.add_done_callback(self._response_callback)
        elif robot_id == 1:
            self.robot1.status = True
            if self.tb3_1_task_client.service_is_ready():
                self.get_logger().info("tb3_1 task client is ready")
                response = self.tb3_1_task_client.call(self.goal_req)
                self.robot1.status = False
                return response
                # future = self.tb3_1_task_client.call_async(self.goal_req)
            else:
                return False
            # future.add_done_callback(self._response_callback)
        elif robot_id == 2:
            self.robot2.status = True
            if self.tb3_2_task_client.service_is_ready():
                self.get_logger().info("tb3_2 task client is ready")
                response = self.tb3_2_task_client.call(self.goal_req)
                self.robot2.status = False
                return response
                # future = self.tb3_2_task_client.call_async(self.goal_req)
            else:
                return False
            # future.add_done_callback(self._response_callback)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            self.get_logger().error("Failed to get a valid response from the service")
            return False
        response: Goal.Response = result
        response.distance = self.distance_calculator.get_distance(robot_id)
        print(f"Received response: {response}")
        return response

    def test_server(self):
        pose = PoseStamped()
        self.send_task(0, pose)
        self.send_task(1, pose)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = TaskAllocatorNode()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

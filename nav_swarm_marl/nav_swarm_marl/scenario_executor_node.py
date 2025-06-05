#!/usr/bin/env python3

"""
Author: Sachin Kumar
Date: 2025-03-26
Program: Scenario Executor Node for Multi-Robot Scenarios in ROS 2

Description:
This script defines a ROS 2 node, `ScenarioExecutorNode`, which is responsible for executing
multi-robot scenarios defined in a YAML file. The node integrates with the `MultiRobotScenario`
class to load and run scenarios, and provides a service to start the execution of these scenarios.
It publishes goal positions for robots and simulates delays to mimic real-world behavior.

Classes:
    - ScenarioExecutorNode: A ROS 2 node that executes multi-robot scenarios.

Functions:
    - main(args=None): Entry point for the ROS 2 node.

ScenarioExecutorNode:
    - __init__(file_path: str): Initializes the node and loads the scenario file.
    - start_scenario_callback(req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        Service callback to start the scenario execution.
    - run_scenario() -> bool: Executes the loaded scenarios and publishes goal positions.

Usage:
    Replace the `file_path` variable in the `main` function with the actual path to your scenario YAML file.
    Run the script to start the ROS 2 node and interact with the `/start_scenario` service to execute scenarios.
"""


import rclpy
from geometry_msgs.msg import PoseStamped
from nav_swarm_marl.scenarios.multi_robot_scenario import MultiRobotScenario
from rclpy.node import Node
from std_srvs.srv import SetBool
import time
from nav_swarm_inter.srv import Goal
from rclpy import Future
from nav_swarm_marl.lib.utils import wait_for_service
from typing import List, Tuple
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

TIMEOUT_SCENARIO_EXECUTION = 20.0  # seconds


class ScenarioExecutorNode(Node, MultiRobotScenario):
    """
    ScenarioExecutorNode is a ROS 2 node that manages and executes multi-robot scenarios.
    This node inherits from both `Node` and `MultiRobotScenario` classes. It provides a service
    to start the execution of predefined scenarios and publishes goal positions for robots.
    Attributes:
        srv (Service): A ROS 2 service that listens to `/start_scenario` requests to start the scenario.
        goal_pub (Publisher): A ROS 2 publisher that publishes `PoseStamped` messages to the `/goal` topic.
        goal_pose (PoseStamped): A message object used to define and publish goal positions for robots.
    Methods:
        __init__(file_path: str):
            Initializes the ScenarioExecutorNode, sets up the service, publisher, and logger.
        start_scenario_callback(req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
            Callback function for the `/start_scenario` service. Starts the scenario execution
            and returns a response indicating success or failure.
        run_scenario() -> bool:
            Executes the predefined scenarios by iterating through their points and publishing
            goal positions. Introduces delays if specified in the scenario configuration.
            Returns True if the scenario was executed successfully, otherwise False.
    """

    def __init__(self, file_path: str):
        Node.__init__(self, "scenario_executor_node")
        MultiRobotScenario.__init__(self, file_path)

        self.srv = self.create_service(
            SetBool, "/start_scenario", self.start_scenario_callback
        )
        self.scenario_client = self.create_client(Goal, "/scenario_goal")
        wait_for_service(self, self.scenario_client)
        self.scenario_req = Goal.Request()
        self.goal_pub = self.create_publisher(PoseStamped, "/goal", 10)
        self.goal_pose = PoseStamped()
        self.get_logger().info("Scenario Executor Node initialized.")
        self.start_scenario_callback(SetBool.Request(), SetBool.Response())

    def send_goal_request(self):
        request = Goal.Request()
        # Fill in request fields as needed, e.g.:
        # request.target = ...
        request.x = 1.0
        request.scenario_id = 3
        future = self.scenario_client.call_async(request)
        # rclpy.spin_until_future_complete(
        #     self, future, timeout_sec=2.0
        # )
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None:
            self.get_logger().error("Service call returned None.")
            return False
        self.get_logger().info(f"Response: {response}")

    def start_scenario_callback(
        self, req: SetBool.Request, res: SetBool.Response
    ) -> SetBool.Response:
        """
        Callback for the service to start the scenario
        """
        self.get_logger().info("Starting scenario...")
        if self.run_scenario():
            res.success = True
            res.message = "Scenario started successfully."
            self.print_report()
        else:
            res.success = False
            res.message = "Failed to start scenario."

        return res

    def run_scenario(self) -> bool:
        """
        Run the scenario
        :return: True if the scenario was run successfully, False otherwise
        """
        scenario_id = 0
        for scenario in self.scenarios:
            self.scenario_req = Goal.Request()
            self.get_logger().info(f"Running scenario: {scenario.name}")
            self.scenario_req.scenario_name = scenario.name
            future_list: List[Future] = list()

            for point in scenario.points:
                self.scenario_req.scenario_id = scenario_id
                self.scenario_req.x = float(point[0])
                self.scenario_req.y = float(point[1])
                self.scenario_req.theta = 0.0

                if self.scenario_client.service_is_ready():
                    self.get_logger().info("Scenario client is ready")
                else:
                    self.get_logger().error("Scenario client is not ready")
                future = self.scenario_client.call_async(self.scenario_req)
                future_list.append(future)
                self.get_logger().info(
                    f"Sending goal to scenario client: {point[0]}, {point[1]}"
                )
                if scenario.delay:
                    time.sleep(
                        scenario.delay
                    )  # Simulate some delay for the robot to reach the point

            # Wait for all futures to complete
            for future in future_list:
                rclpy.spin_until_future_complete(
                    self, future, timeout_sec=TIMEOUT_SCENARIO_EXECUTION
                )
                self.get_logger().info(f"future result: {future.result()}")
                if future.result() is None or future.done() is False:
                    self.get_logger().error("Service call failed.")
                    # return False
                else:
                    result = future.result()
                    # if result is None:
                    #     self.get_logger().error("Service call returned None.")
                    #     # return False
                    response: Goal.Response = result
                    self.get_logger().info(
                        f"Scenario response: {response.success}, {response.message}"
                    )
                    if response.success is True:
                        self.increment_reached_points(scenario_id)
                        self.update_distance(scenario_id, response.distance)

            scenario_id += 1

        self.get_logger().info("Scenario completed.")
        return True


def main(args=None):
    rclpy.init(args=args)
    file_path: str = (
        "/media/asus/backup/zzzzz/ros2/rtw_workspaces/humble_ws/src/NavSwarmMARL/nav_swarm_marl/nav_swarm_marl/scenarios/multi_robot_scenario.yaml"
    )
    # executor = MultiThreadedExecutor()
    node = ScenarioExecutorNode(file_path)
    # executor.add_node(node)
    # executor.spin()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

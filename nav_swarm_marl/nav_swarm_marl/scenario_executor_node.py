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
from rclpy.wait_for_message import wait_for_message
from std_srvs.srv import SetBool
import time


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
        self.goal_pub = self.create_publisher(PoseStamped, "/goal", 10)
        self.goal_pose = PoseStamped()
        self.get_logger().info("Scenario Executor Node initialized.")

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
        for scenario in self.scenarios:
            self.get_logger().info(f"Running scenario: {scenario.name}")
            for point in scenario.points:
                self.goal_pose.pose.position.x = point[0]
                self.goal_pose.pose.position.y = point[1]

                self.goal_pub.publish(
                    self.goal_pose
                )  # replace this call with service call
                if scenario.delay:
                    time.sleep(
                        scenario.delay
                    )  # Simulate some delay for the robot to reach the point

        self.get_logger().info("Scenario completed.")
        return True


def main(args=None):
    rclpy.init(args=args)
    file_path = (
        "path/to/scenario.yaml"  # Replace with the actual path to your scenario file
    )
    node = ScenarioExecutorNode(file_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

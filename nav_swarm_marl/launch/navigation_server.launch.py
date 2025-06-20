#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="nav_swarm_marl",
                executable="navigation_server_node",
                name="navigation_server_tb3_0",
                output="screen",
                namespace="tb3_0",
                remappings=[("scan", "filter_scan")],
            ),
            Node(
                package="nav_swarm_marl",
                executable="navigation_server_node",
                name="navigation_server_tb3_1",
                output="screen",
                namespace="tb3_1",
                remappings=[("scan", "filter_scan")],
            ),
            Node(
                package="nav_swarm_marl",
                executable="navigation_server_node",
                name="navigation_server_tb3_2",
                output="screen",
                namespace="tb3_2",
                remappings=[("scan", "filter_scan")],
            ),
            Node(
                package="hospital_robot_spawner",
                executable="path_publisher",
                name="path_publisher_tb3_0",
                output="screen",
                namespace="tb3_0",
            ),
            Node(
                package="hospital_robot_spawner",
                executable="path_publisher",
                name="path_publisher_tb3_1",
                output="screen",
                namespace="tb3_1",
            ),
            Node(
                package="hospital_robot_spawner",
                executable="path_publisher",
                name="path_publisher_tb3_2",
                output="screen",
                namespace="tb3_2",
            ),
        ]
    )

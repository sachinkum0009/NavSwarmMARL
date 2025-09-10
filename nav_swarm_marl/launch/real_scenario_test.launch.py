#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    static_tfs_launch_action = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("nav_swarm_marl"),
            "launch",
            "static_tfs.launch.py",
        ),
    )
    rl_inference_launch_action = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("nav_swarm_marl"),
            "launch",
            "rl_inference_server.launch.py",
        ),
    )

    filter_scan_launch_action = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("nav_swarm_marl"),
            "launch",
            "laserscan_filter.launch.py",
        ),
    )

    task_allocator_node = Node(
        package="nav_swarm_marl",
        executable="task_allocator_node",
        name="task_allocator_node",
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(static_tfs_launch_action)
    ld.add_action(rl_inference_launch_action)
    ld.add_action(filter_scan_launch_action)
    ld.add_action(task_allocator_node)

    return ld

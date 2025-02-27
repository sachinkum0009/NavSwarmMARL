#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_swarm_marl',
            executable='turtlebot3_bringup',
            name='turtlebot3_bringup',
            output='screen',
            emulate_tty=True
        ),
    ])
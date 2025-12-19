#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_rl',
            executable='trained_agent_manual_rviz',
            name='rl_inference_server',
            output='screen',
            # Inference server for trained RL agents
        ),
    ])
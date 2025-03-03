#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf_tb3_0',
            output='screen',
            arguments=['0', '0', '0.01', '0', '0', '0', 'map', 'tb3_0/odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf_tb3_1',
            output='screen',
            arguments=['0', '0', '0.01', '0', '0', '0', 'map', 'tb3_1/odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf_tb3_2',
            output='screen',
            arguments=['0', '0', '0.01', '0', '0', '0', 'map', 'tb3_2/odom']
        ),
    ])
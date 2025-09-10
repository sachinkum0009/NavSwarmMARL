#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='hospital_robot_spawner',
        #     executable='trained_agent_manual_rviz',
        #     name='trained_agent_tb3_0',
        #     output='screen',
        #     namespace='tb3_0',
        #     # launch_prefix="/media/asus/backup/zzzzz/ros2/rtw_workspaces/humble_ws/rl/bin/python3",
        #     # arguments=['0', '0', '0.01', '0', '0', '0', 'map', 'tb3_0/odom']
        # ),

        # Node(
        #     package='hospital_robot_spawner',
        #     executable='trained_agent_manual_rviz',
        #     name='trained_agent_tb3_1',
        #     output='screen',
        #     namespace='tb3_1',
        #     # launch_prefix="/media/asus/backup/zzzzz/ros2/rtw_workspaces/humble_ws/rl/bin/python3",
        #     # arguments=['0', '0', '0.01', '0', '0', '0', 'map', 'tb3_0/odom']
        # ),
        Node(
            package='turtlebot_rl',
            executable='trained_agent_manual_rviz_real',
            name='trained_agent_tb2',
            output='screen',
            namespace='tb2',
            remappings=[
                ('scan', 'filter_scan')
            ]
            # launch_prefix="/media/asus/backup/zzzzz/ros2/rtw_workspaces/humble_ws/rl/bin/python3",
            # arguments=['0', '0', '0.01', '0', '0', '0', 'map', 'tb3_0/odom']
        ),

        # Node(
        #     package='hospital_robot_spawner',
        #     executable='path_publisher',
        #     name='path_publisher_tb3_0',
        #     output='screen',
        #     namespace='tb3_0',
        # ),
        # Node(
        #     package='hospital_robot_spawner',
        #     executable='path_publisher',
        #     name='path_publisher_tb3_1',
        #     output='screen',
        #     namespace='tb3_1',
        # ),
        # Node(
        #     package='hospital_robot_spawner',
        #     executable='path_publisher',
        #     name='path_publisher_tb3_2',
        #     output='screen',
        #     namespace='tb3_2',
        # ),
    ])

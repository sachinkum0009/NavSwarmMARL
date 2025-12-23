#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    # world_file_name = 'hospital.world'
    # world_file_name = LaunchConfiguration('world_file', default='hospital.world')
    # world_file_path = os.path.join(get_package_share_directory('aws_robomaker_small_warehouse_world'), 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world')
    pkg_dir = get_package_share_directory('turtlebot_rl')
    # world_file_path = "/workspace/src/aws-robomaker-small-warehouse-world/worlds/small_warehouse/small_warehouse.world"
    world_file_path = "/workspace/src/SwarmNavRL/turtlebot_rl/worlds/bit_lab.world"

    # os.environ["GAZEBO_MODEL_PATH"] = os.environ.get("GAZEBO_MODEL_PATH", "") + os.pathsep + os.path.join(pkg_dir, 'models')
    #os.environ["GAZEBO_RESOURCE_PATH"] = os.path.join(pkg_dir, 'worlds')

    # world = PathJoinSubstitution([pkg_dir, 'worlds', world_file_name])
    launch_file_dir = os.path.join(pkg_dir, 'launch')

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')

    return LaunchDescription([
        gazebo,
    ])

# def generate_launch_description():
#     use_sim_time = LaunchConfiguration('use_sim_time', default='True')

#     world_file_name = LaunchConfiguration('world_file', default='hospital.world')
#     pkg_dir = get_package_share_directory('turtlebot_rl')

#     os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')
#     os.environ["GAZEBO_RESOURCE_PATH"] = os.path.join(pkg_dir, 'worlds')

#     world = PathJoinSubstitution([pkg_dir, 'worlds', world_file_name])
#     launch_file_dir = os.path.join(pkg_dir, 'launch')

#     gazebo = ExecuteProcess(
#             cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so',
#             '-s', 'libgazebo_ros_factory.so'],
#             output='screen')

#     return LaunchDescription([
#         gazebo,
#     ])

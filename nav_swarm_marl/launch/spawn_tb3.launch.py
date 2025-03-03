#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    namespace = 'robot1'
    pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')

    TURTLEBOT3_MODEL = 'waffle'
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    tb3_0_urdf_path = os.path.join(pkg_dir, 'models', model_folder, 'model_tb3_0.sdf')
    tb3_1_urdf_path = os.path.join(pkg_dir, 'models', model_folder, 'model_tb3_1.sdf')
    tb3_2_urdf_path = os.path.join(pkg_dir, 'models', model_folder, 'model_tb3_2.sdf')

    # Launch configuration variables
    tb3_0_x_pose = LaunchConfiguration('tb3_0_x_pose', default='1.0')
    tb3_0_y_pose = LaunchConfiguration('tb3_0_y_pose', default='16.0')

    tb3_1_x_pose = LaunchConfiguration('tb3_1_x_pose', default='3.0')
    tb3_1_y_pose = LaunchConfiguration('tb3_1_y_pose', default='16.0')

    tb3_2_x_pose = LaunchConfiguration('tb3_2_x_pose', default='5.0')
    tb3_2_y_pose = LaunchConfiguration('tb3_2_y_pose', default='16.0')


    tb3_0_declare_x_position_cmd = DeclareLaunchArgument('tb3_0_x_pose', default_value='0.0', description='Specify x position of the robot')
    tb3_0_declare_y_position_cmd = DeclareLaunchArgument('tb3_0_y_pose', default_value='0.0', description='Specify y position of the robot')

    tb3_1_declare_x_position_cmd = DeclareLaunchArgument('tb3_1_x_pose', default_value='0.0', description='Specify x position of the robot')
    tb3_1_declare_y_position_cmd = DeclareLaunchArgument('tb3_1_y_pose', default_value='0.0', description='Specify y position of the robot')

    tb3_2_declare_x_position_cmd = DeclareLaunchArgument('tb3_2_x_pose', default_value='0.0', description='Specify x position of the robot')
    tb3_2_declare_y_position_cmd = DeclareLaunchArgument('tb3_2_y_pose', default_value='0.0', description='Specify y position of the robot')

    # Spawn TurtleBot3 with namespace
    spawn_entity_tb3_0 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_0',  # Ensures unique entity name
            '-file', tb3_0_urdf_path,
            '-x', '1.0',
            '-y', '16.0',
            '-z', '0.01',
            '-Y', '-1.5708'
        ],
        name='spawn_entity_tb3_0',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )


    spawn_entity_tb3_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_1',  # Ensures unique entity name
            '-file', tb3_1_urdf_path,
            '-x', '1.0',
            '-y', '11.0',
            '-z', '0.01',
            '-Y', '-1.5708'
        ],
        name='spawn_entity_tb3_1',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )


    spawn_entity_tb3_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_2',  # Ensures unique entity name
            '-file', tb3_2_urdf_path,
            '-x', '3.0',
            '-y', '9.0',
            '-z', '0.01',
            '-Y', '-1.5708'
        ],
        name='spawn_entity_tb3_2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Robot State Publisher with namespace
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     namespace=namespace,
    #     parameters=[{
    #         'robot_description': urdf_path,
    #         'use_sim_time': True
    #     }],
    #     remappings=[
    #         ('/tf', f'{namespace}/tf'),
    #         ('/tf_static', f'{namespace}/tf_static')
    #     ]
    # )

    return LaunchDescription([
        tb3_0_declare_x_position_cmd,
        tb3_0_declare_y_position_cmd,
        spawn_entity_tb3_0,
        tb3_1_declare_x_position_cmd,
        tb3_1_declare_y_position_cmd,
        spawn_entity_tb3_1,
        tb3_2_declare_x_position_cmd,
        tb3_2_declare_y_position_cmd,
        spawn_entity_tb3_2,
        # robot_state_publisher
    ])

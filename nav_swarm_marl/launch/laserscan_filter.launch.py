from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tb3_0_laser_filter_node = Node(
        package='nav_swarm_marl',
        executable='laserscan_filter_node',
        name='laserscan_filter_node',
        output='screen',
        namespace='tb0',
        arguments=['--ros-args', '--log-level', 'WARN'],
        remappings=[
            ('scan_in', 'scan'),
            ('scan_out', 'filter_scan')
        ]
    )
    tb3_1_laser_filter_node = Node(
        package='nav_swarm_marl',
        executable='laserscan_filter_node',
        name='laserscan_filter_node',
        output='screen',
        namespace='tb1',
        arguments=['--ros-args', '--log-level', 'WARN'],
        remappings=[
            ('scan_in', 'scan'),
            ('scan_out', 'filter_scan')
        ]
    )
    tb3_2_laser_filter_node = Node(
        package='nav_swarm_marl',
        executable='laserscan_filter_node',
        name='laserscan_filter_node',
        output='screen',
        namespace='tb2',
        arguments=['--ros-args', '--log-level', 'WARN'],
        remappings=[
            ('scan_in', 'scan'),
            ('scan_out', 'filter_scan')
        ]
    )
    return LaunchDescription([
        tb3_0_laser_filter_node,
        tb3_1_laser_filter_node,
        tb3_2_laser_filter_node,
    ])

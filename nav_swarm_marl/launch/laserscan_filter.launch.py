from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    laser_filter_node = Node(
        package='nav_swarm_marl',
        executable='laserscan_filter_node',
        name='laserscan_filter_node',
        output='screen',
        # parameters=[{
        #     'scan_topic': '/scan_in',
        #     'filtered_scan_topic': '/scan_out'
        # }],
        remappings=[
            ('/scan_in', '/scan'),
            ('/scan_out', '/filter_scan')
        ]
    )
    return LaunchDescription([
        laser_filter_node,
    ])

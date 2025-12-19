from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tb3_0_cmd_pid_controller_node = Node(
        package='nav_swarm_marl',
        executable='cmd_pid_controller_node',
        name='cmd_pid_controller_node',
        output='screen',
        namespace='tb0',
        arguments=['--ros-args', '--log-level', 'WARN'],
        remappings=[
            ('cmd_vel_in', 'cmd_vel_in'),
            ('cmd_vel_out', 'cmd_vel')
        ]
    )

    tb3_1_cmd_pid_controller_node = Node(
        package='nav_swarm_marl',
        executable='cmd_pid_controller_node',
        name='cmd_pid_controller_node',
        output='screen',
        namespace='tb1',
        arguments=['--ros-args', '--log-level', 'WARN'],
        remappings=[
            ('cmd_vel_in', 'cmd_vel_in'),
            ('cmd_vel_out', 'cmd_vel')
        ]
    )

    tb3_2_cmd_pid_controller_node = Node(
        package='nav_swarm_marl',
        executable='cmd_pid_controller_node',
        name='cmd_pid_controller_node',
        output='screen',
        namespace='tb2',
        arguments=['--ros-args', '--log-level', 'WARN'],
        remappings=[
            ('cmd_vel_in', 'cmd_vel_in'),
            ('cmd_vel_out', 'cmd_vel')
        ]
    )
    return LaunchDescription([
        tb3_0_cmd_pid_controller_node,
        tb3_1_cmd_pid_controller_node,
        tb3_2_cmd_pid_controller_node,
    ])

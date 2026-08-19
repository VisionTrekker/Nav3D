from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scan_context_loop',
            executable='scan_context_loop_node',
            name='scan_context_loop_node',
            output='screen',
        ),
    ])

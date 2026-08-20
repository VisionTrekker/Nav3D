from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='goal_marker_server',
            executable='goal_marker_node',
            name='goal_marker_node',
            output='screen',
        ),
    ])

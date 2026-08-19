from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='global_planner',
            executable='global_planner_node',
            name='global_planner_node',
            output='screen',
        ),
    ])

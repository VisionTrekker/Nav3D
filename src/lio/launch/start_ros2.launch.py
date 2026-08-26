#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Use ament_index for synchronous absolute path (Humble's
    # FindPackageShare.perform() requires a LaunchContext, which is
    # unavailable at description-build time).
    package_share = get_package_share_directory("lio")
    config_path = LaunchConfiguration("config_path")
    use_rviz = LaunchConfiguration("use_rviz")

    # Load frame_ids.yaml params — must be an absolute path for --params-file
    frame_ids_yaml = os.path.join(package_share, "yaml", "frame_ids.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value="root_config.yaml"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        Node(
            package="lio",
            executable="lio",
            name="lio_node",
            output="screen",
            parameters=[{"config_path": config_path}],
            arguments=["--params-file", frame_ids_yaml],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("lio"), "rviz", "LIO_ros2.rviz"])],
            condition=IfCondition(use_rviz),
        ),
    ])

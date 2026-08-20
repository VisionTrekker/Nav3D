#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    package_share = FindPackageShare("lio")
    config_path = LaunchConfiguration("config_path")
    use_rviz = LaunchConfiguration("use_rviz")

    # Load frame_ids.yaml params — must be an absolute path for --params-file
    frame_ids_yaml = os.path.join(
        package_share.perform(), "yaml", "frame_ids.yaml"
    )

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
            arguments=["-d", PathJoinSubstitution([package_share, "rviz", "LIO_ros2.rviz"])],
            condition=IfCondition(use_rviz),
        ),
    ])

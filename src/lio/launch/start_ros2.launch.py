#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


MAPPING_REMAPS = [
    ("/LIO/set_elevator_flag", "/lio/mapping/set_elevator_flag"),
    ("/LIO/odom_imu", "/lio/mapping/odom_imu"),
    ("/LIO/odom_vehicle", "/lio/mapping/odom_body"),
    ("/LIO/clouds_lidar", "/lio/mapping/clouds_lidar"),
    ("/LIO/clouds_lidar/effect", "/lio/mapping/clouds_lidar/effect"),
    ("/LIO/clouds_lidar/reject", "/lio/mapping/clouds_lidar/reject"),
    ("/LIO/global_map", "/lio/mapping/global_map"),
    ("/LIO/ikdtree", "/lio/mapping/ikdtree"),
    ("/LIO/in_elevator", "/lio/mapping/in_elevator"),
    ("/LIO/elevator_state", "/lio/mapping/elevator_state"),
]


def generate_launch_description():
    package_share = FindPackageShare("lio")
    config_path = LaunchConfiguration("config_path")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value="root_config.yaml"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        Node(
            package="lio",
            executable="lio",
            name="lio_node",
            output="screen",
            parameters=[{"config_path": config_path}],
            remappings=MAPPING_REMAPS,
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

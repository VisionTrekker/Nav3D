"""Standalone SCAN Mode 3 validation with mock odom, sensor pose, cloud and path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    frame_id = LaunchConfiguration("frame_id")

    planner_overrides = {
        "use_sim_time": False,
        "fsm.navi_mode": 3,
        "grid_map.sensor_type": "lidar",
        "grid_map.cloud_is_world": True,
        "grid_map.need_extrinsic": False,
        "grid_map.frame_id": frame_id,
        "grid_map.sliding_map_frame_id": "sliding_map",
        "grid_map.body_height": 0.4,
    }

    mock_io = Node(
        package="scan_planner",
        executable="scan_mode3_mock_io.py",
        name="scan_mode3_mock_io",
        output="screen",
        parameters=[{"frame_id": frame_id}],
        remappings=[
            ("body_pose", "/scan_mode3/body_pose"),
            ("sensor_pose", "/scan_mode3/sensor_pose"),
            ("cloud", "/scan_mode3/cloud"),
            ("initial_path", "/scan_mode3/initial_path"),
            ("planning/bspline", "/planning/bspline"),
            ("cmd_vel", "/scan_mode3/cmd_vel"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("frame_id", default_value="world"),
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml, planner_overrides],
            remappings=[
                ("body_pose", "/scan_mode3/body_pose"),
                ("sensor_pose", "/scan_mode3/sensor_pose"),
                ("cloud", "/scan_mode3/cloud"),
                ("initial_path", "/scan_mode3/initial_path"),
            ],
        ),
        Node(
            package="scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            output="screen",
            parameters=[controllers_yaml, {"use_sim_time": False}],
            remappings=[
                ("planning/bspline", "/planning/bspline"),
                ("body_pose", "/scan_mode3/body_pose"),
                ("cmd_vel", "/scan_mode3/cmd_vel"),
            ],
        ),
        mock_io,
        RegisterEventHandler(
            OnProcessExit(
                target_action=mock_io,
                on_exit=[Shutdown(reason="SCAN Mode 3 mock validation finished")],
            )
        ),
    ])

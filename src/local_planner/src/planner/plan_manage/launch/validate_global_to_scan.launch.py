"""Validate /global_planner/path -> SCAN Mode 3 -> cmd_vel without touching SCAN core."""

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

    pcd_map_file = LaunchConfiguration("pcd_map_file")
    octomap_output_bt = LaunchConfiguration("octomap_output_bt")

    mock_io = Node(
        package="scan_planner",
        executable="global_to_scan_mock_io.py",
        name="global_to_scan_mock_io",
        output="screen",
        parameters=[{"frame_id": "map"}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "pcd_map_file",
            default_value="/home/nhy/code/vscode/maps/zhiyuan_rev.pcd",
        ),
        DeclareLaunchArgument(
            "octomap_output_bt",
            default_value="/tmp/nav3d_global_planner_zhiyuan_rev.bt",
        ),
        Node(
            package="global_planner",
            executable="global_planner_node",
            name="global_planner_node",
            output="screen",
            parameters=[{
                "pcd_map_file": pcd_map_file,
                "octomap_output_bt": octomap_output_bt,
            }],
        ),
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[
                planner_yaml,
                {
                    "use_sim_time": False,
                    "fsm.navi_mode": 3,
                    "grid_map.sensor_type": "lidar",
                    "grid_map.cloud_is_world": True,
                    "grid_map.need_extrinsic": False,
                    "grid_map.frame_id": "map",
                    "grid_map.sliding_map_frame_id": "sliding_map",
                    # global_planner Path.z is already robot reference height.
                    # Keep SCAN's Mode 3 z semantics aligned by disabling the
                    # incoming reference-path height offset at the boundary.
                    "grid_map.body_height": 0.0,
                },
            ],
            remappings=[
                ("body_pose", "/lio/localization/odom"),
                ("sensor_pose", "/lio/localization/odom"),
                ("cloud", "/global_to_scan/cloud"),
                ("initial_path", "/global_planner/path"),
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
                ("body_pose", "/lio/localization/odom"),
                ("cmd_vel", "/global_to_scan/cmd_vel"),
            ],
        ),
        mock_io,
        RegisterEventHandler(
            OnProcessExit(
                target_action=mock_io,
                on_exit=[Shutdown(reason="global_planner to SCAN validation finished")],
            )
        ),
    ])

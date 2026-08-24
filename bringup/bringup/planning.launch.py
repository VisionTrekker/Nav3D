"""Formal Nav3D planning subsystem launch."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    bringup_share = get_package_share_directory("bringup")
    scan_share = get_package_share_directory("scan_planner")
    config_path = LaunchConfiguration("config").perform(context)
    if not config_path:
        config_path = os.path.join(bringup_share, "config", "planning.yaml")
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["planning"]

    topics = config["topics"]
    global_cfg = config["global"]
    scan_cfg = config["scan"]
    safety_cfg = config["safety"]
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")

    scan_overrides = {
        "use_sim_time": False,
        "fsm.navi_mode": int(scan_cfg["navi_mode"]),
        "grid_map.sensor_type": scan_cfg["sensor_type"],
        "grid_map.cloud_is_world": bool(scan_cfg["cloud_is_world"]),
        "grid_map.need_extrinsic": bool(scan_cfg["need_extrinsic"]),
        "grid_map.frame_id": scan_cfg["frame_id"],
        "grid_map.sliding_map_frame_id": "sliding_map",
        "grid_map.body_height": float(scan_cfg["body_height"]),
    }

    return [
        Node(
            package="global_planner",
            executable="global_planner_node",
            name="global_planner_node",
            output="screen",
            parameters=[{
                "pcd_map_file": global_cfg["pcd_map_file"],
                "octomap_output_bt": global_cfg["octomap_output_bt"],
                "max_endpoint_snap_distance": float(global_cfg["max_endpoint_snap_distance"]),
            }],
        ),
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml, scan_overrides],
            remappings=[
                ("body_pose", topics["odom"]),
                ("sensor_pose", topics["sensor_pose"]),
                ("cloud", topics["cloud"]),
                ("initial_path", topics["global_path"]),
            ],
        ),
        Node(
            package="scan_planner",
            executable="closed_loop_controller",
            name="closed_loop_controller",
            output="screen",
            parameters=[controllers_yaml, {"use_sim_time": False}],
            remappings=[
                ("planning/bspline", topics["bspline"]),
                ("planning/stop_requested", topics["stop_requested"]),
                ("body_pose", topics["odom"]),
                ("cmd_vel", topics["cmd_vel"]),
            ],
        ),
        Node(
            package="bringup",
            executable="planning_safety_supervisor.py",
            name="planning_safety_supervisor",
            output="screen",
            parameters=[{"path_timeout_sec": float(safety_cfg["path_timeout_sec"])}],
            remappings=[
                ("goal_pose", topics["goal_pose"]),
                ("global_planner/path", topics["global_path"]),
                ("planning/stop_requested", topics["stop_requested"]),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value="",
            description="Planning subsystem YAML config. Defaults to bringup/config/planning.yaml",
        ),
        OpaqueFunction(function=_setup),
    ])

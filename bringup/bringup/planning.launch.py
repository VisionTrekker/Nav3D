"""Formal Nav3D planning subsystem launch."""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_LOGGER = launch_logging.get_logger("nav3d.planning")
_RESOLUTION_TOLERANCE = 1.0e-6


def _check_geometry_parameters(global_cfg, bringup_share, scan_share):
    """Report map-resolution mismatches without changing launch behavior."""
    if "octomap_resolution" not in global_cfg:
        raise ValueError("planning.global.octomap_resolution is required")

    octomap_resolution = float(global_cfg["octomap_resolution"])
    if not math.isfinite(octomap_resolution) or octomap_resolution <= 0.0:
        raise ValueError("planning.global.octomap_resolution must be positive")

    icp_yaml_path = os.path.join(bringup_share, "config", "localization_icp.yaml")
    icp_voxel_size = None
    try:
        with open(icp_yaml_path, "r", encoding="utf-8") as stream:
            icp_config = yaml.safe_load(stream) or {}
        icp_voxel_size = float(
            icp_config["fixed_map_icp"]["ros__parameters"]["voxel_size"])
        if not math.isfinite(icp_voxel_size) or icp_voxel_size <= 0.0:
            raise ValueError("fixed_map_icp.voxel_size must be positive")
    except (KeyError, TypeError, ValueError, OSError, yaml.YAMLError) as exc:
        _LOGGER.warning(f"Could not validate fixed-map ICP voxel_size: {exc}")

    scan_yaml_path = os.path.join(scan_share, "config", "planner.yaml")
    scan_resolution = None
    try:
        with open(scan_yaml_path, "r", encoding="utf-8") as stream:
            scan_config = yaml.safe_load(stream) or {}
        scan_resolution = float(
            scan_config["scan_planner_node"]["ros__parameters"]["grid_map.resolution"])
        if not math.isfinite(scan_resolution) or scan_resolution <= 0.0:
            raise ValueError("grid_map.resolution must be positive")
    except (KeyError, TypeError, ValueError, OSError, yaml.YAMLError) as exc:
        _LOGGER.warning(f"Could not validate SCAN grid_map.resolution: {exc}")

    icp_label = f"{icp_voxel_size:.3f} m" if icp_voxel_size is not None else "unknown"
    scan_label = f"{scan_resolution:.3f} m" if scan_resolution is not None else "unknown"
    _LOGGER.info(
        "Map geometry parameters: "
        f"map_loader/global OctoMap={octomap_resolution:.3f} m, "
        f"fixed_map_icp voxel_size={icp_label}, SCAN local grid={scan_label}")

    if icp_voxel_size is not None:
        if math.isclose(
            octomap_resolution,
            icp_voxel_size,
            rel_tol=0.0,
            abs_tol=_RESOLUTION_TOLERANCE,
        ):
            _LOGGER.info(
                "Map geometry consistency PASS: Global OctoMap and fixed-map ICP "
                "use the same resolution")
        else:
            _LOGGER.warning(
                "Map geometry consistency WARNING: Global/MapLoader OctoMap "
                f"resolution={octomap_resolution:.3f} m differs from fixed-map ICP "
                f"voxel_size={icp_voxel_size:.3f} m")

    if scan_resolution is not None:
        _LOGGER.info(
            "SCAN grid resolution is checked independently: "
            f"{scan_resolution:.3f} m; it is intentionally allowed to be "
            "finer than the global map")

    return octomap_resolution


def _setup(context):
    bringup_share = get_package_share_directory("bringup")
    scan_share = get_package_share_directory("scan_planner")
    config_path = LaunchConfiguration("config").perform(context)
    pcd_map_override = LaunchConfiguration("pcd_map_file").perform(context)
    if not config_path:
        config_path = os.path.join(bringup_share, "config", "planning.yaml")
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["planning"]

    topics = config["topics"]
    global_cfg = config["global"]
    scan_cfg = config["scan"]
    safety_cfg = config["safety"]
    use_cmd_odom_feedback = bool(scan_cfg.get("use_cmd_odom_feedback", False))
    use_simulated_sensing = bool(scan_cfg.get("use_simulated_sensing", False))
    local_odom_topic = topics.get("local_odom", topics["odom"])
    pcd_map_file = pcd_map_override or global_cfg["pcd_map_file"]
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    expected_octomap_resolution = _check_geometry_parameters(
        global_cfg, bringup_share, scan_share)
    global_planner_param_keys = [
        "robot_radius",
        "max_iterations",
        "snap_search_radius_cells",
        "require_ground_support",
        "strict_direct_ground_support",
        "ground_support_xy_radius_cells",
        "ground_support_depth_cells",
        "enable_preblocked_costmap",
        "preblocked_costmap_radius_cells",
        "preblocked_costmap_weight",
        "lowest_traversable_only",
    ]
    global_planner_overrides = {
        key: global_cfg[key]
        for key in global_planner_param_keys
        if key in global_cfg
    }
    map_loader_param_keys = [
        "voxel_downsample_m",
        "min_points_per_voxel",
        "min_cluster_voxels",
    ]
    map_loader_overrides = {
        key: global_cfg[key]
        for key in map_loader_param_keys
        if key in global_cfg
    }

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

    planning_odom_topic = local_odom_topic if use_cmd_odom_feedback else topics["odom"]

    actions = [
        Node(
            package="map_loader",
            executable="map_loader_node",
            name="map_loader_node",
            output="screen",
            parameters=[{
                "pcd_path": pcd_map_file,
                "resolution": float(global_cfg["octomap_resolution"]),
            }, map_loader_overrides],
        ),
        Node(
            package="global_planner",
            executable="global_planner_node",
            name="global_planner_node",
            output="screen",
            parameters=[{
                # Keep direct PCD loading available to the standalone global
                # planner, while the formal Nav3D chain consumes Map Loader's
                # transient-local OctoMap boundary.
                "pcd_map_file": pcd_map_file,
                "octomap_output_bt": global_cfg["octomap_output_bt"],
                "max_endpoint_snap_distance": float(global_cfg["max_endpoint_snap_distance"]),
                "expected_octomap_resolution": expected_octomap_resolution,
                "use_octomap_topic": True,
                "octomap_topic": topics["octomap"],
                "odom_topic": planning_odom_topic,
                "require_map_frame_odom": planning_odom_topic == "/lio/localization/odom" or
                use_cmd_odom_feedback,
                "goal_topic": topics["goal_pose"],
                "path_topic": topics["global_path"],
            }, global_planner_overrides],
        ),
    ]

    if use_cmd_odom_feedback and use_simulated_sensing:
        simulated_sensor_frame = scan_cfg.get("simulated_sensor_frame_id", "lidar_frame")
        actions.extend([
            Node(
                package="local_sensing_node",
                executable="pcl_render_node",
                name="closed_loop_lidar_simulator",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "sensor_type": "lidar",
                    "body_pose_topic": local_odom_topic,
                    "pcd_map_file": pcd_map_file,
                    "use_global_map_topic": False,
                    "world_frame_id": scan_cfg["frame_id"],
                    "sensor_frame_id": simulated_sensor_frame,
                    "publish_tf": False,
                    "sensing_rate": float(scan_cfg.get("simulated_sensor_rate", 10.0)),
                    "downsample_res": float(
                        scan_cfg.get("simulated_sensor_downsample_m", 0.15)),
                }],
                remappings=[
                    ("body_pose", local_odom_topic),
                    ("cloud", topics["cloud"]),
                    ("sensor_cloud", topics["cloud"] + "/sensor"),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="sim_base_lidar_publisher",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                    "--frame-id", "base_link", "--child-frame-id", simulated_sensor_frame,
                ],
            ),
        ])

    actions.extend([
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml, scan_overrides],
            remappings=[
                ("body_pose", local_odom_topic),
                ("sensor_pose", local_odom_topic if use_cmd_odom_feedback else topics["sensor_pose"]),
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
                ("body_pose", local_odom_topic),
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
    ])

    if use_cmd_odom_feedback:
        actions.append(
            Node(
                package="scan_planner",
                executable="go2_kinematic_sim",
                name="go2_kinematic_sim",
                output="screen",
                parameters=[
                    controllers_yaml,
                    {
                        "use_sim_time": False,
                        "init_odom_topic": "",
                        "initial_pose_topic": "/initialpose",
                        "require_initial_pose": True,
                        "publish_tf": True,
                        "frame_id": scan_cfg["frame_id"],
                        "child_frame_id": "base_link",
                    },
                ],
                remappings=[
                    ("body_pose", local_odom_topic),
                    ("cmd_vel", topics["cmd_vel"]),
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value="",
            description="Planning subsystem YAML config. Defaults to bringup/config/planning.yaml",
        ),
        DeclareLaunchArgument(
            "pcd_map_file",
            default_value="",
            description="Optional PCD override shared by Map Loader and Global Planner",
        ),
        OpaqueFunction(function=_setup),
    ])

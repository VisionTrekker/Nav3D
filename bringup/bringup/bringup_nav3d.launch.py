from launch import LaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.conditions import IfCondition
from bringup.launch_helpers import lio_mapping_remaps, lio_localization_remaps, driver_remaps


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    scene = LaunchConfiguration('scene')
    bag_path = LaunchConfiguration('bag_path')
    use_loc = LaunchConfiguration('use_localization')
    gps_factor_enabled = LaunchConfiguration('gps_factor_enabled')
    map_arg = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='bag',
                             description='Sensor mode: bag | real | sim'),
        DeclareLaunchArgument('scene', default_value='campus3',
                             description='Scene name'),
        DeclareLaunchArgument('bag_path',
                             default_value='/media/lenovo/disk/planner_ws/data-rosbag2/Campus3',
                             description='Path to rosbag2 directory'),
        DeclareLaunchArgument('use_localization', default_value='false',
                             description='Run lio_localization (true) or lio_mapping (false)'),
        DeclareLaunchArgument('gps_factor_enabled', default_value='false',
                             description='Enable GPS factor'),
        DeclareLaunchArgument('map',
                             default_value='/media/lenovo/disk/planner_ws/maps/campus3.pcd',
                             description='Path to PCD map file'),

        # Rosbag replay when mode==bag
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--loop'],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'bag'"])),
        ),

        # Static transform: map -> odom (identity placeholder, lio publishes odom->base)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', '0', 'map', 'odom'],
        ),

        # kiss_icp_node — always active; fine-corrects lio_localization poses
        Node(
            package='kiss_icp_bridge',
            executable='kiss_icp_node',
            name='kiss_icp_node',
            output='screen',
        ),

        # unitree_go2w driver (onboard-OrinNX ONLY; package absent from dev tree,
        # resolves on the real machine). Runs only in mode:=real.
        Node(
            package='unitree_go2w',
            executable='driver',
            name='unitree_driver',
            output='screen',
            condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"])),
            remappings=driver_remaps(),
        ),

        # LIO mapping (use_localization == false)
        Node(
            package='lio_backup',
            executable='lio',
            name='lio_mapping',
            output='screen',
            parameters=[{
                'config_path': 'root_config_bag.yaml',
                'frame_ids_path': 'frame_ids.yaml',
            }],
            remappings=lio_mapping_remaps(),
            condition=IfCondition(PythonExpression(["'", use_loc, "' == 'false'"])),
        ),

        # LIO localization (use_localization == true)
        Node(
            package='lio_backup',
            executable='lio',
            name='lio_localization',
            output='screen',
            parameters=[{
                'config_path': 'root_config_localization.yaml',
                'frame_ids_path': 'frame_ids.yaml',
            }],
            remappings=lio_localization_remaps(),
            condition=IfCondition(PythonExpression(["'", use_loc, "' == 'true'"])),
        ),

        # Scan context loop closure
        Node(
            package='scan_context_loop',
            executable='scan_context_loop_node',
            name='scan_context_loop_node',
            output='screen',
        ),

        # Map loader
        Node(
            package='map_loader',
            executable='map_loader_node',
            name='map_loader_node',
            output='screen',
            parameters=[{'pcd_path': map_arg}],
        ),

        # Goal marker server
        Node(
            package='goal_marker_server',
            executable='goal_marker_node',
            name='goal_marker_node',
            output='screen',
        ),

        # Global planner
        Node(
            package='global_planner',
            executable='global_planner_node',
            name='global_planner_node',
            output='screen',
        ),

        # Local (scan) planner — package=scan_planner per Task 9 finding
        Node(
            package='scan_planner',
            executable='scan_planner_node',
            name='scan_planner_node',
            output='screen',
            remappings=[('/cmd_vel', '/local_planner/cmd_vel')],
        ),

        # Rviz2 after 2 s warm-up
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['rviz2', '-d', '/media/lenovo/disk/planner_ws/src/Nav3D/bringup/rviz/nav3d_bag.rviz'],
                    output='screen',
                ),
            ],
        ),
    ])

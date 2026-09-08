from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from bringup.launch_helpers import lio_mapping_remaps, lio_localization_remaps, driver_remaps


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    bag_path = LaunchConfiguration('bag_path')
    bag_loop = LaunchConfiguration('bag_loop')
    bag_start_delay = LaunchConfiguration('bag_start_delay')
    use_loc = LaunchConfiguration('use_localization')
    auto_initialpose = LaunchConfiguration('auto_initialpose')
    initialpose = LaunchConfiguration('initialpose')
    enable_goal_marker = LaunchConfiguration('enable_goal_marker')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_scan_context = LaunchConfiguration('enable_scan_context')
    scan_context_cloud_topic = LaunchConfiguration('scan_context_cloud_topic')
    scan_context_odom_topic = LaunchConfiguration('scan_context_odom_topic')
    scan_context_output_topic = LaunchConfiguration('scan_context_output_topic')
    map_arg = LaunchConfiguration('map')
    lio_mapping_config = PythonExpression([
        "'root_config_real.yaml' if '", mode,
        "' == 'real' else 'root_config_bag.yaml'"
    ])
    lio_localization_config = PythonExpression([
        "'root_config_real.yaml' if '", mode,
        "' == 'real' else 'root_config_localization.yaml'"
    ])
    localization_icp_yaml = PathJoinSubstitution([
        FindPackageShare('bringup'), 'config', 'localization_icp.yaml'
    ])
    planning_launch = PathJoinSubstitution([
        FindPackageShare('bringup'), 'launch', 'planning.launch.py'
    ])
    planning_mapping_yaml = PathJoinSubstitution([
        FindPackageShare('bringup'), 'config', 'planning_mapping.yaml'
    ])
    planning_localization_yaml = PathJoinSubstitution([
        FindPackageShare('bringup'), 'config', 'planning.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('bringup'), 'rviz', 'nav3d_bag.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='bag',
                             description='Sensor mode: bag | real | sim'),
        DeclareLaunchArgument('bag_path',
                             default_value='/media/lenovo/disk/planner_ws/data-rosbag2/Campus3',
                             description='Path to rosbag2 directory'),
        DeclareLaunchArgument(
            'bag_loop', default_value='true',
            description='Loop rosbag playback when mode:=bag'),
        DeclareLaunchArgument(
            'bag_start_delay', default_value='0.0',
            description='Seconds to wait before rosbag playback so sensor subscribers can connect'),
        DeclareLaunchArgument('use_localization', default_value='false',
                             description='Run lio_localization (true) or lio_mapping (false)'),
        DeclareLaunchArgument(
            'auto_initialpose', default_value='false',
            description='Publish configured map-frame initialpose once in localization mode'),
        DeclareLaunchArgument(
            'initialpose', default_value='',
            description='Map-frame initial pose as x,y,z,yaw; used only with auto_initialpose=true'),
        DeclareLaunchArgument(
            'enable_goal_marker', default_value='false',
            description='Start the optional interactive goal marker server'),
        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='Start RViz after the navigation graph is ready'),
        DeclareLaunchArgument(
            'enable_scan_context', default_value='false',
            description='Start the online Scan Context diagnostic node'),
        DeclareLaunchArgument(
            'scan_context_cloud_topic', default_value='/lio/mapping/clouds_lidar',
            description='Scan Context local cloud topic'),
        DeclareLaunchArgument(
            'scan_context_odom_topic', default_value='/lio/mapping/odom_body',
            description='Scan Context raw odometry topic'),
        DeclareLaunchArgument(
            'scan_context_output_topic', default_value='/scan_context_loop/loop_closure',
            description='Scan Context online loop-candidate topic'),
        DeclareLaunchArgument('map',
                             default_value='/home/nhy/code/vscode/Nav3D/maps/campus3_no_elevator.pcd',
                             description='Path to PCD map file'),

        # Rosbag replay when mode==bag
        TimerAction(
            period=bag_start_delay,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', bag_path, '--loop'],
                    condition=IfCondition(PythonExpression([
                        "'", mode, "' == 'bag' and '", bag_loop, "' == 'true'"])),
                ),
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', bag_path],
                    condition=IfCondition(PythonExpression([
                        "'", mode, "' == 'bag' and '", bag_loop, "' == 'false'"])),
                ),
            ],
        ),

        # Static transform: map -> odom (identity placeholder, lio publishes odom->base)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            condition=IfCondition(PythonExpression([
                "'", use_loc, "' == 'false' and '", mode, "' != 'sim'"
            ])),
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
            package='lio',
            executable='lio',
            name='lio_mapping',
            output='screen',
            parameters=[{'config_path': lio_mapping_config}],
            remappings=lio_mapping_remaps(),
            condition=IfCondition(PythonExpression([
                "'", use_loc, "' == 'false' and '", mode, "' != 'sim'"
            ])),
        ),

        # LIO localization (use_localization == true)
        Node(
            package='lio',
            executable='lio',
            name='lio_localization',
            output='screen',
            parameters=[{'config_path': lio_localization_config}],
            remappings=lio_localization_remaps(),
            condition=IfCondition(PythonExpression([
                "'", use_loc, "' == 'true' and '", mode, "' != 'sim'"
            ])),
        ),

        # Localization boundary: owns the dynamic map -> odom transform and
        # publishes the composed map -> base_link odometry.  It is mutually
        # exclusive with the mapping-mode identity map -> odom publisher above.
        Node(
            package='lio_localization',
            executable='localization_composer',
            name='localization_composer',
            output='screen',
            parameters=[localization_icp_yaml],
            condition=IfCondition(PythonExpression([
                "'", use_loc, "' == 'true' and '", mode, "' != 'sim'"
            ])),
        ),

        # Optional deterministic initialization for bag/field bringup.  The
        # pose is supplied in the fixed PCD map frame; raw odometry remains
        # strictly an odom-frame input.
        Node(
            package='lio_localization',
            executable='auto_initialpose_once',
            name='auto_initialpose_once',
            output='screen',
            parameters=[{'map_pose': initialpose}],
            condition=IfCondition(PythonExpression([
                "'", use_loc, "' == 'true' and '", mode, "' != 'sim' and '",
                auto_initialpose, "' == 'true'"
            ])),
        ),

        # Fixed-PCD refinement publishes pose measurements only.  The composer
        # above remains the sole localization owner of dynamic map -> odom.
        Node(
            package='lio_localization',
            executable='fixed_map_icp',
            name='fixed_map_icp',
            output='screen',
            parameters=[localization_icp_yaml, {'map_path': map_arg}],
            condition=IfCondition(PythonExpression([
                "'", use_loc, "' == 'true' and '", mode, "' != 'sim'"
            ])),
        ),

        # Scan context loop closure
        Node(
            package='scan_context_loop',
            executable='scan_context_loop_node',
            name='scan_context_loop_node',
            output='screen',
            parameters=[{
                'cloud_topic': scan_context_cloud_topic,
                'odom_topic': scan_context_odom_topic,
                'output_topic': scan_context_output_topic,
            }],
            condition=IfCondition(enable_scan_context),
        ),

        # Goal marker server
        Node(
            package='goal_marker_server',
            executable='goal_marker_node',
            name='goal_marker_node',
            output='screen',
            condition=IfCondition(enable_goal_marker),
        ),

        # Formal planning subsystem in mapping integration mode.  map -> odom
        # is identity, so the mapping body odometry can be transformed into map
        # without relabeling its message frame.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_launch),
            launch_arguments={
                'config': planning_mapping_yaml,
                'pcd_map_file': map_arg,
            }.items(),
            condition=IfCondition(PythonExpression([
                "'", use_loc, "' == 'false' and '", mode, "' != 'sim'"
            ])),
        ),

        # Formal navigation mode consumes the composer's true map -> base_link
        # odometry and starts the same planner/controller chain exactly once.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_launch),
            launch_arguments={
                'config': planning_localization_yaml,
                'pcd_map_file': map_arg,
            }.items(),
            condition=IfCondition(PythonExpression([
                "'", use_loc, "' == 'true' or '", mode, "' == 'sim'"
            ])),
        ),

        # Rviz2 after 2 s warm-up
        TimerAction(
            period=2.0,
            condition=IfCondition(enable_rviz),
            actions=[
                ExecuteProcess(
                    cmd=['rviz2', '-d', rviz_config],
                    output='screen',
                ),
            ],
        ),
    ])

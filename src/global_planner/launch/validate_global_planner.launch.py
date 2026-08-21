from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pcd_map_file = LaunchConfiguration("pcd_map_file")
    octomap_output_bt = LaunchConfiguration("octomap_output_bt")

    return LaunchDescription([
        DeclareLaunchArgument(
            "pcd_map_file",
            default_value="/home/nhy/code/vscode/maps/zhiyuan_rev.pcd",
            description="Existing project PCD map for Stage 2 global_planner validation",
        ),
        DeclareLaunchArgument(
            "octomap_output_bt",
            default_value="/tmp/nav3d_global_planner_zhiyuan_rev.bt",
            description="Temporary OctoMap .bt output generated from the PCD map",
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
    ])

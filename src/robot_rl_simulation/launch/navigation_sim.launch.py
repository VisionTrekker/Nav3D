from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo


NOT_READY_MESSAGE = (
    "navigation_sim resources are installed, but rl_locomotion_a1 and the Gazebo "
    "spawn/control launch graph have not been implemented yet."
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=""),
        DeclareLaunchArgument("start_pose", default_value="0,0,0.35,0"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("goal_pose", default_value=""),
        DeclareLaunchArgument("gui", default_value="true"),
        LogInfo(msg=NOT_READY_MESSAGE),
    ])

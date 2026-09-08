from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cloud_topic = LaunchConfiguration('cloud_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    output_topic = LaunchConfiguration('output_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/lio/mapping/clouds_lidar',
            description='Local point cloud consumed by Scan Context'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/lio/mapping/odom_body',
            description='Raw odometry associated with the local scan'),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/scan_context_loop/loop_closure',
            description='Diagnostic online loop-candidate output'),
        Node(
            package='scan_context_loop',
            executable='scan_context_loop_node',
            name='scan_context_loop_node',
            output='screen',
            parameters=[{
                'cloud_topic': cloud_topic,
                'odom_topic': odom_topic,
                'output_topic': output_topic,
            }],
        ),
    ])

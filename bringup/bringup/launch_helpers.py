from launch_ros.actions import Node


def lio_mapping_remaps():
    return [
        ('/LIO/odom_imu', '/lio/mapping/odom_imu'),
        ('/LIO/odom_vehicle', '/lio/mapping/odom_body'),
        ('/LIO/clouds_lidar', '/lio/mapping/clouds_lidar'),
        ('/LIO/global_map', '/lio/mapping/global_map'),
        ('/LIO/ikdtree', '/lio/mapping/ikdtree'),
        ('/LIO/in_elevator', '/lio/mapping/in_elevator'),
        ('/LIO/set_elevator_flag', '/lio/mapping/set_elevator_flag'),
    ]


def lio_localization_remaps():
    return [
        ('/LIO/odom_imu', '/lio/localization/odom_imu'),
        ('/LIO/odom_vehicle', '/lio/localization/odom_body'),
        ('/lio/localization/odom_body', '/lio/localization/odom'),
    ]


def driver_remaps():
    return [('/local_planner/cmd_vel', '/cmd_vel')]

from launch_ros.actions import Node


def lio_mapping_remaps():
    return [
        ('/LIO/odom_imu', '/lio/mapping/odom_imu'),
        ('/LIO/odom_vehicle', '/lio/mapping/odom_body'),
        ('/LIO/clouds_lidar', '/lio/mapping/clouds_lidar'),
        ('/LIO/clouds_lidar/effect', '/lio/mapping/clouds_lidar/effect'),
        ('/LIO/clouds_lidar/reject', '/lio/mapping/clouds_lidar/reject'),
        ('/LIO/global_map', '/lio/mapping/global_map'),
        ('/LIO/ikdtree', '/lio/mapping/ikdtree'),
        ('/LIO/in_elevator', '/lio/mapping/in_elevator'),
        ('/LIO/elevator_state', '/lio/mapping/elevator_state'),
        ('/LIO/set_elevator_flag', '/lio/mapping/set_elevator_flag'),
    ]


def lio_localization_remaps():
    # Elevator-LIO remains a local odometry producer in navigation mode.  Keep
    # every raw output under the frozen mapping namespace; only the composer
    # may publish /lio/localization/odom.
    return list(lio_mapping_remaps())


def driver_remaps():
    return [('/local_planner/cmd_vel', '/cmd_vel')]

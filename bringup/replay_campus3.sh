#!/usr/bin/env bash
# Replay Elevator-LIO open-source Campus3 dataset with offline LIO.
#
# Usage:
#   bash bringup/replay_campus3.sh
#
# Default bag: /media/lenovo/disk/planner_ws/data-rosbag2/Campus3
# Default config: root_config_campus3.yaml (sensors/mid360_campus3.yaml +
#                 runtime/mapping_campus3.yaml)
#
# What it does:
#   1. Publishes static TF vehicle -> livox_frame using dataset calibration
#      (lidar_t_body=[-0.2, 0, -0.15], lidar_R_body=R_z(90°)).
#   2. Replays the bag in loop mode (50 Hz lidar + 200 Hz IMU).
#   3. Launches Elevator-LIO with root_config_campus3.yaml.
#
# Notes:
#   - The bag contains /LIO/set_elevator_flag (1 msg) and includes elevator
#     segments, so elevator.enable=true in mapping_campus3.yaml.
#   - ROS_DOMAIN_ID must match your environment (this workspace uses 18).
#   - Static TF child_frame_id is "livox_frame" (matches mid360_campus3.yaml).
set -e

NAV3D_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_PATH="${1:-/media/lenovo/disk/planner_ws/data-rosbag2/Campus3}"
ROOT_CONFIG="${2:-root_config_campus3.yaml}"

# --- Clean env (avoid stale ROS workspace pollution) ---
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH 2>/dev/null || true
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-18}

# --- Source ROS + Nav3D ---
source /opt/ros/humble/setup.bash
source "$NAV3D_ROOT/install/setup.bash"

if [ ! -d "$BAG_PATH" ]; then
  echo "ERROR: bag not found at $BAG_PATH" >&2
  exit 1
fi

echo "[1/3] Publishing static TF vehicle -> livox_frame"
echo "      lidar_t_body=[-0.2, 0, -0.15]  lidar_R_body=R_z(90°)"
# For a static TF "vehicle -> livox_frame" we need the inverse of (lidar -> vehicle).
# calibration_offsets gives lidar_t_vehicle / lidar_R_vehicle (position of vehicle in lidar frame,
# rotation of vehicle in lidar frame). The inverse transform for the publisher is computed by
# main_loop below; we precompute it here for the static publisher.
# lidar_R_vehicle = R_z(90°), so vehicle_R_lidar = R_z(-90°) = [[0,1,0],[-1,0,0],[0,0,1]]
# vehicle_t_lidar = -vehicle_R_lidar * lidar_t_vehicle
#                  = -[[0,1,0],[-1,0,0],[0,0,1]] * [-0.2, 0, -0.15]
#                  = -[0*(-0.2)+1*0+0*(-0.15), -1*(-0.2)+0*0+0*(-0.15), 0+0+1*(-0.15)]
#                  = -[0, 0.2, -0.15]
#                  = [0, -0.2, 0.15]
# Express the rotation as a quaternion via roll/pitch/yaw:
# R_z(-90°)  -> roll=0, pitch=0, yaw=-1.5708 rad
ros2 run tf2_ros static_transform_publisher \
  --frame-id vehicle --child-frame-id livox_frame \
  --x 0.0 --y -0.2 --z 0.15 \
  --roll 0.0 --pitch 0.0 --yaw -1.5708 &
TF_PID=$!
sleep 1

echo "[2/3] Replaying bag (loop): $BAG_PATH"
ros2 bag play "$BAG_PATH" --loop &
BAG_PID=$!
sleep 3

echo "[3/3] Launching Elevator-LIO (config_path:=$ROOT_CONFIG)"
ros2 launch lio start_ros2.launch.py \
  config_path:="$ROOT_CONFIG" use_rviz:=true &
LIO_PID=$!

echo
echo "Replaying + LIO running."
echo "  Fixed Frame in RViz: world"
echo "  Topics: /LIO/{odom_imu, odom_vehicle, clouds_lidar, global_map, ikdtree, in_elevator}"
echo "  Static TF: vehicle -> livox_frame (R_z(-90°))"
echo "  Note: elevator.enable=true. Trigger by publishing to /LIO/set_elevator_flag."
echo "Ctrl-C to stop all."

cleanup() {
  echo "Stopping..."
  [ -n "$LIO_PID" ] && kill "$LIO_PID" 2>/dev/null || true
  [ -n "$BAG_PID" ] && kill "$BAG_PID" 2>/dev/null || true
  [ -n "$TF_PID" ] && kill "$TF_PID" 2>/dev/null || true
}
trap cleanup INT TERM
wait
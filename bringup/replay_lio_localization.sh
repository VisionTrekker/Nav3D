#!/usr/bin/env bash
# Replay Campus3 bag with lio_localization for fine localization.
#
# Usage:
#   bash bringup/replay_lio_localization.sh                      # uses default bag
#   bash bringup/replay_lio_localization.sh <bag_path>          # custom bag
#
# Default bag: /media/lenovo/disk/planner_ws/data-rosbag2/Campus3
#
# What it does:
#   1. Plays the bag with last 30s trimmed (elevator segment at end).
#   2. Launches Elevator-LIO with root_config_localization.yaml
#      (sensors/mid360_bag.yaml + runtime/relocation_localization.yaml + logging/default.yaml).
#   3. Publishes static TF vehicle -> livox_frame (R_z(-90°) Campus3 calibration).
#
# Notes:
#   - ROS_DOMAIN_ID must match your environment (this workspace uses 18).
set -e

NAV3D_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-/media/lenovo/disk/planner_ws/data-rosbag2/Campus3}"

# --- Resolve bag duration and compute trim (skip last 30s = elevator segment) ---
DURATION=$(ros2 bag info "$BAG" 2>/dev/null | awk '/Duration:/ {print $2}')
if [ -z "$DURATION" ]; then
  echo "ERROR: could not read bag duration for $BAG" >&2
  exit 1
fi
TRIMMED=$(python3 -c "print(max(0.0, float('$DURATION') - 30.0))")
echo "[bag] duration=${DURATION}s  trimmed=${TRIMMED}s  (skip last 30s elevator)"

# --- Clean env (avoid stale ROS workspace pollution) ---
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH 2>/dev/null || true
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-18}

# --- Source ROS + Nav3D ---
source /opt/ros/humble/setup.bash
source "$NAV3D_ROOT/install/setup.bash"

if [ ! -d "$BAG" ]; then
  echo "ERROR: bag not found at $BAG" >&2
  exit 1
fi

# --- Resolve PACKAGE_ROOT_DIR (lio install/share/lio) ---
LIO_PKG_DIR="$(ros2 pkg prefix lio)/share/lio"
if [ -z "$LIO_PKG_DIR" ] || [ ! -d "$LIO_PKG_DIR" ]; then
  echo "ERROR: lio package not found in ROS environment. Did colcon build succeed?" >&2
  exit 1
fi

# --- Static TF: vehicle -> livox_frame (Campus3 calibration: R_z(-90°)) ---
# lidar_R_vehicle = R_z(90°), so vehicle_R_lidar = R_z(-90°)
# vehicle_t_lidar = -vehicle_R_lidar * lidar_t_vehicle
#                 = -[[0,1,0],[-1,0,0],[0,0,1]] * [-0.2, 0, -0.15]
#                 = [0, -0.2, 0.15]
echo "[1/3] Publishing static TF vehicle -> livox_frame  (R_z(-90°))"
ros2 run tf2_ros static_transform_publisher \
  --frame-id vehicle --child-frame-id livox_frame \
  --x 0.0 --y -0.2 --z 0.15 \
  --roll 0.0 --pitch 0.0 --yaw -1.5708 &
TF_PID=$!
sleep 1

# --- Play bag (background, looped so data keeps flowing) ---
echo "[2/3] Replaying bag (loop): $BAG  (--duration $TRIMMED)"
ros2 bag play "$BAG" --duration "$TRIMMED" --loop &
BAG_PID=$!
sleep 2

# --- Launch lio_localization ---
echo "[3/3] Launching Elevator-LIO localization (config_path:=root_config_localization.yaml use_rviz:=false)"
ros2 launch lio start_ros2.launch.py config_path:=root_config_localization.yaml use_rviz:=false &
LIO_PID=$!

echo
echo "lio_localization running in background (pid=$LIO_PID)."
echo "  Topics: /LIO/{odom_imu, odom_vehicle, clouds_lidar}"
echo "  Localization: /lio/localization/{odom,score,odom_imu,odom_body}"
echo "  Static TF: vehicle -> livox_frame (R_z(-90°))"
echo "  Elevator: DISABLED in relocation_localization.yaml"
echo "Ctrl-C to stop."

cleanup() {
  echo "Stopping..."
  [ -n "$LIO_PID" ] && kill "$LIO_PID" 2>/dev/null || true
  [ -n "$BAG_PID" ] && kill "$BAG_PID" 2>/dev/null || true
  [ -n "$TF_PID" ] && kill "$TF_PID" 2>/dev/null || true
}
trap cleanup INT TERM

# Wait for lio to exit
wait "$LIO_PID" || true
exit 0

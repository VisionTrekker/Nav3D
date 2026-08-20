#!/usr/bin/env bash
# Replay a recorded Go2W bag and run Elevator-LIO offline for testing.
#
# Usage:
#   bash bringup/replay_lio.sh                          # uses default bag path
#   bash bringup/replay_lio.sh <bag_path>               # custom bag
#
# Default bag: /media/lenovo/disk/planner_ws/lio_bag/go2w_lio_20260812_112429
#   (workspace-local path; $HOME/lio_bag would be wrong)
#
# What it does:
#   1. Publishes static TF base_link -> livox_frame using hand-measured extrinsics
#      (mid360: xyz=(0.18, 0, 0.13) relative to base_link, pitch=18 deg around y).
#   2. Replays the bag in loop mode (so /livox/lidar + /livox/imu keep flowing).
#   3. Launches Elevator-LIO with root_config_real.yaml (which selects
#      sensors/mid360_real.yaml + runtime/mapping_real.yaml + logging/default.yaml).
#
# Notes:
#   - Do NOT start livox_ros_driver2 while replaying (the bag already has data).
#   - ROS_DOMAIN_ID must match the machine that recorded the bag (this workspace
#     uses 18; export ROS_DOMAIN_ID=18 before running if needed).
#   - The real machine publishes no /tf or /tf_static, which is why this script
#     publishes the static TF for base_link -> livox_frame.
#   - Elevator-LIO's `publish.cpp` auto-disables /LIO/global_map and /LIO/ikdtree
#     after the first publish if cost > 10 ms. Use RViz PointCloud2 with
#     Decay Time = 999 on /LIO/clouds_lidar to accumulate a pseudo-map.
set -e

NAV3D_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_PATH="${1:-/media/lenovo/disk/planner_ws/lio_bag/go2w_lio_20260812_112429}"

# --- Clean env (avoid stale ROS workspace pollution) ---
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH 2>/dev/null || true
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-18}

# --- Source ROS + Nav3D (bash variants for BASH_SOURCE) ---
source /opt/ros/humble/setup.bash
source "$NAV3D_ROOT/install/setup.bash"

if [ ! -d "$BAG_PATH" ]; then
  echo "ERROR: bag not found at $BAG_PATH" >&2
  echo "  Usage: bash $0 <bag_path>" >&2
  exit 1
fi

echo "[1/3] Publishing static TF base_link -> livox_frame"
echo "      xyz=(0.18, 0, 0.13)  pitch=18 deg (R_y(0.314))"
ros2 run tf2_ros static_transform_publisher \
  --frame-id base_link --child-frame-id livox_frame \
  --x 0.18 --y 0.0 --z 0.13 \
  --roll 0.0 --pitch 0.314 --yaw 0.0 &
TF_PID=$!
sleep 1

echo "[2/3] Replaying bag (loop): $BAG_PATH"
ros2 bag play "$BAG_PATH" --loop &
BAG_PID=$!
sleep 3

echo "[3/3] Launching Elevator-LIO (config_path:=root_config_real.yaml)"
ros2 launch lio start_ros2.launch.py \
  config_path:=root_config_real.yaml use_rviz:=true &
LIO_PID=$!

echo
echo "Replaying + LIO running."
echo "  Fixed Frame in RViz: world"
echo "  Topics: /LIO/{odom_imu, odom_vehicle, clouds_lidar, in_elevator}"
echo "  TF: world -> IMU (tilted 18 deg), world -> body (level)"
echo "  Static TF: base_link -> livox_frame (pitch 18 deg)"
echo "Ctrl-C to stop all."

cleanup() {
  echo "Stopping..."
  [ -n "$LIO_PID" ] && kill "$LIO_PID" 2>/dev/null || true
  [ -n "$BAG_PID" ] && kill "$BAG_PID" 2>/dev/null || true
  [ -n "$TF_PID" ] && kill "$TF_PID" 2>/dev/null || true
}
trap cleanup INT TERM
wait
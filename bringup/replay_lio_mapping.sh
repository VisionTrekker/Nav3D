#!/usr/bin/env bash
# Replay Campus3 bag with lio_mapping for PCD map generation.
#
# Usage:
#   bash bringup/replay_lio_mapping.sh                      # uses default bag
#   bash bringup/replay_lio_mapping.sh <bag_path>           # custom bag
#
# Default bag: /media/lenovo/disk/planner_ws/data-rosbag2/Campus3
#
# What it does:
#   1. Plays the bag with last 30s trimmed (elevator segment at end).
#   2. Launches Elevator-LIO with root_config_bag.yaml
#      (sensors/mid360_bag.yaml + runtime/mapping_bag.yaml + logging/default.yaml).
#   3. Publishes static TF vehicle -> livox_frame (R_z(-90°) Campus3 calibration).
#   4. On shutdown (Ctrl-C), copies the saved PCD to /media/lenovo/disk/planner_ws/maps/campus3.pcd.
#
# Notes:
#   - lio saves global PCD on node shutdown (Ctrl-C / SIGINT) via shutdown_save_maps().
#   - PCD output (before copy): <PACKAGE_ROOT_DIR>/PCD/<timestamp>_scans.pcd
#     where PACKAGE_ROOT_DIR = lio install/share/lio directory.
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

# --- Create a start-time marker for finding the PCD saved during this run ---
MARKER_FILE="/tmp/lio_mapping_start_$$"
touch "$MARKER_FILE"

# --- Resolve PACKAGE_ROOT_DIR (lio install/share/lio) ---
LIO_PKG_DIR="$(ros2 pkg prefix lio)/share/lio"
if [ -z "$LIO_PKG_DIR" ] || [ ! -d "$LIO_PKG_DIR" ]; then
  echo "ERROR: lio package not found in ROS environment. Did colcon build succeed?" >&2
  exit 1
fi
PCD_DIR="$LIO_PKG_DIR/PCD"

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

# --- Launch lio_mapping ---
echo "[3/3] Launching Elevator-LIO (config_path:=root_config_bag.yaml use_rviz:=false)"
ros2 launch lio start_ros2.launch.py config_path:=root_config_bag.yaml use_rviz:=false &
LIO_PID=$!

echo
echo "lio_mapping running in background (pid=$LIO_PID)."
echo "  Topics: /LIO/{odom_imu, odom_vehicle, clouds_lidar, global_map}"
echo "  Static TF: vehicle -> livox_frame (R_z(-90°))"
echo "  Elevator: DISABLED in mapping_bag.yaml"
echo
echo "PCD is saved on node shutdown (Ctrl-C)."
echo "  Source: $PCD_DIR/<timestamp>_scans.pcd"
echo "  Dest:   /media/lenovo/disk/planner_ws/maps/campus3.pcd"
echo "Ctrl-C to stop and trigger save."

cleanup() {
  echo "Stopping..."
  [ -n "$LIO_PID" ] && kill "$LIO_PID" 2>/dev/null || true
  [ -n "$BAG_PID" ] && kill "$BAG_PID" 2>/dev/null || true
  [ -n "$TF_PID" ] && kill "$TF_PID" 2>/dev/null || true
}
trap cleanup INT TERM

# Wait for lio to exit (shutdown_save_maps runs on SIGINT)
wait "$LIO_PID" || true
LIO_EXIT_CODE=$?

# --- Find and copy the PCD saved during this run ---
echo
echo "[post-shutdown] Finding saved PCD in $PCD_DIR..."

mkdir -p /media/lenovo/disk/planner_ws/maps

# Find newest *_scans.pcd created after the marker file
SAVED_PCD=$(find "$PCD_DIR" -name '*_scans.pcd' -newer "$MARKER_FILE" -print -quit 2>/dev/null)
rm -f "$MARKER_FILE"

if [ -n "$SAVED_PCD" ] && [ -f "$SAVED_PCD" ]; then
  DEST="/media/lenovo/disk/planner_ws/maps/campus3.pcd"
  cp "$SAVED_PCD" "$DEST"
  SIZE=$(ls -lh "$DEST" | awk '{print $5}')
  echo "[post-shutdown] PCD copied: $DEST ($SIZE)"
else
  echo "[post-shutdown] WARNING: No *_scans.pcd found in $PCD_DIR after lio shutdown." >&2
  echo "[post-shutdown] lio may have exited without saving. Check lio log output above." >&2
  exit 1
fi

exit $LIO_EXIT_CODE

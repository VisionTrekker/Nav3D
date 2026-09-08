#!/usr/bin/env bash
# Nav3D 单窗口全自动联合启动与验收脚本
# 用法:
#   chmod +x nav3d_one_window_fixed.sh
#   ./nav3d_one_window_fixed.sh
#
# 当前默认：
#   - 自动编译
#   - 自动启动 bringup + rosbag + RViz
#   - 自动发布 map 坐标系 initialpose 和 goal_pose
#   - RViz 只观察地图、全局路径和局部轨迹
#   - 发布 Goal 前先建立 path、bspline、cmd_vel 监听，避免漏掉快速消息

# 不启用 set -u：ROS2 Humble 的 setup.bash 会读取部分尚未定义的环境变量，
# 使用 nounset 会导致 source /opt/ros/humble/setup.bash 直接退出。
set +u

# =========================
# 0. 用户配置
# =========================
WORKSPACE="/home/nhy/code/vscode/Nav3D"
BAG_PATH="$WORKSPACE/datasets/Elevator-LIO-Dataset-rosbag2/Campus3"
MAP_PATH="$WORKSPACE/maps/campus3_no_elevator.pcd"
CMD_VEL_TOPIC="/local_planner/cmd_vel"
LOCAL_ODOM_TOPIC="/local_planner/sim_odom"
# 1: fixed-map closed-loop simulation; 0: original bag + LIO localization mode.
# The closed-loop mode makes the simulated odom and simulated LiDAR the only
# pose/sensing source after /initialpose, so bag replay cannot move the robot.
CLOSED_LOOP_SIM=1
PLANNING_ODOM_TOPIC="$LOCAL_ODOM_TOPIC"

BUILD_ON_START=1
ENABLE_RVIZ=true
ENABLE_SCAN_CONTEXT=false
KEEP_RUNNING_AFTER_PASS=1
# 给 LIO 和自动 initialpose 订阅端留出 DDS 建联时间，避免错过 bag 首帧。
BAG_START_DELAY="5.0"

# initialpose:
#   rviz = 等待你在 RViz 使用 2D Pose Estimate
#   auto = 首个有效 raw odom 到达时自动绑定下面给出的 map 位姿
INIT_MODE="auto"
# 固定 PCD map 中已验证连通的同层起点。
INIT_X="-0.857"
INIT_Y="15.018"
INIT_Z="0.520"
INIT_YAW_RAD="-1.467"

# goal:
#   rviz = 等待你在 RViz 使用 Goal Tool（发布 /goal_pose）
#   auto = 脚本自动发布下面给出的 map 目标
#   off  = 只验收定位，不做规划
GOAL_MODE="auto"
# 固定 PCD map 中与起点处于同一连通域的前向验收点。
GOAL_X="-1.150"
GOAL_Y="27.000"
GOAL_Z="0.286"
GOAL_YAW_RAD="-1.468445495"

STARTUP_TIMEOUT=90
DATA_TIMEOUT=120
INITIALPOSE_TIMEOUT=300
LOCALIZATION_TIMEOUT=120
GOAL_TIMEOUT=300
PLAN_TIMEOUT=120
PLANAR_RPY_TOL_DEG="0.50"
TF_CHECK_TIMEOUT=5

# 自动验收在预录轨迹进入已验证的同层连通段后再发 Goal。
# 触发点由 INIT/GOAL 自动插值，不是第 9、10 个用户坐标参数。
AUTO_GOAL_TRIGGER_PROGRESS="0.50"
AUTO_GOAL_TRIGGER_RADIUS_XY="1.00"
AUTO_GOAL_TRIGGER_MAX_Z_ERROR="1.00"
AUTO_GOAL_SAME_FLOOR_MAX_PROGRESS="1.35"

RUN_DIR="/tmp/nav3d_one_window"
LOG_FILE="$RUN_DIR/bringup.log"
ERROR_MARKER="$RUN_DIR/upstream_failure.txt"
mkdir -p "$RUN_DIR"
rm -f "$RUN_DIR"/*.yaml "$RUN_DIR"/*.txt "$LOG_FILE"

LAUNCH_PID=""
LAUNCH_PGID=""
MAIN_PID="$$"
SCRIPT_PGID="$(ps -o pgid= -p "$$" | tr -d ' ')"
ERROR_MONITOR_PID=""
PATH_LISTENER_PID=""
BSPLINE_LISTENER_PID=""
CMD_VEL_LISTENER_PID=""
ACCEPTANCE_PASSED=0

# =========================
# 工具函数
# =========================
step() {
  echo
  echo "============================================================"
  echo "[$(date '+%H:%M:%S')] $*"
  echo "============================================================"
}

die() {
  echo
  echo "[失败] $*"
  echo
  if [[ -s "$ERROR_MARKER" ]]; then
    echo "---- 上游故障分类 ----"
    cat "$ERROR_MARKER"
    echo "----------------------"
  fi
  if [[ -f "$LOG_FILE" ]]; then
    echo "---- bringup 最后 50 行日志 ----"
    tail -n 50 "$LOG_FILE" || true
    echo "--------------------------------"
  fi
  exit 1
}

cleanup() {
  echo
  echo "[退出] 正在清理本脚本启动的 bringup..."
  if [[ -n "${ERROR_MONITOR_PID:-}" ]] && kill -0 "$ERROR_MONITOR_PID" 2>/dev/null; then
    kill "$ERROR_MONITOR_PID" 2>/dev/null || true
  fi
  wait "${ERROR_MONITOR_PID:-}" 2>/dev/null || true
  stop_topic_listener "${PATH_LISTENER_PID:-}"
  stop_topic_listener "${BSPLINE_LISTENER_PID:-}"
  stop_topic_listener "${CMD_VEL_LISTENER_PID:-}"

  if [[ -n "${LAUNCH_PGID:-}" && "${LAUNCH_PGID:-}" != "$SCRIPT_PGID" ]]; then
    kill -TERM -- "-$LAUNCH_PGID" 2>/dev/null || true
    sleep 2
    kill -KILL -- "-$LAUNCH_PGID" 2>/dev/null || true
  elif [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    sleep 2
  fi
}
trap cleanup EXIT

handle_signal() {
  if [[ "$ACCEPTANCE_PASSED" -eq 1 ]]; then
    echo "[退出] 联合验收已通过，正在正常结束观察并清理进程。"
    exit 0
  fi
  if [[ -s "$ERROR_MARKER" ]]; then
    cat "$ERROR_MARKER"
  else
    echo "[失败] 测试被中断，未发现已记录的上游错误。"
  fi
  exit 1
}
trap handle_signal INT TERM

wait_topic_exists() {
  local topic="$1"
  local timeout_s="$2"
  local start now
  start=$(date +%s)

  while true; do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      echo "[OK] topic 已出现: $topic"
      return 0
    fi

    now=$(date +%s)
    if (( now - start >= timeout_s )); then
      return 1
    fi
    sleep 1
  done
}

wait_log_pattern() {
  local pattern="$1"
  local timeout_s="$2"
  local start now
  start=$(date +%s)

  while true; do
    if [[ -f "$LOG_FILE" ]] && grep -Eq "$pattern" "$LOG_FILE"; then
      return 0
    fi
    if [[ -s "$ERROR_MARKER" ]] || ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      return 1
    fi

    now=$(date +%s)
    if (( now - start >= timeout_s )); then
      return 1
    fi
    sleep 0.2
  done
}

wait_one_message() {
  local topic="$1"
  local timeout_s="$2"
  local outfile="$3"

  rm -f "$outfile"
  if timeout "${timeout_s}s" ros2 topic echo --once "$topic" >"$outfile" 2>/dev/null; then
    if [[ -s "$outfile" ]]; then
      echo "[OK] 收到消息: $topic"
      return 0
    fi
  fi
  return 1
}

start_topic_listener() {
  local topic="$1"
  local timeout_s="$2"
  local outfile="$3"
  local pid_name="$4"

  rm -f "$outfile" "${outfile}.err"
  (
    local deadline remaining
    deadline=$((SECONDS + timeout_s))
    while (( SECONDS < deadline )); do
      remaining=$((deadline - SECONDS))
      if timeout "${remaining}s" ros2 topic echo --once "$topic" \
        >"$outfile" 2>"${outfile}.err" && [[ -s "$outfile" ]]; then
        exit 0
      fi
      sleep 0.2
    done
    exit 1
  ) &
  printf -v "$pid_name" '%s' "$!"
}

wait_topic_listener() {
  local topic="$1"
  local pid="$2"
  local outfile="$3"

  if wait "$pid" 2>/dev/null && [[ -s "$outfile" ]]; then
    echo "[OK] 收到消息: $topic"
    return 0
  fi
  return 1
}

stop_topic_listener() {
  local pid="${1:-}"
  if [[ -z "$pid" ]]; then
    return 0
  fi
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

list_existing_nav3d_processes() {
  ps -eo pid=,ppid=,lstart=,cmd= | awk \
    -v self="$$" -v workspace="$WORKSPACE" '
    $1 == self { next }
    {
      command = $0
      # The inspection awk command contains the workspace path in its own
      # arguments; do not report that short-lived helper as a stale node.
      if (command ~ /[[:space:]]awk -v self=/) { next }
      if (command ~ /rosbag2_player/ ||
          command ~ /ros2 bag play/ ||
          command ~ /bringup_nav3d\.launch\.py/ ||
          index(command, workspace "/install/lio/") ||
          index(command, workspace "/install/lio_localization/") ||
          index(command, workspace "/install/map_loader/") ||
          index(command, workspace "/install/global_planner/") ||
          index(command, workspace "/install/scan_planner/") ||
          index(command, workspace "/install/bringup/")) {
        print command
      }
    }'
}

check_start_clean() {
  local existing
  existing="$(list_existing_nav3d_processes)"
  if [[ -n "$existing" ]]; then
    echo "[启动阻断] 检测到已有 rosbag/player、LIO 或 Nav3D 进程："
    printf '%s\n' "$existing"
    die "已有运行实例，禁止叠加启动。请先在原启动终端按 Ctrl+C，再重新执行本脚本。"
  fi
}

refresh_ros_graph() {
  echo "[检查] 刷新 ROS graph daemon，清理已退出节点的发现缓存"
  timeout 5s ros2 daemon stop >/dev/null 2>&1 || true
  sleep 0.5
  timeout 5s ros2 daemon start >/dev/null 2>&1 || true
  sleep 1
}

require_single_publisher() {
  local topic="$1"
  local label="$2"
  local info count attempt

  for attempt in {1..10}; do
    info="$(timeout 10s ros2 topic info -v "$topic" 2>/dev/null || true)"
    count="$(printf '%s\n' "$info" | awk '/^Publisher count:/{print $3; exit}')"
    if [[ "$count" == "1" ]]; then
      echo "[OK] $label publisher count=1: $topic"
      return 0
    fi
    sleep 1
  done

  echo "[失败] $label publisher count=$count，等待 ROS 图收敛后仍不为 1。"
  printf '%s\n' "$info"
  return 1
}

require_single_node() {
  local node="$1"
  local count attempt

  for attempt in {1..15}; do
    count="$(ros2 node list -a 2>/dev/null | grep -Fx "$node" | wc -l)"
    if [[ "$count" == "1" ]]; then
      echo "[OK] node 实例数=1: $node"
      return 0
    fi
    if [[ "$count" -gt 1 ]]; then
      break
    fi
    sleep 1
  done

  echo "[失败] node $node 实例数=$count，期望恰好为 1。"
  ros2 node list -a 2>/dev/null | grep -Fx "$node" || true
  return 1
}

validate_odom_stream() {
  local topic="$1"
  local sample_count="$2"
  local timeout_s="$3"
  local expected_frame="$4"
  local label="$5"

  python3 - "$topic" "$sample_count" "$timeout_s" "$expected_frame" "$label" <<'PY'
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


topic, required, timeout_s, expected_frame, label = sys.argv[1:]
required = int(required)
timeout_s = float(timeout_s)
state = {
    'count': 0,
    'previous_stamp': None,
    'first_stamp': None,
    'last_stamp': None,
    'failure': None,
}


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def callback(msg):
    if state['failure'] is not None:
        return

    if msg.header.frame_id != expected_frame or msg.child_frame_id != 'base_link':
        state['failure'] = (
            f"{label}: frame contract invalid: "
            f"{msg.header.frame_id!r}->{msg.child_frame_id!r}, "
            f"expected {expected_frame!r}->'base_link'")
        return

    values = [
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    ]
    if not all(math.isfinite(value) for value in values):
        state['failure'] = f"{label}: pose contains NaN or Inf"
        return

    quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
    if quaternion_norm <= 1.0e-12:
        state['failure'] = f"{label}: quaternion is denormalized or zero"
        return

    current_stamp = stamp_ns(msg)
    if state['previous_stamp'] is not None and current_stamp <= state['previous_stamp']:
        state['failure'] = (
            f"{label}: non-monotonic timestamp: "
            f"previous={state['previous_stamp']} current={current_stamp}")
        return

    if state['first_stamp'] is None:
        state['first_stamp'] = current_stamp
    state['previous_stamp'] = current_stamp
    state['last_stamp'] = current_stamp
    state['count'] += 1


rclpy.init()
node = rclpy.create_node('nav3d_odom_stream_validator')
node.create_subscription(Odometry, topic, callback, qos_profile_sensor_data)
deadline = time.monotonic() + timeout_s
try:
    while (state['failure'] is None and state['count'] < required and
           time.monotonic() < deadline):
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    node.destroy_node()
    rclpy.shutdown()

if state['failure'] is not None:
    print(f"[FAIL] {state['failure']}")
    sys.exit(2)
if state['count'] < required:
    print(
        f"[FAIL] {label}: only {state['count']}/{required} valid messages "
        f"received within {timeout_s:.1f}s")
    sys.exit(3)

delta = state['last_stamp'] - state['first_stamp'] if required > 1 else 0
print(
    f"[PASS] {label}: {state['count']} consecutive finite messages, "
    f"frame={expected_frame}->base_link, "
    f"timestamp_delta_ns={delta}")
PY
}

wait_odom_near_pose() {
  local topic="$1"
  local target_x="$2"
  local target_y="$3"
  local target_z="$4"
  local radius_xy="$5"
  local max_z_error="$6"
  local timeout_s="$7"

  python3 - \
    "$topic" "$target_x" "$target_y" "$target_z" \
    "$radius_xy" "$max_z_error" "$timeout_s" <<'PY'
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


topic = sys.argv[1]
target_x, target_y, target_z = map(float, sys.argv[2:5])
radius_xy, max_z_error, timeout_s = map(float, sys.argv[5:8])
state = {
    'matched': None,
    'failure': None,
    'previous_stamp': None,
    'last_pose': None,
    'closest_pose': None,
    'closest_score': None,
}


def callback(msg):
    if state['matched'] is not None or state['failure'] is not None:
        return
    if msg.header.frame_id != 'map' or msg.child_frame_id != 'base_link':
        state['failure'] = (
            f"invalid frame {msg.header.frame_id!r}->{msg.child_frame_id!r}")
        return

    pose = msg.pose.pose
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    )
    if not all(math.isfinite(value) for value in values):
        state['failure'] = 'pose contains NaN or Inf'
        return

    stamp = int(msg.header.stamp.sec) * 1_000_000_000 + msg.header.stamp.nanosec
    if state['previous_stamp'] is not None and stamp <= state['previous_stamp']:
        state['failure'] = (
            f"non-monotonic timestamp: previous={state['previous_stamp']} current={stamp}")
        return
    state['previous_stamp'] = stamp

    distance_xy = math.hypot(
        pose.position.x - target_x, pose.position.y - target_y)
    z_error = abs(pose.position.z - target_z)
    current_pose = (
        pose.position.x, pose.position.y, pose.position.z,
        distance_xy, z_error, stamp)
    score = distance_xy + z_error
    state['last_pose'] = current_pose
    if state['closest_score'] is None or score < state['closest_score']:
        state['closest_score'] = score
        state['closest_pose'] = current_pose
    if distance_xy <= radius_xy and z_error <= max_z_error:
        state['matched'] = current_pose


rclpy.init()
node = rclpy.create_node('nav3d_auto_goal_trigger')
node.create_subscription(Odometry, topic, callback, qos_profile_sensor_data)
deadline = time.monotonic() + timeout_s
try:
    while (state['matched'] is None and state['failure'] is None and
           time.monotonic() < deadline):
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    node.destroy_node()
    rclpy.shutdown()

if state['failure'] is not None:
    print(f"[FAIL] 自动 Goal 触发定位无效: {state['failure']}")
    sys.exit(2)
if state['matched'] is None:
    print(
        f"[FAIL] 定位未在 {timeout_s:.1f}s 内到达自动 Goal 触发区: "
        f"target=({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
    if state['closest_pose'] is not None:
        x, y, z, distance_xy, z_error, stamp = state['closest_pose']
        print(
            f"[FAIL] 最接近触发区的定位: actual=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"distance_xy={distance_xy:.3f} z_error={z_error:.3f} stamp_ns={stamp}")
    if state['last_pose'] is not None:
        x, y, z, distance_xy, z_error, stamp = state['last_pose']
        print(
            f"[FAIL] 最近一帧定位: actual=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"distance_xy={distance_xy:.3f} z_error={z_error:.3f} stamp_ns={stamp}")
    sys.exit(3)

x, y, z, distance_xy, z_error, stamp = state['matched']
print(
    f"[PASS] 已到达自动 Goal 触发区: actual=({x:.3f}, {y:.3f}, {z:.3f}) "
    f"distance_xy={distance_xy:.3f} z_error={z_error:.3f} stamp_ns={stamp}")
PY
}

latest_odom_same_floor_as_goal() {
  local topic="$1"
  local init_x="$2"
  local init_y="$3"
  local init_z="$4"
  local goal_x="$5"
  local goal_y="$6"
  local goal_z="$7"
  local max_z_error="$8"
  local max_progress="$9"
  local timeout_s="${10}"

  python3 - \
    "$topic" "$init_x" "$init_y" "$init_z" \
    "$goal_x" "$goal_y" "$goal_z" \
    "$max_z_error" "$max_progress" "$timeout_s" <<'PY'
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


topic = sys.argv[1]
init = tuple(map(float, sys.argv[2:5]))
goal = tuple(map(float, sys.argv[5:8]))
max_z_error = float(sys.argv[8])
max_progress = float(sys.argv[9])
timeout_s = float(sys.argv[10])
state = {'pose': None, 'failure': None}
segment = (
    goal[0] - init[0],
    goal[1] - init[1],
    goal[2] - init[2],
)
segment_norm2 = sum(value * value for value in segment)


def callback(msg):
    if state['pose'] is not None or state['failure'] is not None:
        return
    if msg.header.frame_id != 'map' or msg.child_frame_id != 'base_link':
        state['failure'] = (
            f"invalid frame {msg.header.frame_id!r}->{msg.child_frame_id!r}")
        return

    pose = msg.pose.pose
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    )
    if not all(math.isfinite(value) for value in values):
        state['failure'] = 'pose contains NaN or Inf'
        return

    stamp = int(msg.header.stamp.sec) * 1_000_000_000 + msg.header.stamp.nanosec
    position = (pose.position.x, pose.position.y, pose.position.z)
    z_error = abs(position[2] - goal[2])
    progress = 0.0
    if segment_norm2 > 1.0e-12:
        progress = sum(
            (position[i] - init[i]) * segment[i] for i in range(3)) / segment_norm2
    state['pose'] = (
        position[0], position[1], position[2], z_error, progress, stamp)


rclpy.init()
node = rclpy.create_node('nav3d_goal_floor_checker')
node.create_subscription(Odometry, topic, callback, qos_profile_sensor_data)
deadline = time.monotonic() + timeout_s
try:
    while state['pose'] is None and state['failure'] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    node.destroy_node()
    rclpy.shutdown()

if state['failure'] is not None:
    print(f"[FAIL] Goal 同层检查定位无效: {state['failure']}")
    sys.exit(2)
if state['pose'] is None:
    print(f"[FAIL] Goal 同层检查未在 {timeout_s:.1f}s 内收到定位")
    sys.exit(3)

x, y, z, z_error, progress, stamp = state['pose']
print(
    f"[OK] Goal 同层检查: current=({x:.3f}, {y:.3f}, {z:.3f}) "
    f"goal_z={goal[2]:.3f} z_error={z_error:.3f} "
    f"route_progress={progress:.3f} stamp_ns={stamp}")
if z_error > max_z_error or progress < -0.10 or progress > max_progress:
    sys.exit(4)
PY
}

UPSTREAM_ERROR_PATTERN='pose contains NaN or Inf|TF_NAN_INPUT|TF_DENORMALIZED_QUATERNION|non-monotonic raw IMU timestamps|time_stamp is out of range|Requested time range is out of range|too few points|IMU Data Not Enough'

monitor_upstream_log() {
  while [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; do
    if [[ -s "$LOG_FILE" ]]; then
      local match category
      match="$(rg -n -i -m 1 "$UPSTREAM_ERROR_PATTERN" "$LOG_FILE" 2>/dev/null || true)"
      if [[ -n "$match" ]]; then
        case "$match" in
          *"non-monotonic raw IMU"*|*"time_stamp is out of range"*|*"Requested time range"*|*"too few points"*|*"IMU Data Not Enough"*|*"TF_NAN_INPUT"*|*"TF_DENORMALIZED_QUATERNION"*)
            category="LIO failure"
            ;;
          *)
            category="Localization failure"
            ;;
        esac
        {
          echo "[分类] $category"
          echo "$match"
        } > "$ERROR_MARKER"
        kill -TERM "$MAIN_PID" 2>/dev/null || true
        return 0
      fi
    fi
    sleep 0.25
  done
}

show_msg_head() {
  local file="$1"
  local lines="${2:-35}"
  if [[ -f "$file" ]]; then
    sed -n "1,${lines}p" "$file"
  fi
}

validate_planar_alignment_log() {
  local log_file="$1"
  local tolerance_deg="$2"

  python3 - "$log_file" "$tolerance_deg" <<'PY'
import math
import re
import sys

log_file = sys.argv[1]
tolerance_deg = float(sys.argv[2])
pattern = re.compile(
    r'(Initialized map -> odom|Updated map -> odom).*'
    r'rpy_deg=\(([-+0-9.eE]+),\s*([-+0-9.eE]+),\s*([-+0-9.eE]+)\)')

init_count = 0
update_count = 0
bad_lines = []

with open(log_file, 'r', encoding='utf-8', errors='replace') as handle:
    for line in handle:
        if 'Initialized map -> odom' not in line and 'Updated map -> odom' not in line:
            continue

        if 'planar alignment' not in line:
            bad_lines.append(f'non-planar map->odom log: {line.strip()}')
            continue

        match = pattern.search(line)
        if not match:
            bad_lines.append(f'missing rpy_deg diagnostics: {line.strip()}')
            continue

        kind = match.group(1)
        roll = float(match.group(2))
        pitch = float(match.group(3))
        if kind.startswith('Initialized'):
            init_count += 1
        else:
            update_count += 1

        if not math.isfinite(roll) or not math.isfinite(pitch):
            bad_lines.append(f'non-finite map->odom roll/pitch: {line.strip()}')
        elif abs(roll) > tolerance_deg or abs(pitch) > tolerance_deg:
            bad_lines.append(
                f'map->odom is tilted: roll={roll:.3f} deg pitch={pitch:.3f} deg '
                f'tolerance={tolerance_deg:.3f} deg')

if init_count == 0:
    bad_lines.append('missing planar Initialized map -> odom log')

if bad_lines:
    for item in bad_lines:
        print(f'[FAIL] {item}')
    sys.exit(2)

print(
    f'[PASS] map->odom planar alignment: init_logs={init_count} '
    f'update_logs={update_count} roll/pitch_tolerance_deg={tolerance_deg:.3f}')
PY
}

require_tf_transform() {
  local target_frame="$1"
  local source_frame="$2"
  local timeout_s="$3"
  local label="$4"
  local target_safe source_safe outfile

  target_safe="${target_frame//\//_}"
  source_safe="${source_frame//\//_}"
  outfile="$RUN_DIR/tf_${target_safe}_${source_safe}.txt"

  python3 - "$target_frame" "$source_frame" "$timeout_s" >"$outfile" 2>&1 <<'PY' || true
import math
import sys
import time

import rclpy
from tf2_ros import Buffer, TransformListener


target_frame, source_frame, timeout_s = sys.argv[1], sys.argv[2], float(sys.argv[3])
rclpy.init()
node = rclpy.create_node('nav3d_tf_once_checker')
buffer = Buffer()
listener = TransformListener(buffer, node)
deadline = time.monotonic() + timeout_s
last_error = ''
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            transform = buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            print('Translation:')
            print(
                f'- Translation: [{translation.x:.3f}, '
                f'{translation.y:.3f}, {translation.z:.3f}]')
            print(
                f'- Rotation: in Quaternion (xyzw) '
                f'[{rotation.x:.3f}, {rotation.y:.3f}, '
                f'{rotation.z:.3f}, {rotation.w:.3f}]')
            sys.exit(0)
        except Exception as exc:  # tf2 exposes several distro-specific errors.
            last_error = str(exc)
finally:
    node.destroy_node()
    rclpy.shutdown()

print(f'No transform {target_frame} -> {source_frame}: {last_error}')
sys.exit(1)
PY

  if grep -q "Translation:" "$outfile"; then
    echo "[PASS] TF 连通: $label ($target_frame -> $source_frame)"
    grep -m 1 -A 2 "Translation:" "$outfile" || true
    return 0
  fi

  echo "[FAIL] TF 不连通: $label ($target_frame -> $source_frame)"
  tail -n 8 "$outfile" || true
  return 1
}

require_robot_tf_chain() {
  require_tf_transform "map" "base_link" "$TF_CHECK_TIMEOUT" "定位输出 map->base_link" \
    || return 1
  require_tf_transform "map" "lidar_frame" "$TF_CHECK_TIMEOUT" "雷达外参链 map->base_link->lidar_frame" \
    || return 1
}

require_scan_tf_chain() {
  require_robot_tf_chain || return 1
  require_tf_transform "map" "sliding_map" "$TF_CHECK_TIMEOUT" "SCAN 滑窗 map->sliding_map" \
    || return 1
}

fail_if_local_planner_emergency() {
  local match
  match="$(rg -n -m 1 "Replan failed [0-9]+ times|EMERGENCY_STOP" "$LOG_FILE" 2>/dev/null || true)"
  if [[ -n "$match" ]]; then
    echo "[FAIL] 局部规划进入 emergency/replan failure:"
    echo "$match"
    return 1
  fi
  return 0
}

check_bag_player() {
  ros2 node list 2>/dev/null | grep -Eqi 'rosbag|player'
}

quat_from_yaw() {
  local yaw="$1"
  python3 - "$yaw" <<'PY'
import math, sys
yaw=float(sys.argv[1])
print(f"{math.sin(yaw/2.0):.12f} {math.cos(yaw/2.0):.12f}")
PY
}

# =========================
# 1. 环境检查
# =========================
step "1/10 检查 ROS2 / 工作区 / 数据"

[[ -d "$WORKSPACE" ]] || die "工作区不存在: $WORKSPACE"
[[ -d "$BAG_PATH" ]] || die "rosbag 路径不存在: $BAG_PATH"
[[ -f "$MAP_PATH" ]] || die "PCD 地图不存在: $MAP_PATH"
[[ -f /opt/ros/humble/setup.bash ]] || die "未找到 /opt/ros/humble/setup.bash"
[[ -f "$WORKSPACE/bringup/config/localization_icp.yaml" ]] \
  || die "定位 ICP 配置不存在: $WORKSPACE/bringup/config/localization_icp.yaml"
grep -Eq '^[[:space:]]*planar_initial_alignment:[[:space:]]*true[[:space:]]*$' \
  "$WORKSPACE/bringup/config/localization_icp.yaml" \
  || die "localization_icp.yaml 未开启 planar_initial_alignment: true"

cd "$WORKSPACE" || die "无法进入工作区"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

echo "[OK] WORKSPACE = $WORKSPACE"
echo "[OK] BAG       = $BAG_PATH"
echo "[OK] MAP       = $MAP_PATH"
echo "[OK] INIT      = $INIT_X,$INIT_Y,$INIT_Z,$INIT_YAW_RAD"
echo "[OK] GOAL      = $GOAL_X,$GOAL_Y,$GOAL_Z,$GOAL_YAW_RAD"
echo "[OK] ALIGNMENT = planar map->odom, roll/pitch tolerance ${PLANAR_RPY_TOL_DEG} deg"
echo "[OK] POST_PASS = $([[ "$KEEP_RUNNING_AFTER_PASS" -eq 1 ]] && echo keep_running || echo auto_cleanup)"

check_start_clean
refresh_ros_graph

# =========================
# 2. 编译
# =========================
if [[ "$BUILD_ON_START" -eq 1 ]]; then
  step "2/10 编译 Nav3D 所需包"

  CONFLICT_PATH="$WORKSPACE/build/scan_planner_msgs/ament_cmake_python/scan_planner_msgs/scan_planner_msgs"
  if [[ -d "$CONFLICT_PATH" && ! -L "$CONFLICT_PATH" ]]; then
    echo "[提示] 检测到 scan_planner_msgs 旧目录与 symlink 冲突，自动清理该包旧产物。"
    rm -rf "$WORKSPACE/build/scan_planner_msgs" "$WORKSPACE/install/scan_planner_msgs"
  fi

  colcon build \
    --packages-select \
      lio \
      lio_localization \
      map_loader \
      global_planner \
      local_sensing_node \
      bringup \
      scan_planner \
      scan_planner_msgs \
    --symlink-install \
    --cmake-args -DHUMBLE_ROS=humble \
    || die "colcon build 失败"

  echo "[OK] 编译完成"
else
  step "2/10 跳过编译"
fi

[[ -f "$WORKSPACE/install/setup.bash" ]] || die "install/setup.bash 不存在，工作区可能未成功编译"
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"

# =========================
# 3. 启动 bringup
# =========================
step "3/10 启动完整联合模块"

echo "[提示] bringup 完整日志保存到: $LOG_FILE"
AUTO_INITIALPOSE="false"
INITIALPOSE_ARG=""
if [[ "$INIT_MODE" == "auto" ]]; then
  [[ -n "$INIT_X" && -n "$INIT_Y" && -n "$INIT_Z" && -n "$INIT_YAW_RAD" ]] \
    || die "INIT_MODE=auto，但 INIT_X/Y/Z/YAW_RAD 没有全部填写"
  AUTO_INITIALPOSE="true"
  INITIALPOSE_ARG="$INIT_X,$INIT_Y,$INIT_Z,$INIT_YAW_RAD"
elif [[ "$INIT_MODE" != "rviz" ]]; then
  die "INIT_MODE 只支持 auto 或 rviz，当前值为: $INIT_MODE"
fi

RUN_MODE="bag"
USE_LOCALIZATION="true"
if [[ "$CLOSED_LOOP_SIM" -eq 1 ]]; then
  RUN_MODE="sim"
  USE_LOCALIZATION="false"
  AUTO_INITIALPOSE="false"
  INITIALPOSE_ARG="$INIT_X,$INIT_Y,$INIT_Z,$INIT_YAW_RAD"
fi

setsid --wait ros2 launch bringup bringup_nav3d.launch.py \
  mode:="$RUN_MODE" \
  use_localization:="$USE_LOCALIZATION" \
  bag_path:="$BAG_PATH" \
  bag_loop:=false \
  bag_start_delay:="$BAG_START_DELAY" \
  map:="$MAP_PATH" \
  auto_initialpose:="$AUTO_INITIALPOSE" \
  initialpose:="$INITIALPOSE_ARG" \
  enable_rviz:="$ENABLE_RVIZ" \
  enable_scan_context:="$ENABLE_SCAN_CONTEXT" \
  >"$LOG_FILE" 2>&1 &

LAUNCH_PID=$!
echo "[OK] bringup PID = $LAUNCH_PID"

sleep 3
kill -0 "$LAUNCH_PID" 2>/dev/null || die "bringup 启动后立即退出"
LAUNCH_PGID="$(ps -o pgid= -p "$LAUNCH_PID" | tr -d ' ')"
if [[ -z "$LAUNCH_PGID" || "$LAUNCH_PGID" == "$SCRIPT_PGID" ]]; then
  die "无法建立独立 bringup 进程组，拒绝继续启动"
fi
monitor_upstream_log &
ERROR_MONITOR_PID=$!

# =========================
# 4. 等核心 topics
# =========================
step "4/10 等待核心接口出现"

if [[ "$CLOSED_LOOP_SIM" -eq 1 ]]; then
  echo "[模式] 固定地图闭环仿真：不启动 rosbag/LIO，统一使用 /cmd_vel -> $LOCAL_ODOM_TOPIC"
else
  wait_topic_exists "/livox/imu" "$STARTUP_TIMEOUT" \
    || die "[分类] LIO failure: 未发现 LIO IMU 输入 /livox/imu"

  wait_topic_exists "/livox/lidar" "$STARTUP_TIMEOUT" \
    || die "[分类] LIO failure: 未发现 LIO LiDAR 输入 /livox/lidar"

  require_single_node "/rosbag2_player" \
    || die "[分类] LIO failure: rosbag2_player 实例数不为 1"

  require_single_publisher "/livox/imu" "LIO IMU 输入" \
    || die "[分类] LIO failure: /livox/imu publisher 数量不是 1"

  require_single_publisher "/livox/lidar" "LIO LiDAR 输入" \
    || die "[分类] LIO failure: /livox/lidar publisher 数量不是 1"

  wait_topic_exists "/lio/mapping/odom_body" "$STARTUP_TIMEOUT" \
    || die "[分类] LIO failure: 未发现 /lio/mapping/odom_body"

  require_single_publisher "/lio/mapping/odom_body" "LIO 原始里程计输出" \
    || die "[分类] LIO failure: /lio/mapping/odom_body publisher 数量不是 1"
fi

wait_topic_exists "/initialpose" "$STARTUP_TIMEOUT" \
  || die "[分类] Localization failure: 未发现 /initialpose"

wait_topic_exists "/map_loader/octomap" "$STARTUP_TIMEOUT" \
  || die "[分类] Global planning failure: 未发现 /map_loader/octomap"

wait_topic_exists "/global_planner/path" "$STARTUP_TIMEOUT" \
  || die "[分类] Global planning failure: 未发现 /global_planner/path"

wait_topic_exists "/planning/bspline" "$STARTUP_TIMEOUT" \
  || die "[分类] Local planning failure: 未发现 /planning/bspline"

wait_topic_exists "$CMD_VEL_TOPIC" "$STARTUP_TIMEOUT" \
  || die "[分类] Local planning failure: 未发现 $CMD_VEL_TOPIC"

require_single_publisher "$CMD_VEL_TOPIC" "局部控制输出" \
  || die "[分类] Local planning failure: $CMD_VEL_TOPIC publisher 数量不是 1"

wait_topic_exists "$LOCAL_ODOM_TOPIC" "$STARTUP_TIMEOUT" \
  || die "[分类] Local planning failure: 未发现 $LOCAL_ODOM_TOPIC"

require_single_publisher "$LOCAL_ODOM_TOPIC" "局部闭环 odom" \
  || die "[分类] Local planning failure: $LOCAL_ODOM_TOPIC publisher 数量不是 1"

echo
echo "[检查] /initialpose 连接关系："
ros2 topic info /initialpose -v || true

if [[ "$CLOSED_LOOP_SIM" -eq 1 ]]; then
  if ! ros2 topic info /initialpose -v 2>/dev/null | grep -q "Node name: go2_kinematic_sim"; then
    die "go2_kinematic_sim 没有订阅 /initialpose"
  fi
  echo "[OK] go2_kinematic_sim 已订阅 /initialpose"
elif ! ros2 topic info /initialpose -v 2>/dev/null | grep -q "Node name: localization_composer"; then
  die "localization_composer 没有订阅 /initialpose"
else
  echo "[OK] localization_composer 已订阅 /initialpose"
fi

# =========================
# 5. 确认 LIO 数据真的在流动
# =========================
if [[ "$CLOSED_LOOP_SIM" -eq 0 ]]; then
  step "5/10 等待 LIO 原始里程计数据"

  if ! validate_odom_stream \
    "/lio/mapping/odom_body" 20 "$DATA_TIMEOUT" "odom" "LIO 原始里程计"; then
    if ! check_bag_player; then
      die "[分类] LIO failure: 连续验证 /lio/mapping/odom_body 失败，且 rosbag player 已不存在"
    fi
    die "[分类] LIO failure: /lio/mapping/odom_body 未通过连续 finite/单调时间戳验证"
  fi
fi

# =========================
# 6. initialpose
# =========================
step "6/10 建立 map -> odom 初始定位"

if [[ "$CLOSED_LOOP_SIM" -eq 1 ]]; then
  read -r INIT_QZ INIT_QW < <(quat_from_yaw "$INIT_YAW_RAD")
  echo "[自动] 发布闭环仿真初始位姿 /initialpose:"
  echo "       x=$INIT_X y=$INIT_Y z=$INIT_Z yaw=$INIT_YAW_RAD"
  ros2 topic pub --once --wait-matching-subscriptions 1 --keep-alive 1.0 \
    /initialpose \
    geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: $INIT_X, y: $INIT_Y, z: $INIT_Z}, orientation: {x: 0.0, y: 0.0, z: $INIT_QZ, w: $INIT_QW}}}}" \
    >/dev/null \
    || die "[分类] Localization failure: 闭环仿真 /initialpose 发布失败"
  echo "[OK] 已向 go2_kinematic_sim 发布 map 初始位姿；后续以 $PLANNING_ODOM_TOPIC 连续输出验收"
elif [[ "$INIT_MODE" == "auto" ]]; then
  echo "[自动] 首个有效 raw odom 已绑定到 map 位姿:"
  echo "       x=$INIT_X y=$INIT_Y z=$INIT_Z yaw=$INIT_YAW_RAD"
  if ! wait_log_pattern "Published configured map-frame /initialpose" "$INITIALPOSE_TIMEOUT"; then
    die "[分类] Localization failure: 自动 initialpose 节点没有完成首帧发布"
  fi
  if ! wait_log_pattern "Initialized map -> odom.*planar alignment" "$INITIALPOSE_TIMEOUT"; then
    die "[分类] Localization failure: composer 未使用 planar alignment 初始化 map -> odom"
  fi
  validate_planar_alignment_log "$LOG_FILE" "$PLANAR_RPY_TOL_DEG" \
    || die "[分类] Localization failure: map->odom 初始化姿态不是水平 planar 对齐"
  echo "[OK] 自动 /initialpose 已由 localization_composer 按首帧时间戳接收"
else
  echo
  echo ">>> 现在只做一个鼠标动作："
  echo ">>> 在 RViz 顶部选择 [2D Pose Estimate]"
  echo ">>> 在 PCD 地图中机器人真实起点按下鼠标并拖出真实朝向，然后松开。"
  echo
  echo "脚本正在等待 /initialpose，最多等待 ${INITIALPOSE_TIMEOUT}s ..."
  if ! wait_one_message "/initialpose" "$INITIALPOSE_TIMEOUT" "$RUN_DIR/initialpose.yaml"; then
    if ! check_bag_player; then
      die "等待 initialpose 期间 rosbag 已结束；请重新运行脚本并尽快在 RViz 设置初始位姿"
    fi
    die "等待 /initialpose 超时；RViz 可能没有成功发布 2D Pose Estimate"
  fi

  echo "---- 收到的 initialpose ----"
  show_msg_head "$RUN_DIR/initialpose.yaml" 30
  echo "----------------------------"
fi

# =========================
# 7. 定位验收
# =========================
step "7/10 等待统一闭环定位 $PLANNING_ODOM_TOPIC"

if ! wait_topic_exists "$PLANNING_ODOM_TOPIC" "$LOCALIZATION_TIMEOUT"; then
  die "[分类] Localization failure: 没有出现 $PLANNING_ODOM_TOPIC"
fi

require_single_publisher "$PLANNING_ODOM_TOPIC" "统一闭环定位输出" \
  || die "[分类] Localization failure: $PLANNING_ODOM_TOPIC publisher 数量不是 1"

if ! validate_odom_stream \
  "$PLANNING_ODOM_TOPIC" 20 "$LOCALIZATION_TIMEOUT" "map" "统一闭环定位里程计"; then
  if [[ "$CLOSED_LOOP_SIM" -eq 0 ]] && ! check_bag_player; then
    die "[分类] Localization failure: 连续验证 $PLANNING_ODOM_TOPIC 失败，且 rosbag player 已不存在"
  fi
  die "[分类] Localization failure: $PLANNING_ODOM_TOPIC 未通过连续 finite/单调时间戳验证"
fi

if [[ "$CLOSED_LOOP_SIM" -eq 0 ]]; then
  echo
  echo "---- 定位相关日志 ----"
  grep -E "Initialized map -> odom|Published fixed-map ICP correction|Updated map -> odom|Ignoring pose correction" \
    "$LOG_FILE" | tail -n 25 || true
  echo "----------------------"

  validate_planar_alignment_log "$LOG_FILE" "$PLANAR_RPY_TOL_DEG" \
    || die "[分类] Localization failure: map->odom 被 full SE3 或 roll/pitch 倾斜污染"
else
  echo "[PASS] 定位来源唯一：$PLANNING_ODOM_TOPIC，由 /cmd_vel 闭环模拟器发布"
fi

require_robot_tf_chain \
  || die "[分类] Localization/TF failure: map->base_link->lidar_frame TF 树不连通"

if grep -q "Initialized map -> odom" "$LOG_FILE"; then
  echo "[PASS] 已找到 Initialized map -> odom"
else
  echo "[提示] 日志暂未找到 'Initialized map -> odom'，但以实际 localization odom 数据为主。"
fi

# =========================
# 8. 目标点
# =========================
if [[ "$GOAL_MODE" == "off" ]]; then
  step "8/10 已按配置跳过规划目标"
  echo "[完成] 定位接口验收到这里结束。"
  exit 0
fi

step "8/10 设置全局规划目标"

if ! wait_log_pattern "OctoMap (received|ready):" "$STARTUP_TIMEOUT"; then
  die "[分类] Global planning failure: Global Planner 地图尚未完成初始化"
fi
echo "[OK] Global Planner 地图已完成初始化"

if ! validate_odom_stream \
  "$PLANNING_ODOM_TOPIC" 1 "$LOCALIZATION_TIMEOUT" "map" "Goal 前最新定位"; then
  die "[分类] Localization failure: 发布 goal 前最新定位不是有效 map -> base_link"
fi
echo "[OK] 发布 goal 前最新 localization 已通过 frame/finite 检查"

require_robot_tf_chain \
  || die "[分类] Localization/TF failure: 发布 goal 前 map->base_link->lidar_frame TF 已断开"

if [[ "$GOAL_MODE" == "auto" ]]; then
  [[ -n "$GOAL_X" && -n "$GOAL_Y" && -n "$GOAL_Z" && -n "$GOAL_YAW_RAD" ]] \
    || die "GOAL_MODE=auto，但 GOAL_X/Y/Z/YAW_RAD 没有全部填写"

  if [[ "$INIT_MODE" == "auto" ]]; then
    if [[ "$CLOSED_LOOP_SIM" -eq 1 ]] || latest_odom_same_floor_as_goal \
      "$PLANNING_ODOM_TOPIC" \
      "$INIT_X" "$INIT_Y" "$INIT_Z" \
      "$GOAL_X" "$GOAL_Y" "$GOAL_Z" \
      "$AUTO_GOAL_TRIGGER_MAX_Z_ERROR" \
      "$AUTO_GOAL_SAME_FLOOR_MAX_PROGRESS" \
      "$LOCALIZATION_TIMEOUT"; then
      echo "[自动] 当前定位已与 goal 同层，跳过中间触发点等待。"
    else
      read -r GOAL_TRIGGER_X GOAL_TRIGGER_Y GOAL_TRIGGER_Z < <(
        python3 - \
          "$INIT_X" "$INIT_Y" "$INIT_Z" \
          "$GOAL_X" "$GOAL_Y" "$GOAL_Z" \
          "$AUTO_GOAL_TRIGGER_PROGRESS" <<'PY'
import sys

start = tuple(map(float, sys.argv[1:4]))
goal = tuple(map(float, sys.argv[4:7]))
progress = float(sys.argv[7])
trigger = tuple(a + progress * (b - a) for a, b in zip(start, goal))
print(*(f'{value:.9f}' for value in trigger))
PY
      )

      echo "[自动] 当前定位不在 goal 同层，等待预录定位进入已验证的同层规划段:"
      echo "       target=($GOAL_TRIGGER_X, $GOAL_TRIGGER_Y, $GOAL_TRIGGER_Z)"
      if ! wait_odom_near_pose \
        "$PLANNING_ODOM_TOPIC" \
        "$GOAL_TRIGGER_X" "$GOAL_TRIGGER_Y" "$GOAL_TRIGGER_Z" \
        "$AUTO_GOAL_TRIGGER_RADIUS_XY" "$AUTO_GOAL_TRIGGER_MAX_Z_ERROR" \
        "$LOCALIZATION_TIMEOUT"; then
        if ! check_bag_player; then
          die "[分类] Localization failure: rosbag 播放结束前未到达自动 Goal 触发区，未发布 /goal_pose"
        fi
        die "[分类] Localization failure: 未到达自动 Goal 的同层触发区"
      fi
    fi
  fi

  read -r GOAL_QZ GOAL_QW < <(quat_from_yaw "$GOAL_YAW_RAD")

  echo "[自动] 发布 /goal_pose:"
  echo "       x=$GOAL_X y=$GOAL_Y z=$GOAL_Z yaw=$GOAL_YAW_RAD"

  step "9/10 在发布 Goal 前建立规划结果监听"
  start_topic_listener \
    "/global_planner/path" "$PLAN_TIMEOUT" "$RUN_DIR/global_path.yaml" PATH_LISTENER_PID
  start_topic_listener \
    "/planning/bspline" "$PLAN_TIMEOUT" "$RUN_DIR/bspline.yaml" BSPLINE_LISTENER_PID
  start_topic_listener \
    "$CMD_VEL_TOPIC" "$PLAN_TIMEOUT" "$RUN_DIR/cmd_vel.yaml" CMD_VEL_LISTENER_PID
  sleep 1

  # Both the global planner and safety supervisor consume the goal.  Wait for
  # both DDS matches so a single publication cannot race either subscriber.
  ros2 topic pub --once --wait-matching-subscriptions 2 --keep-alive 1.0 \
    /goal_pose \
    geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}, orientation: {x: 0.0, y: 0.0, z: $GOAL_QZ, w: $GOAL_QW}}}" \
    >/dev/null \
    || die "自动发布 /goal_pose 失败"
else
  echo
  echo ">>> 定位已经通过。现在只做第二个鼠标动作："
  echo ">>> 在 RViz 使用发布 /goal_pose 的 Goal Tool / 2D Goal Pose，"
  echo ">>> 在同一楼层可通行位置点击目标并拖出方向。"
  echo
  echo "脚本正在等待 /goal_pose，最多等待 ${GOAL_TIMEOUT}s ..."
  if ! wait_one_message "/goal_pose" "$GOAL_TIMEOUT" "$RUN_DIR/goal_pose.yaml"; then
    if ! check_bag_player; then
      die "等待目标期间 rosbag 已结束"
    fi
    die "等待 /goal_pose 超时；请确认 RViz Goal Tool 发布的是 /goal_pose"
  fi
  echo "[OK] 收到 /goal_pose"
fi

# =========================
# 9. 全局/局部规划验收
# =========================
if [[ "$GOAL_MODE" != "auto" ]]; then
  step "9/10 等待 global path / bspline / cmd_vel"
fi

if [[ "$GOAL_MODE" == "auto" ]]; then
  if ! wait_topic_listener "/global_planner/path" "$PATH_LISTENER_PID" "$RUN_DIR/global_path.yaml"; then
    die "[分类] Global planning failure: 没有收到 /global_planner/path"
  fi
else
  if ! wait_one_message "/global_planner/path" "$PLAN_TIMEOUT" "$RUN_DIR/global_path.yaml"; then
    die "[分类] Global planning failure: 没有收到 /global_planner/path"
  fi
fi
echo "[PASS] 收到 /global_planner/path"

if grep -q "frame_id: map" "$RUN_DIR/global_path.yaml"; then
  echo "[PASS] global path frame_id = map"
fi

if [[ "$GOAL_MODE" == "auto" ]]; then
  if ! wait_topic_listener "/planning/bspline" "$BSPLINE_LISTENER_PID" "$RUN_DIR/bspline.yaml"; then
    die "[分类] Local planning failure: 没有收到 /planning/bspline"
  fi
else
  if ! wait_one_message "/planning/bspline" "$PLAN_TIMEOUT" "$RUN_DIR/bspline.yaml"; then
    die "[分类] Local planning failure: 没有收到 /planning/bspline"
  fi
fi
echo "[PASS] 收到 /planning/bspline"

if ! validate_odom_stream \
  "$LOCAL_ODOM_TOPIC" 3 "$LOCALIZATION_TIMEOUT" "map" "局部规划闭环 odom"; then
  die "[分类] Local planning failure: 收到局部轨迹后 $LOCAL_ODOM_TOPIC 已停止或无效"
fi

require_scan_tf_chain \
  || die "[分类] Local planning/TF failure: 局部规划后 TF 树不完整"

if [[ "$GOAL_MODE" == "auto" ]]; then
  if ! wait_topic_listener "$CMD_VEL_TOPIC" "$CMD_VEL_LISTENER_PID" "$RUN_DIR/cmd_vel.yaml"; then
    die "[分类] Local planning failure: 没有收到 $CMD_VEL_TOPIC"
  fi
  # The controller publishes zero commands while waiting for a trajectory.
  # Take a fresh sample after B-spline reception so the final record belongs
  # to the planning response rather than the pre-goal idle state.
  if ! wait_one_message "$CMD_VEL_TOPIC" "$PLAN_TIMEOUT" "$RUN_DIR/cmd_vel.yaml"; then
    die "[分类] Local planning failure: 收到 global path/bspline，但没有取得规划后的 $CMD_VEL_TOPIC"
  fi
else
  if ! wait_one_message "$CMD_VEL_TOPIC" "$PLAN_TIMEOUT" "$RUN_DIR/cmd_vel.yaml"; then
    die "[分类] Local planning failure: 没有收到 $CMD_VEL_TOPIC"
  fi
fi
echo "[PASS] 收到 $CMD_VEL_TOPIC"

fail_if_local_planner_emergency \
  || die "[分类] Local planning failure: 收到局部输出后 SCAN 进入 emergency/replan failure"

# =========================
# 10. 总结
# =========================
step "10/10 Nav3D 联合验收 PASS"

echo "已确认："
if [[ "$CLOSED_LOOP_SIM" -eq 1 ]]; then
  echo "  [PASS] /initialpose -> go2_kinematic_sim"
  echo "  [PASS] $PLANNING_ODOM_TOPIC (唯一闭环定位输出)"
  echo "  [PASS] $MAP_PATH -> /local_planner/sim_cloud"
else
  echo "  [PASS] /lio/mapping/odom_body"
  echo "  [PASS] /initialpose -> localization_composer"
  echo "  [PASS] /lio/localization/odom"
fi
echo "  [PASS] /map_loader/octomap"
echo "  [PASS] /global_planner/path"
echo "  [PASS] /planning/bspline"
echo "  [PASS] $CMD_VEL_TOPIC"
echo "  [PASS] $LOCAL_ODOM_TOPIC"
echo "  [PASS] TF map->base_link->lidar_frame"
echo "  [PASS] TF map->sliding_map"
echo
echo "bringup 日志：$LOG_FILE"
echo
ACCEPTANCE_PASSED=1

if [[ "$KEEP_RUNNING_AFTER_PASS" -eq 1 ]]; then
  echo "脚本会继续保持 bringup 运行；bag 播放结束后 TF 会自然过期。"
  echo "按 Ctrl+C 时，本脚本会清理它启动的 bringup。"
  while kill -0 "$LAUNCH_PID" 2>/dev/null; do
    sleep 2
  done
else
  echo "联合验收已通过；默认自动清理 bringup，避免 bag 结束后的残留 TF 画面造成误判。"
  exit 0
fi

die "bringup 进程已退出"

# Nav3D

**基于 LiDAR-Inertial Odometry 和 Unitree A1 强化学习运动控制的三维导航系统**

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![License: GPL-2.0](https://img.shields.io/badge/License-GPL%202.0-green.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()

---

## 概述

Nav3D 是一个完整的三维自主导航系统，通过深度重构和整合现有开源组件（Elevator-LIO、SCAN-Planner、livox_ros_driver2、jie_octomap），并移植 legbot_3D_Nav 的强化学习运动控制到 ROS 2，实现 Gazebo 仿真和 Go2W 实机部署的完整导航闭环。

**项目目标：**

- **最终部署目标：** Unitree Go2W 实机三维导航
- **快速验证策略：** 先在 Gazebo 中使用 Unitree A1 强化学习运控验证完整流程
- **参考项目：** [legbot_3D_Nav ](https://github.com/Robot-Nav/legbot_3D_Nav)
- **开发策略：** 单 Git 仓库，所有组件统一管理，分层启动由 `bringup` 编排

**系统组件：**

- **LIO（LiDAR-Inertial Odometry）** — 实时三维建图和定位（参考 [Elevator-LIO](https://github.com/xiaofan4122/Elevator-LIO.git)）
- **Livox MID-360** — 固态激光雷达官方驱动
- **三维全局规划器** — 多层环境路径规划（参考 [jie_3d_nav](https://github.com/6-robot/jie_3d_nav.git) 和 [OctoPlanner3D-ROS2](https://github.com/JackJu-HIT/OctoPlanner3D-ROS2.git)）
- **局部规划器** — Mode 3 导航（参考 [SCAN-Planner-Ros2](https://github.com/xiaoqi371317/SCAN-Planner-Ros2.git)）
- **Unitree A1 强化学习运动控制** — 四足机器人 Gazebo 仿真（移植 `legbot_3D_Nav`中参考的 [unitree_guide](https://gitee.com/HuiyangCao/unitree_guide) 到 ROS 2 版本）
- **PCD 地图持久化** — 离线地图保存和加载
- **闭环检测** — scan_context_loop 全局定位
- **Gazebo Classic** — 仿真环境，包含停车场世界

**完整数据流（9 个组件协作）：**

```
sensor → lio_mapping → save_map → map_loader_node → 
lio_localization + scan_context_loop_node → 
global_planner → local_planner → rl_locomotion_controller
```

**主要特性：**

- 实时三维激光雷达惯性里程计
- PCD 地图持久化和闭环检测
- 多楼层导航
- 基于强化学习的四足机器人控制（TorchScript 推理）
- ROS 2 Humble 原生支持，完整传感器融合
- 支持 PointCloud2 和 Livox CustomMsg 格式点云

---

## 系统要求

### 硬件
- **CPU:** x86_64, 推荐 4 核以上
- **GPU:** NVIDIA GPU，支持 CUDA 12.x（用于强化学习策略推理）
- **内存:** 最低 16 GB，推荐 32 GB
- **存储:** 10 GB 可用空间

### 软件
- **操作系统:** Ubuntu 22.04 LTS
- **ROS 2:** Humble Hawksbill
- **Gazebo:** Gazebo Classic 11
- **CUDA:** 12.x（用于 Torch 构建）
- **LibTorch:** 2.5.1 cu126，cxx11 ABI

---

## 安装

### 1. 前置依赖

**安装 ROS 2 Humble：**
```bash
# 参考官方指南: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
sudo apt update && sudo apt install ros-humble-desktop
```

**安装依赖包：**
```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-controller-manager \
  ros-humble-effort-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-teleop-twist-keyboard \
  libeigen3-dev \
  libpcl-dev \
  libgtest-dev \
  cmake \
  build-essential
```

**安装 LibTorch（用于 A1 强化学习控制）：**
```bash
# 从 PyTorch 官网下载 LibTorch cu126
cd ~/3rd
wget https://download.pytorch.org/libtorch/cu126/libtorch-shared-with-deps-2.13.0%2Bcu126.zip
unzip libtorch-shared-with-deps-2.13.0%2Bcu126.zip
export LIBTORCH_ROOT="$HOME/3rd/libtorch"
```

### 2. 克隆仓库

```bash
# 工作空间为 planner_ws
mkdir -p /media/lenovo/disk/planner_ws/src
cd /media/lenovo/disk/planner_ws/src

# 克隆 Nav3D（如果尚未克隆）
git clone https://github.com/VisionTrekker/Nav3D.git Nav3D
cd Nav3D
```

**工作空间路径说明：**
- **工作空间：** `/media/lenovo/disk/planner_ws`
- **Git 仓库根目录：** `/media/lenovo/disk/planner_ws/src/Nav3D`
- **参考项目：** `/media/lenovo/disk/planner_ws/Ref_proj_tmp/`
  - `legbot_3D_Nav/` — A1 强化学习运控参考
  - `legbot_3D_Nav_blog/` — 开发博客和文档
  - `Elevator-LIO/` — LIO 建图参考
  - `SCAN-Planner-Ros2/` 和 `SCAN-Planner-Pure-ROS2/` — 局部规划器参考
  - `jie_3d_nav/` 和 `OctoPlanner3D-ROS2/` — 全局规划器参考
- **旧版设计文档：** `Nav3D/docs/superpowers/specs/2026-08-17-go2w-3d-nav-design.md`（本地 gitignored）

### 3. 编译

**标准编译（不含 Torch）：**

```bash
cd /media/lenovo/disk/planner_ws
source /opt/ros/humble/setup.zsh  # 或 setup.bash
colcon build --symlink-install
```

**启用 Torch 的编译（用于 A1 仿真）：**
```bash
cd /media/lenovo/disk/planner_ws
source /opt/ros/humble/setup.zsh

# 设置 LibTorch 环境变量
export LIBTORCH_ROOT="$HOME/3rd/libtorch"
export Torch_DIR="$LIBTORCH_ROOT/share/cmake/Torch"
export CMAKE_PREFIX_PATH="$LIBTORCH_ROOT:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$LIBTORCH_ROOT/lib:$LD_LIBRARY_PATH"

# 启用 Torch 支持编译
colcon build --symlink-install \
  --packages-select robot_rl_simulation \
  --cmake-args -DBUILD_TORCH_POLICY=ON
```

**选择性编译（特定包）：**
```bash
# 仅编译 LIO 包
colcon build --symlink-install --packages-select lio livox_ros_driver2

# 仅编译 A1 仿真
colcon build --symlink-install --packages-select robot_rl_simulation
```

---

## 快速开始

### 1. 激活工作空间

```bash
cd /media/lenovo/disk/planner_ws
source install/setup.zsh  # 或 setup.bash
```

### 2. 启动 A1 Gazebo 仿真

**基础 Gazebo 世界 + 建图：**

```bash
ros2 launch robot_rl_simulation mapping_sim.launch.py
```

这会启动：
- 带有停车场世界的 Gazebo
- Unitree A1 机器人模型
- `gazebo_ros2_control` 控制器
- 机体状态里程计发布器

**完整导航栈（计划中）：**

```bash
ros2 launch robot_rl_simulation navigation_sim.launch.py
```

### 3. 手动控制（Teleop）

```bash
# 在新终端中
cd /media/lenovo/disk/planner_ws
source install/setup.zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

使用键盘发送 `/cmd_vel` 命令：
- `i` / `k` : 前进 / 后退
- `j` / `l` : 左转 / 右转
- `u` / `o` : 对角移动
- `空格` : 紧急停止

---

## 架构

### 系统组件

```
┌─────────────────────────────────────────────────────────────┐
│                      Nav3D 系统                              │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐   ┌──────────────┐   ┌─────────────┐    │
│  │  Livox       │   │  IMU         │   │  里程计     │    │
│  │  MID-360     │──▶│  融合        │──▶│  (LIO)      │    │
│  └──────────────┘   └──────────────┘   └─────────────┘    │
│         │                                      │            │
│         └──────────────────┬──────────────────┘            │
│                            ▼                                │
│                   ┌─────────────────┐                       │
│                   │  三维建图       │                       │
│                   │  (PCD)          │                       │
│                   └─────────────────┘                       │
│                            │                                │
│                            ▼                                │
│                   ┌─────────────────┐                       │
│                   │  全局规划器     │                       │
│                   └─────────────────┘                       │
│                            │                                │
│                            ▼                                │
│                   ┌─────────────────┐                       │
│                   │  局部规划器     │                       │
│                   │  (SCAN Mode 3)  │                       │
│                   └─────────────────┘                       │
│                            │                                │
│                            ▼ /cmd_vel                       │
│                   ┌─────────────────┐                       │
│                   │ 强化学习运动    │                       │
│                   │ 控制器          │                       │
│                   │ (TorchScript)   │                       │
│                   └─────────────────┘                       │
│                            │                                │
│                            ▼ /effort                        │
│                   ┌─────────────────┐                       │
│                   │ Unitree A1      │                       │
│                   │ (Gazebo)        │                       │
│                   └─────────────────┘                       │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 包结构

```
Nav3D/
├── bringup/              # 启动文件和配置
│   ├── config/           # 导航参数

├── driver/               # 驱动
│   └── livox_ros_driver2/  # Livox MID-360 驱动
├── src/
│   ├── global_planner/   # 全局路径规划
│   ├── goal_marker_server/  # 终止点选取节点
│   ├── lio/              # 激光雷达惯性里程计
│   ├── lio_localization/  # 全局定位
│   ├── local_planner/    # 局部路径规划
│   ├── map_loader/       # PCD 地图加载器
│   ├── robot_rl_simulation/ # A1 强化学习控制
│   │   ├── config/       # ROS 参数
│   │   ├── description/  # URDF、网格、控制器
│   │   ├── gazebo_plugins/ # 机体状态插件
│   │   ├── include/      # C++ 头文件
│   │   ├── launch/       # 启动文件
│   │   ├── models/       # Gazebo 模型
│   │   ├── policies/     # RL运控模型
│   │   ├── src/          # C++ 实现
│   │   ├── test/         # GTest 测试套件
│   │   └── worlds/       # Gazebo 场景文件
│   └── scan_context_loop/ # 闭环检测
└── .superpowers/
    └── sdd/              # 系统设计文档
```

---

## 开发指南

### 编译系统

**工作空间结构：**
- **实际工作空间：** `/media/lenovo/disk/planner_ws`
- **Git 仓库根目录：** `/media/lenovo/disk/planner_ws/src/Nav3D`
- **参考项目根目录：** `/media/lenovo/disk/planner_ws/Ref_proj_tmp/`

**编译命令：**

```bash
# 清理编译
cd /media/lenovo/disk/planner_ws
colcon build --symlink-install

# 编译特定包
colcon build --symlink-install --packages-select robot_rl_simulation

# 详细编译输出
colcon build --symlink-install --event-handlers console_direct+

# 并行编译（4 个作业）
colcon build --symlink-install --parallel-workers 4
```

### 测试

**运行所有测试：**
```bash
colcon test
colcon test-result --verbose
```

**测试特定包：**
```bash
colcon test --packages-select robot_rl_simulation
colcon test-result --verbose --test-result-base build/robot_rl_simulation
```

**A1 仿真测试结果（当前）：**
- ✅ 5/5 测试套件通过
- ✅ 20/20 个测试通过
- 覆盖：契约、关节适配器、观测构建器、PD 控制器

### 代码审查

**提交前检查：**

1. **编译通过**
```bash
colcon build --packages-select <你的包> --event-handlers console_direct+
```

2. **所有测试通过**
```bash
colcon test --packages-select <你的包>
```

3. **无硬编码密钥**（API 密钥、凭据）

4. **代码风格**（C++ 用 clang-format，Python 用 black）

5. **提交消息格式：**
```
<类型>: <简短描述>

<可选详细说明>

<可选页脚>
```
类型：`feat`（新功能）、`fix`（修复）、`refactor`（重构）、`docs`（文档）、`test`（测试）、`chore`（杂项）、`perf`（性能）

### Git 工作流

**受保护文件（请勿还原）：**
- `bringup/config/frame_ids.yaml`（坐标系 ID 配置）
- `bringup/config/planning.yaml`（局部规划器参数）
- `src/map_loader/`（地图加载改动）
- `src/scan_context_loop/`（闭环检测改动）
- `.vscode/`, `build/`, `install/`, `log/`（IDE 和编译产物）

**暂存改动：**
```bash
# 检查状态
git status

# 暂存特定文件
git add src/robot_rl_simulation/src/rl_locomotion_a1.cpp

# 提交
git commit -m "feat(robot_rl): 实现策略推理循环"
```

---

## 故障排查

### 编译问题

**问题：** `Could NOT find Torch`

**解决方案：**
```bash
export LIBTORCH_ROOT="$HOME/3rd/libtorch"
export Torch_DIR="$LIBTORCH_ROOT/share/cmake/Torch"
export CMAKE_PREFIX_PATH="$LIBTORCH_ROOT:$CMAKE_PREFIX_PATH"
colcon build --cmake-args -DBUILD_TORCH_POLICY=ON
```

---

**问题：** `Gazebo protobuf 冲突`

**解决方案：** 已在 CMakeLists.txt 中通过限制 Torch include 范围修复。

---

**问题：** `undefined reference to 'at::cuda::is_available()'`

**解决方案：** 确保 `policy_runner.cpp` 中包含 `#include <torch/cuda.h>`

---

### 运行时问题

**问题：** 节点启动但日志显示 "TorchScript unavailable"

**解决方案：** 编译时未启用 Torch。使用 `-DBUILD_TORCH_POLICY=ON` 重新编译。

---

**问题：** 控制器未收到力矩命令

**解决方案：**
1. 检查控制器激活：`ros2 control list_controllers`
2. 检查话题：`ros2 topic echo /a1_effort_controller/commands`
3. 验证节点状态：检查日志中是否显示 "stopped" 或 "estop"

---

**问题：** A1 机器人在 Gazebo 中穿过地面

**解决方案：** 确保世界文件包含正确接触参数的 `<physics>` 标签。

---

### 性能问题

**问题：** 策略推理太慢（< 50 Hz）

**解决方案：**
1. 检查 CUDA 可用性：`nvidia-smi`
2. 检查 LibTorch 后端：节点日志应显示 "CUDA" 而非 "CPU"
3. 减少观测历史帧数（需重新训练策略）

---

**问题：** Gazebo 运行缓慢（< 实时）

**解决方案：**
1. 降低传感器更新频率
2. 简化碰撞网格
3. 禁用 GUI：`ros2 launch ... gui:=false`

---

## 配置

### A1 策略参数

编辑 `src/robot_rl_simulation/config/a1_policy.yaml`：

```yaml
rl_locomotion_a1:
  ros__parameters:
    policy_model_path: "install/robot_rl_simulation/share/robot_rl_simulation/policy/policy_act_inference_stair.pt"
    policy_rate_hz: 50.0          # 策略推理频率
    effort_publish_rate_hz: 200.0  # 力矩命令发布频率
    kp: 40.0                       # PD 比例增益
    kd: 0.5                        # PD 微分增益
    action_scale: 0.25             # 策略动作缩放
    cmd_vel_timeout_sec: 0.5       # 命令超时
    effort_limits: [33.5, 33.5, 33.5, 33.5, 33.5, 33.5, 33.5, 33.5, 33.5, 33.5, 33.5, 33.5]
```

### 控制器配置

编辑 `src/robot_rl_simulation/description/a1/config/controllers.yaml`：

```yaml
a1_effort_controller:
  type: effort_controllers/JointGroupEffortController
  joints:
    - FL_hip_joint
    - FL_thigh_joint
    - FL_calf_joint
    - FR_hip_joint
    - FR_thigh_joint
    - FR_calf_joint
    - RL_hip_joint
    - RL_thigh_joint
    - RL_calf_joint
    - RR_hip_joint
    - RR_thigh_joint
    - RR_calf_joint
```

---

## ROS 2 接口

### 话题

#### 订阅

| 话题 | 类型 | 描述 |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | 运动速度命令 |
| `/joint_states` | `sensor_msgs/JointState` | 关节位置/速度（200 Hz）|
| `/simulation/a1/body_state` | `nav_msgs/Odometry` | 机体位姿/速度（100 Hz）|
| `/emergency_stop` | `std_msgs/Bool` | 紧急停止信号 |

#### 发布

| 话题 | 类型 | 描述 |
|-------|------|-------------|
| `/a1_effort_controller/commands` | `std_msgs/Float64MultiArray` | 关节力矩命令（200 Hz）|

### 服务

*计划中：*
- `/rl_locomotion_a1/reset` — 重置策略状态
- `/rl_locomotion_a1/arm` — 启动控制器（从 estop 恢复）

### 参数

完整参数列表见 `config/a1_policy.yaml`。

---

## 性能指标

### A1 强化学习控制（当前状态）

- **策略推理：** 50 Hz（目标）
- **力矩发布：** 200 Hz
- **观测延迟：** < 5 ms
- **GTest 覆盖率：** 20/20 测试通过（契约、适配器、控制器）

### LIO（生产环境）

- **建图频率：** 10 Hz
- **定位频率：** 200 Hz
- **点云处理：** 实时
- **闭环检测：** 基于子图

---

## 已知问题和限制

### 当前限制

1. **LibTorch 编译：** 编译错误尚未解决（Torch 集成受阻）
2. **启动集成：** `rl_locomotion_a1` 节点实际上未在导航栈中启动
3. **控制器安全：** 节点崩溃/退出时无看门狗（零力矩依赖节点运行）
4. **端到端测试：** Gazebo + 导航栈尚未一起验证

### 计划功能

- [ ] 完整的 Torch 编译和运行时验证
- [ ] 模拟 MID-360 传感器插件
- [ ] LIO 与模拟传感器数据集成
- [ ] 完整导航栈启动
- [ ] SCAN Mode 3 集成
- [ ] 多层停车场导航演示

---

## 文档

### 系统设计文档

位于 `.superpowers/sdd/`：

- `2026-08-26-a1-rl-simulation-nav-design.md` — 原始设计规范
- `2026-08-28-a1-rl-simulation-implementation-status.md` — 当前实施状态（最新）

### 代码文档

- C++ 头文件包含内联 Doxygen 注释
- 启动文件包含内联 Python 文档字符串
- 配置 YAML 文件包含参数描述

### 外部资源

- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [Gazebo Classic 教程](http://classic.gazebosim.org/tutorials)
- [LibTorch C++ API](https://pytorch.org/cppdocs/)
- [Unitree A1 规格](https://www.unitree.com/products/a1/)

---

## 贡献

### 代码风格

- **C++:** clang-format（Google 风格）
- **Python:** black + isort
- **CMake:** 小写函数名，2 空格缩进
- **提交消息：** 约定式提交格式

### Pull Request 流程

1. 创建功能分支：`git checkout -b feature/你的功能名称`
2. 进行更改并使用约定式提交消息提交
3. 运行测试：`colcon test --packages-select <你的包>`
4. 推送并创建 pull request
5. 处理审查意见

### 测试要求

- 所有新功能必须包含 GTest 覆盖（C++）或 pytest（Python）
- 最低 80% 代码覆盖率
- 所有现有测试必须通过

---

## 许可证

- **Nav3D 核心：** GPL-2.0-or-later
- **Unitree A1 模型：** BSD-3-Clause（Unitree Robotics）
- **LIO：** BSD-3-Clause
- **Livox 驱动：** BSD-3-Clause

详见 LICENSE 文件。

---

## 致谢

- **Unitree Robotics** 提供 A1 机器人模型和文档
- **Livox Technology** 提供 MID-360 激光雷达驱动
- **ROS 2 社区** 提供优秀的中间件和工具
- **PyTorch 团队** 提供 LibTorch C++ API

---

## 联系方式

**维护者：** Nav3D 团队  
**邮箱：** xiaofan0100@sjtu.edu.cn  
**问题反馈：** 请通过 GitHub Issues 报告 bug 和功能请求

---

**最后更新：** 2026-08-28  
**状态：** 🟡 活跃开发中 — A1 强化学习控制基础完成，Torch 集成进行中

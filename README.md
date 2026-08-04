# Go2W 室内 3D 定位导航避障

目标：在 `/media/lenovo/disk/planner_ws/src/Nav3D/` 单 git 仓库内，参考一个已有的完整的导航栈的结构排步（本地地址为：`/media/lenovo/disk/Embodied_AI/dddmr_navigation`），深度重构 Elevator-LIO / SCAN-Planner / livox_ros_driver2 / jie_octomap 到`Nav3D/src`中。在 Gazebo 和 实机部署中跑通"建图  → 定位 → 全局规划 → 局部避障 → 仿真狗到达"的完整闭环。

包含：

建图`Nav3D/src/lio`（clone https://github.com/xiaofan4122/Elevator-LIO.git ）

全局规划`Nav3D/src/global_planner`（参考 /media/lenovo/disk/3D_Nav/jie_3d_nav 和 /media/lenovo/disk/3D_Nav/OctoPlanner3D-ROS2中的3D A*）

局部规划`Nav3D/src/local_planner`（clone https://github.com/wuyi2121/SCAN-Planner.git）

仿真基座直接使用`/media/lenovo/disk/Embodied_AI/GazeboQuadbot`

驱动:Mid360驱动使用官方的[`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2.git)

分层启动由 `bringup` 编排



仓库构建时指南：

```bash
mkdir [path/planner_ws/src]
cd [path/planner_ws/src]
git clone git@github.com:VisionTrekker/Nav3D.git
```



## Global Constraints

- **ROS 2 发行版**:仅支持 Humble (Ubuntu 22.04)
- **命名空间**:所有发布 / 订阅统一前缀 `/nav3d/`,评判在节点代码层完成还是使用launch remap 更清晰方便
- **话题命名**:统一 `/nav3d/*` 前缀,按 producer 命名空间(如 `/nav3d/lio/odom`、`/nav3d/local_planner/cmd_vel`)。ROS2 话题 token 不能以数字开头,故用 `nav3d`
- **tf frame**: `map → odom → base_link → lidar_frame`,`map ≡ odom` 静态发布
- **上游代码约定**:仅改话题字符串,**不修改算法**
- **许可证**:项目对外保留各上游原许可证
- **仿真优先**:本计划目标是先 Gazebo 闭环，验证算法后实机部署
- **提交规范**:Conventional Commits,每次可验证任务完成后必须 commit
- **colcon cmake 标志(强制)**:所有 `colcon build` 命令必须传 `--cmake-args -DHUMBLE_ROS=humble`,否则 `livox_ros_driver2` 会因 `LIVOX_INTERFACES_INCLUDE_DIRECTORIES=NOTFOUND` 失败
- **依赖项前置**:Livox-SDK2 必须已 source build 到 `/usr/local/`(非 apt、非 git submodule);`rosdep install` 可能报 apr-1 / libapr1-dev 缺失,但 optional 不阻塞







## 暂拟的 Gazebo 仿真教程

### 1. 前置依赖

```bash
# ROS 2 Humble + Gazebo 11 (Classic)
sudo apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-robot-localization ros-humble-gazebo-ros2-control

# 本工作区 build
cd /media/lenovo/disk/planner_ws
source /opt/ros/humble/setup.zsh
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DHUMBLE_ROS=humble
source install/setup.zsh
```

2. 仿真底座(GazeboQuadbot)

仿真基于外部 workspace [GazeboQuadbot](`/media/lenovo/disk/Embodied_AI/GazeboQuadbot`),提供 Go2W 模型 + **Livox Mid-360 激光仿真** + IMU + 室内场景 world。

```bash
# 终端 1: 启动 Gazebo 仿真 (Go2W + mid360 + bigHHH 室内场景)
source /opt/ros/humble/setup.zsh
source /media/lenovo/disk/Embodied_AI/GazeboQuadbot/install/setup.zsh
ros2 launch go2w_config go2w_lidar_gps.launch.py
```

启动后话题:

- `/livox/lidar` — mid360 激光 (sensor_msgs/PointCloud2, PointXYZI)
- `/livox/imu` — IMU (200 Hz)
- `/cmd_vel` — 狗运动控制入口 (teleop 按键有效)

### 3. LIO 建图(Elevator-LIO)

```bash
# 终端 2: 启动 LIO mapping 模式 (消费 /livox/*, 累积 ikd-tree 地图)
cd /media/lenovo/disk/planner_ws
source install/setup.zsh
source /media/lenovo/disk/Embodied_AI/GazeboQuadbot/install/setup.zsh
ros2 launch lio start_ros2.launch.py config_path:=root_config_sim.yaml
```

关键配置：

- `yaml/sensors/mid360_sim.yaml`:仿真专用外参(需按 GazeboQuadbot URDF 标定)
  - `lidar_type: 2`(PointCloud2 输入,非真实 Livox CustomMsg)
  - `imu_t_lidar = (-0.011, -0.02329, 0.04412)`(mid360 内置 IMU)
  - `lidar_R_body = R_y(0.4)`, `lidar_t_body = (-0.2806, 0, -0.0224)`(mid360 安装:body 前 0.25m 上 0.13m,绕 y 倾斜 0.4 rad)
- `yaml/runtime/mapping.yaml`:`global_map_pub_enable: true`(RViz 可视化全局地图)

### 4. 驱动狗运动 + 建图

```bash
# 终端 3: teleop 控制狗走动 (按 i=前进, j/l=转向, k=停)
source /opt/ros/humble/setup.zsh
source /media/lenovo/disk/Embodied_AI/GazeboQuadbot/install/setup.zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- **RViz 订阅** `/nav3d/lio/global_map` 查看实时建图
- **RViz 订阅** `/lio_node/path_topic_name` 查看定位轨迹
- 驱动狗走遍场景所有房间/走廊,LIO 实时累积地图

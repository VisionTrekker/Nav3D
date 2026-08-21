# Nav3D 规划部分开发日志

本文件用于记录规划部分的接口契约、问题清单、调整方向和每日 17:00 复盘。当前阶段只处理规划模块边界，不修改算法逻辑。

## 2026-08-21

### 当前目标

- 冻结算法边界，避免在接口未稳定前修改规划算法实现。
- 先把规划链路的接口契约写清楚，后续只按契约做 launch、yaml 或 adapter node 层面的对齐。
- 后续开发过程中，每天 17:00 记录当天基于哪些问题推进、如何解决、还剩哪些风险。

### 算法边界冻结

本阶段不修改以下算法核心：

- `src/local_planner/src/planner/bspline_opt`
- `src/local_planner/src/planner/path_searching`
- `src/local_planner/src/planner/plan_manage` 中 SCAN-Planner 的核心规划、优化、FSM 逻辑
- `src/global_planner` 中已有全局规划算法逻辑

允许调整的范围：

- launch 编排
- yaml 参数与 topic 字段
- topic remap
- adapter node
- RViz/bringup 层的边界配置
- 必要的接口契约文档和开发日志

原则：

- 不把两个规划项目揉成一个大模块。
- 全局规划和局部规划保持独立节点，通过 ROS 2 topic 契约连接。
- 算法内部先不动，稳定性问题先优先在输入输出、frame、QoS、启动顺序、adapter 层解决。

### 规划接口契约

规划主链路：

```text
/goal_pose -> global_planner -> /global_planner/path -> local_planner -> /local_planner/cmd_vel
```

| Topic | 消息类型 | frame_id | 发布者 | 订阅者 | QoS 目标 | 频率/触发 | 状态 |
|---|---|---|---|---|---|---|---|
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | `map` | `goal_marker_node` | `global_planner_node` | reliable, depth=1 | RViz 目标更新/确认时发布 | 已有 |
| `/lio/localization/odom` | `nav_msgs/msg/Odometry` | `map` 或 TF `map -> odom` 约定下的定位输出 | `lio_localization` | `global_planner_node`, `local_planner` | reliable, depth=10 | 约 50 Hz | 需联调确认 |
| `/global_planner/path` | `nav_msgs/msg/Path` | `map` | `global_planner_node` | `local_planner`, RViz | reliable, depth=1, transient_local | 新目标触发，并建议后续补 1 Hz heartbeat | 全局侧已有，局部侧待对齐 |
| `/local_planner/cmd_vel` | `geometry_msgs/msg/Twist` | 无 | `local_planner` / controller | driver, RViz | reliable, depth=10 | 约 20 Hz | 待 bringup 对齐 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 无 | bringup remap 后的 local planner 输出 | 机器人 driver | reliable, depth=10 | 约 20 Hz | 后续实机/仿真接入 |

### 当前代码观察

- `goal_marker_node` 当前发布 `/goal_pose`，`frame_id=map`。
- `global_planner_node` 当前订阅 `/goal_pose` 和 `/lio/localization/odom`，发布 `/global_planner/path`。
- `global_planner_node` 当前实现仍是直线路径 stub，不是最终 OctoMap A*。
- SCAN-Planner 局部规划在 `navi_mode=3` 时订阅 `initial_path`，消息类型是 `nav_msgs/msg/Path`。
- 因此全局规划和局部规划的数据类型基本一致，但 topic 名称与启动模式仍需要在边界层对齐。

### 当前问题清单

#### P1. 规划链路目前更像仓库合并，不是完整稳定集成

- 现象：
  - 全局规划和局部规划都在 workspace 内。
  - 两者之间缺少稳定的启动层连接契约。
- 影响：
  - 只启动 bringup 时，不能保证 `/global_planner/path` 被局部规划消费。
- 调整方向：
  - 先通过 launch remap 或 adapter node 把 `/global_planner/path` 对齐到局部规划的 `initial_path`。

#### P2. 局部规划默认模式不是全局路径跟踪模式

- 现象：
  - SCAN-Planner 默认 `fsm.navi_mode=1`，主要等待 RViz 手动目标。
  - 设计方案要求局部规划跟踪 `/global_planner/path`。
- 影响：
  - 即使全局规划发布了 path，局部规划也可能不进入参考路径模式。
- 调整方向：
  - 在规划集成 launch 或 yaml 中明确设置 `fsm.navi_mode=3`。

#### P3. 局部规划输出 cmd_vel 需要明确由哪个节点发布

- 现象：
  - SCAN-Planner 的规划节点发布 `planning/bspline`。
  - `closed_loop_controller` 才发布 `cmd_vel`。
- 影响：
  - 只启动 `scan_planner_node` 不等于已经有 `/local_planner/cmd_vel`。
- 调整方向：
  - 规划集成 launch 中同时启动 planner 和 controller，controller 输出 remap 到 `/local_planner/cmd_vel`。

#### P4. frame 约定需要在接口层明确

- 现象：
  - 设计方案规划链路以 `map` 为全局 frame。
  - SCAN-Planner 原始配置存在 `world` frame 习惯。
- 影响：
  - 如果 path、odom、局部地图 frame 不一致，后续会出现看似算法失败的接口问题。
- 调整方向：
  - 先在契约中规定 `/goal_pose`、`/global_planner/path` 使用 `map`。
  - 后续通过 TF 或 adapter 层统一到 SCAN-Planner 可接受的 frame。

### 下一步建议

1. 新建或整理一个只负责规划集成的 launch，例如 `planner_bridge.launch.py` 或在 bringup 中增加明确的规划段。
2. 在 launch/yaml 层设置局部规划 `fsm.navi_mode=3`。
3. 将局部规划输入 `initial_path` remap 到 `/global_planner/path`。
4. 同时启动局部 controller，并将 `cmd_vel` remap 到 `/local_planner/cmd_vel`。
5. 用 mock odom 和简单 path 做最小闭环验证，确认 topic 能串起来。
6. 规划接口稳定后，再做真实地图、定位、OctoMap A* 和 E2E 测试。

### 第二阶段：只验证 global_planner

验证范围：

- 不制作新地图。
- 不修改建图算法。
- 不修改当前全局规划路径生成算法。
- 只验证已有 PCD 地图参数、当前位姿、`/goal_pose` 是否能触发 `/global_planner/path`。

地图输入约定：

- 使用项目现有地图：`/home/nhy/code/vscode/maps/zhiyuan_rev.pcd`
- 地图路径只能通过 launch/yaml 参数传入，不硬编码到 C++ 源码。

新增边界配置：

- `src/global_planner/config/global_planner.yaml`
- `src/global_planner/launch/validate_global_planner.launch.py`

当前注意事项：

- 当前 `global_planner_node` 仍是直线 stub；本阶段只做参数入口和触发链路验证。
- 2026-08-21 已确认本机地图文件：`/home/nhy/code/vscode/maps/zhiyuan_rev.pcd`

### 第二阶段补充：接入 OctoPlanner3D 真实后端

问题：

- `/lio/localization/odom + /goal_pose -> /global_planner/path` 已能发布 `nav_msgs/msg/Path`，但路径点按固定比例递增，确认来自直线插值 stub。
- `zhiyuan_rev.pcd` 此前只做路径参数存在性检查，没有真正进入 PCD -> OctoMap -> A* 规划链路。

处理边界：

- 只修改 `src/global_planner` 及其必要 CMake/package 依赖。
- 不修改 OctoPlanner3D A* 核心算法。
- `global_planner_node` 只保留 ROS2 adapter 职责：订阅 odom/goal，调用后端，发布 `nav_msgs/msg/Path`。

已完成调整：

- 复用本机 `/home/nhy/code/vscode/OctoPlanner3D-ROS2` 中的 `planner/include/global_planner.h`、`planner/src/global_planner.cpp` 和 PCD -> OctoMap converter。
- 删除当前直线插值 stub，改为：
  `PCD -> OctoMap -> GlobalPlanner::setOctomap() -> makePlan(odom, goal) -> getPlannerResults() -> /global_planner/path`
- 保持接口不变：
  `/lio/localization/odom`、`/goal_pose`、`/global_planner/path`，`frame_id=map`。
- 增加日志：`PCD loaded`、`OctoMap ready`、`odom received`、`goal received`、`planning started`、`planning success/failed`、`path point count`。

验证结果：

- `colcon build --packages-select global_planner` 通过。
- validate launch 已确认加载 `3333771` 个 PCD 点，OctoMap `resolution=0.200`，`leaf_nodes=245944`。
- 测试 `(0,0,0) -> (5,3,1)` 时，A* 找到路径：`661` iterations，`24` waypoints。
- `/global_planner/path` 输出 `frame_id=map`，点列来自 A* 结果，不再是原来的 20 段直线插值。

当前注意事项：

- 大地图初始化耗时约 50 秒左右，主要在 PCD 转换和 OctoPlanner 派生层预处理。
- converter 会把临时 OctoMap 输出到 `/tmp/nav3d_global_planner_zhiyuan_rev.bt`，不污染项目目录。

### 第三阶段：local_planner / SCAN Mode 3 独立验证

验证范围：

- 不连接 `/global_planner/path`。
- 不修改 `bspline_opt`、`path_searching`、FSM planning core。
- 只通过验证 launch 和 mock I/O 节点测试 Mode 3 链路。

当前实际接口：

- `body_pose`: `nav_msgs/msg/Odometry`，测试 remap 到 `/scan_mode3/body_pose`，frame=`world`。
- `sensor_pose`: `nav_msgs/msg/Odometry`，测试 remap 到 `/scan_mode3/sensor_pose`，frame=`world`。
- `cloud`: `sensor_msgs/msg/PointCloud2`，测试 remap 到 `/scan_mode3/cloud`，frame=`world`。
- `initial_path`: `nav_msgs/msg/Path`，测试 remap 到 `/scan_mode3/initial_path`，frame=`world`。
- `planning/bspline`: `scan_planner_msgs/msg/Bspline`，测试输出 `/planning/bspline`。
- `cmd_vel`: `geometry_msgs/msg/Twist`，测试 remap 到 `/scan_mode3/cmd_vel`。

验证结果：

- `fsm.navi_mode=3`。
- mock 节点先持续发布 `body_pose/sensor_pose/cloud`，再发布 `initial_path`。
- SCAN 日志出现 `Reference path accepted`。
- `planning/bspline` 输出 1 条轨迹，`traj_id=1`，control points=30。
- `closed_loop_controller` 收到轨迹并发布 `cmd_vel`。
- 测试 launch 可自动退出，命令返回 0。

z 语义：

- Mode 3 当前会在 `prepareReferenceWaypoints()` 内对每个 `initial_path` 点执行 `z + grid_map.body_height`。
- 当前 `grid_map.body_height=0.4`，因此 `initial_path` 的 z 表示路线/地面高度，SCAN 内部目标机身参考点高度会额外加 0.4 m。

### 第四阶段：接入 `/global_planner/path` 到 SCAN Mode 3

问题：

- `global_planner` 输出的 `Path.z` 已经是机器人参考点/可通行体素中心高度。
- SCAN Mode 3 默认会对 incoming `initial_path.z` 再加 `grid_map.body_height`，如果直接 remap 会重复抬高 z。

处理方式：

- 不修改 SCAN 核心规划算法。
- 在接入验证 launch 中把 `grid_map.body_height` 覆盖为 `0.0`。
- 直接 remap `initial_path := /global_planner/path`。
- SCAN frame 覆盖为 `map`，与 global planner path 保持一致。
- mock 节点持续发布 `/lio/localization/odom` 和 cloud，且在收到 global path 前周期重发 `/goal_pose`，避免 global planner 地图初始化期间丢 goal。

验证结果：

- `/global_planner/path` 输出 `24` 个点，frame=`map`，首点 z=`0.3`。
- SCAN 日志出现 `Reference path accepted`。
- SCAN 规划目标 z=`0.3`，没有被额外加到 `0.7`。
- `/planning/bspline` 输出 1 条轨迹，`traj_id=1`，control points=`23`。
- `closed_loop_controller` 收到轨迹后发布 `/global_to_scan/cmd_vel`。
- 测试 launch 自动退出，命令返回 0。

### 第五阶段：规划子系统正式 launch 收尾

正式入口：

- `bringup/bringup/planning.launch.py`
- 参数集中在 `bringup/config/planning.yaml`

正式链路：

```text
/goal_pose
-> global_planner_node
-> /global_planner/path
-> scan_planner_node Mode 3
-> /planning/bspline
-> closed_loop_controller
-> /local_planner/cmd_vel
```

安全边界：

- 新 goal 到达时，`planning_safety_supervisor` 先发布 `/planning/stop_requested=true`，controller 清空旧轨迹并输出 0。
- 收到该 goal 之后的新 `/global_planner/path` 后，supervisor 发布 `/planning/stop_requested=false`，允许新 B-spline 接管。
- 如果不可达 goal 没有产生 fresh global path，3 秒后保持 stop，不复用旧轨迹。

验收结果：

- 正式 launch 可启动 `global_planner_node`、`scan_planner_node`、`closed_loop_controller`、`planning_safety_supervisor`。
- 第一目标 `(5,3,1)`：global path=24 点，B-spline traj_id=1，control_points=23，cmd_vel 有输出。
- 第二目标 `(1,1,0.3)`：global path=6 点，B-spline traj_id=2，control_points=14，重规划通过。
- 不可达目标 `(100,100,10)`：global planner 报地图外失败；supervisor 超时保持 stop；controller 清空旧轨迹，不发布新 B-spline。
- 当前唯一剩余问题：PCD -> OctoMap 初始化约 50 秒，暂不优化，只记录为性能问题。

---

## 每日 17:00 复盘模板

```md
## YYYY-MM-DD 17:00 复盘

### 今天基于哪些问题推进

- 

### 今天做了哪些调整

- 

### 如何验证

- 

### 已解决的问题

- 

### 未解决的问题

- 

### 风险和注意事项

- 

### 明天优先级

- 
```

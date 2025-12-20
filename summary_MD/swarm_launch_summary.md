# `swarm.launch` 启动总览

## 启动链路
- 入口：`src/planner/plan_manage/launch/swarm.launch`
- 直接节点：`map_generator/random_forest`（全局随机森林障碍物地图）
- 批量 include（10 架机，drone_id 0..9）：`run_in_sim.launch`
  - 每个 `run_in_sim` 内 include：`advanced_param.xml`（主要规划节点与参数）、`simulator.xml`（仿真/传感器/可视化）、并启动 `traj_server`

## 主要节点与话题（单机命名约定：`drone_<id>_<node_name>`）
注意：本项目的节点并非运行在 `drone_<id>` 的 ROS namespace 下（即 `/drone_0/node`），而是通过在节点名称和话题前添加 `drone_<id>_` 前缀来进行隔离（例如节点名为 `/drone_0_traj_server`）。

- 地图生成：`random_forest`
  - 发布全局点云 `map_generator/global_cloud`
- 规划节点：`drone_<id>_ego_planner_node`（来自 `advanced_param.xml`）
  - 订阅：`/drone_<id>_visual_slam/odom`（里程计）、`/drone_<id>_pcl_render_node/cloud` 或 depth/pose 话题
  - 发布：`/drone_<id>_planning/bspline`（轨迹）、`/drone_<id>_planning/data_display`，同时广播/接收 `/broadcast_bspline`
  - 参数：速度/加速度/轨迹优化、局部栅格地图参数、全局路点等
- 轨迹服务器：`drone_<id>_traj_server`
  - 订阅：`drone_<id>_planning/bspline`
  - 发布：`drone_<id>_planning/pos_cmd`（`quadrotor_msgs/PositionCommand`）
    > **消息结构详解 (`quadrotor_msgs/PositionCommand`)**：
    > 该消息是控制层的核心接口，定义了无人机在特定时刻的期望状态。
    > - `Header header`: 标准 ROS 头，包含时间戳和坐标系（通常为 `world`）。
    > - `Point position`: 期望位置 (x, y, z)。
    > - `Vector3 velocity`: 期望速度 (vx, vy, vz)。
    > - `Vector3 acceleration`: 期望加速度 (ax, ay, az)，用于前馈控制。
    > - `float64 yaw`: 期望偏航角（弧度）。
    > - `float64 yaw_dot`: 期望偏航角速度。
    > - `float64[3] kx`, `kv`: 位置与速度控制增益，允许上层规划器动态调整控制器的刚度。
    > - `uint32 trajectory_id`: 轨迹唯一标识符，用于区分不同的轨迹段。
    > - `uint8 trajectory_flag`: 轨迹状态标志位（如 `READY=1`, `COMPLETED=3`, `ABORT=4` 等），用于状态机管理。
- 简易动力学仿真：`drone_<id>_poscmd_2_odom`（在 `simulator.xml`）
  - 订阅：`drone_<id>_planning/pos_cmd`
  - 发布：`drone_<id>_visual_slam/odom`（供规划与可视化）
- 传感器模拟：`drone_<id>_pcl_render_node`
  - 订阅：`map_gen在 `swarm.launch` 中为每个 drone 设置 `init_(x,y,z)` 与 `target_(x,y,z)`（轨迹起点/全局路点）
- 规划/优化/感知参数：详见 `advanced_param.xml`（规划时域、速度/加速度约束、局部地图大小、深度滤波等）
- 传感器仿真参数：`simulator.xml` 中的 `pcl_render_node`、`poscmd_2_odom` 初始高度/频率等

## 新增功能：Swarm Circle Demo (绕圈演示)
为测试基本的 swarm 控制与可视化，新增了 `swarm_circle_demo.launch`。
- **功能**：启动 3 架无人机（红、绿、蓝），在空地图中绕圆心进行匀速圆周运动。
- **启动方式**：`roslaunch ego_planner swarm_circle_demo.launch`
- **主要组件**：
  - `swarm_circle_demo.py`：Python 脚本，计算圆周轨迹并发布 `PositionCommand`。
  - `poscmd_2_odom`：为每架飞机模拟动力学。
  - `odom_visualization`：在 RViz 中显示飞机模型。
  - `rviz`：预加载可视化配置。
- **注意**：该 Demo 不使用 `ego_planner_node` 进行规划，而是直接发送控制指令，用于验证仿真器和可视化链路。

## 启动顺序与依赖
1) `map_generator/random_forest` 创建全局障碍物点云
2) 对每个 `drone_id`：
   - 启动 `ego_planner_node`（规划+地图）
   - 启动 `traj_server`（Bspline → PositionCommand）
   - 启动 `poscmd_2_odom`（PositionCommand → Odom）
   - 启动 `pcl_render_node`（里程计 + 地图 → 点云/深度）
   - 启动 `odom_visualization`（可视化用）
3) 全局轨迹广播通道 `/broadcast_bspline` 用于多机协同

## 关键信息流（单机）
- `pcl_render_node` 输出点云/深度 → `ego_planner_node` → 生成 `bspline`
- `bspline` → `traj_server` → `pos_cmd` → `poscmd_2_odom` → `visual_slam/odom`
- `visual_slam/odom` 同时回馈 `ego_planner_node` 与 `odom_visualization`
- 多机：`/broadcast_bspline` 汇聚/分发多机轨迹erator/global_cloud`、`drone_<id>_visual_slam/odom`
  - 发布：`/drone_<id>_pcl_render_node/cloud`、相机姿态/深度（按参数 remap）
- 可视化：`drone_<id>_odom_visualization`
  - 订阅：`drone_<id>_visual_slam/odom`
  - 发布：RViz marker/TF（颜色等参数可调）
- 其他（可选/注释）：`so3_control`、`quadrotor_simulator_so3` 等在 `simulator.xml` 中被注释掉

## 话题与重映射关系（单机示例）
- 里程计：`~odom_world` → `/drone_<id>_visual_slam/odom`
- 规划轨迹：`~planning/bspline` → `/drone_<id>_planning/bspline`
- 轨迹广播：`planning/broadcast_bspline_from_planner` ↔ `/broadcast_bspline`（全局共享）
- 位置指令：`position_cmd` → `/drone_<id>_planning/pos_cmd`
- 传感输入：`~grid_map/cloud` → `/drone_<id>_pcl_render_node/cloud`；或 depth/pose 话题按参数 remap
- 地图输入：`~global_map` → `/map_generator/global_cloud`

## 参数与环境
- 地图尺寸（入口 args）：`map_size_x=42.0, map_size_y=30.0, map_size_z=5.0`
- 单机初始位姿/目标：在 `swarm.launch` 中为每个 drone 设置 `init_(x,y,z)` 与 `target_(x,y,z)`（轨迹起点/全局路点）
- 规划/优化/感知参数：详见 `advanced_param.xml`（规划时域、速度/加速度约束、局部地图大小、深度滤波等）
- 传感器仿真参数：`simulator.xml` 中的 `pcl_render_node`、`poscmd_2_odom` 初始高度/频率等

## 新增功能：Swarm Circle Demo (绕圈演示)
为测试基本的 swarm 控制与可视化，新增了 `swarm_circle_demo.launch`。
- **功能**：启动 3 架无人机（红、绿、蓝），在空地图中绕圆心进行匀速圆周运动。
- **启动方式**：`roslaunch ego_planner swarm_circle_demo.launch`
- **主要组件**：
  - `swarm_circle_demo.py`：Python 脚本，计算圆周轨迹并发布 `PositionCommand`。
  - `poscmd_2_odom`：为每架飞机模拟动力学。
  - `odom_visualization`：在 RViz 中显示飞机模型。
  - `rviz`：预加载可视化配置。
- **注意**：该 Demo 不使用 `ego_planner_node` 进行规划，而是直接发送控制指令，用于验证仿真器和可视化链路。

## 启动顺序与依赖
1) `map_generator/random_forest` 创建全局障碍物点云
2) 对每个 `drone_id`：
   - 启动 `ego_planner_node`（规划+地图）
   - 启动 `traj_server`（Bspline → PositionCommand）
   - 启动 `poscmd_2_odom`（PositionCommand → Odom）
   - 启动 `pcl_render_node`（里程计 + 地图 → 点云/深度）
   - 启动 `odom_visualization`（可视化用）
3) 全局轨迹广播通道 `/broadcast_bspline` 用于多机协同

## 关键信息流（单机）
- `pcl_render_node` 输出点云/深度 → `ego_planner_node` → 生成 `bspline`
- `bspline` → `traj_server` → `pos_cmd` → `poscmd_2_odom` → `visual_slam/odom`
- `visual_slam/odom` 同时回馈 `ego_planner_node` 与 `odom_visualization`
- 多机：`/broadcast_bspline` 汇聚/分发多机轨迹

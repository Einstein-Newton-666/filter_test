# filter_test - ROS2 自瞄仿真与 typed GTSAM 图优化工作区

本仓库用于验证三类自瞄目标状态估计方案：

- 装甲板仿真 + 装甲板 typed GTSAM 图优化。
- 能量机关仿真 + `RuneCvGraph` 图优化 + 大符曲线拟合。
- 传统 EKF/UKF 滤波器，用作对照路径。

当前最重要的说明文档是
[docs/current_project_architecture.html](docs/current_project_architecture.html)。README 只保留入口信息、运行命令和当前状态；计算原理、公式和模块边界放在网页里。

## 当前状态

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| 装甲板仿真 | 可用 | 运动积分、标准四装甲板、像素噪声、PnP、marker/image 调试。 |
| 前哨站模型 | 可用 | `mode: outpost` 三装甲板仿真；传统滤波和 typed graph 均支持三槽位、120 度、独立 `dz_1/dz_2`。 |
| 能量机关仿真 | 可用 | 小符每次 1 片随机扇叶；大符每次 2 片随机扇叶；3 秒切换；`RuneTargets` 只包含观测到的五点扇叶，不发布物理编号。 |
| 装甲板图优化 | 主链路 | typed GTSAM，角点像素因子 + 几何因子 + auto_aim 风格边缘重投影因子。 |
| 滤波前端 + 图优化后端 | 可选 | `filter_graph_optimizer` 近似同步 `/detector/armors` 和 `/track_result`，用滤波结果作为图优化初值/弱先验。 |
| 能量机关图优化 | 可用 | 5 点 PnP prior、JLU 风格辅助 `Pose3`、auto_aim 风格直接重投影、大符 Ceres 曲线拟合。 |
| 传统滤波 | 保留 | EKF/UKF + CV/Singer，用于对照和旧链路。 |

## 一眼看懂链路

装甲板纯图优化链路：

```text
armor_simulation_node
  -> /detector/armors
  -> graph_optimizer_test
  -> ArmorGraphTracker
  -> ArmorCvPixelGraph
  -> auto_graph::GraphOptimizer
  -> GTSAM
  -> /tracker/target, /graph_optimizer/marker
```

滤波前端 + 图优化后端链路：

```text
armor_simulation_node
  -> /detector/armors
  -> filter -> /track_result
  -> filter_graph_optimizer
  -> ArmorGraphTracker / ArmorCvPixelGraph
  -> GTSAM
  -> /tracker/target, /graph_optimizer/marker
```

能量机关链路：

```text
rune_simulation_node
  -> /rune_detector/rune_targets  # 紧凑五点观测数组，不带物理扇叶编号
  -> /rune_simulation/ground_truth, /rune_simulation/marker
  -> rune_graph_optimizer
  -> RuneCvGraph
  -> auto_graph::GraphOptimizer
  -> GTSAM
  -> /rune_graph_optimizer/rune_info, /rune_graph_optimizer/target_pose
```

相机外参链路：

```text
gimbal_simulation
  -> TF: odom -> gimbal_link -> camera_link -> camera_optical_frame
  -> 仿真器和图优化节点运行时 lookupTransform("odom", "camera_optical_frame")
```

## 包结构

| 包 | 作用 |
| --- | --- |
| `auto_aim_interfaces` | 共享 ROS 消息：`Armors`、`TrackerTarget`、`RuneTarget(s)`、`RuneInfo` 等。 |
| `armor_simulation` | 装甲板/前哨站/能量机关仿真，相机投影，PnP，云台 TF，角度解算。 |
| `filter_test` | 传统滤波器、装甲板图优化、能量机关图优化、ROS 节点和测试。 |

关键文件：

```text
src/armor_simulation/
  include/armor_simulation/armor_geometry.hpp    # 装甲板角点
  include/armor_simulation/rune_geometry.hpp     # 能量机关/前哨站几何
  include/armor_simulation/camera_model.hpp      # 投影、畸变、PnP
  include/armor_simulation/detection_noise.hpp   # 像素噪声、掉检
  src/armor_simulation.cpp                       # 装甲板/前哨站仿真节点
  src/rune_simulation.cpp                        # 能量机关仿真节点
  src/gimbal_simulation.cpp                      # 云台动力学和 TF

src/filter_test/
  include/filter_test/filter.hpp                  # ArmorFilter 对外接口
  include/filter_test/filter_test.hpp             # 传统滤波 ROS 节点外壳
  include/filter_test/filters/                    # EKF/UKF、CV/Singer、滤波工具
  include/filter_test/ros_utils/                  # 相机信息、图优化节点、滤波前端适配工具
  include/filter_test/graph_optimizer/graph_core.hpp
  include/filter_test/graph_optimizer/armor_model.hpp
  include/filter_test/graph_optimizer/rune_model.hpp
  src/filter_test.cpp                             # filter 节点入口
  src/filter_graph_optimizer.cpp                  # 滤波器前端 + 图优化后端节点
  src/filters/filter.cpp                          # ArmorFilter 业务实现
  src/graph_optimizer/graph_core.cpp
  src/graph_optimizer/armor_model.cpp
  src/graph_optimizer/rune_model.cpp
  src/ros_utils/camera_info_utils.cpp
  src/ros_utils/graph_optimizer_node_utils.cpp
  src/graph_optimizer_test.cpp
  src/rune_graph_optimizer.cpp
```

## 快速运行

环境：Ubuntu 22.04、ROS2 Humble、C++17、GTSAM 4.3。当前 GTSAM 在 `/usr/local/lib`：

```bash
source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

低并发构建，适合内存紧张机器：

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --parallel-workers 1 \
  --packages-select auto_aim_interfaces armor_simulation filter_test \
  --cmake-args -DBUILD_TESTING=ON
```

当前 launch 文件会读取 `config.yaml` 顶层 `tracker_backend` 作为默认后端：

- `graph`：纯图优化后端。
- `filter_graph`：传统滤波器前端 + 图优化后端。

注意：当前 `filter_test.launch.py` 的返回列表实际启用仿真、云台和 `filter` 节点；
`graph_optimizer_test`、`filter_graph_optimizer`、`angle_solver`、能量机关和 jlu 节点仍以注释形式保留。
若要用 launch 一次拉起完整后端链路，需要先确认对应节点已加入 `LaunchDescription`；
单独调试时也可以直接用 `ros2 run` 启动目标节点。

```bash
ros2 launch filter_test filter_test.launch.py
ros2 launch filter_test filter_test.launch.py tracker_backend:=graph
ros2 launch filter_test filter_test.launch.py tracker_backend:=filter_graph
```

单独运行常用节点：

```bash
ros2 run armor_simulation gimbal_simulation \
  --ros-args --params-file src/filter_test/config/config.yaml

ros2 run armor_simulation armor_simulation_node \
  --ros-args --params-file src/armor_simulation/config/simulation_config.yaml

ros2 run armor_simulation rune_simulation_node \
  --ros-args --params-file src/armor_simulation/config/simulation_config.yaml

ros2 run filter_test graph_optimizer_test \
  --ros-args --params-file src/filter_test/config/config.yaml

ros2 run filter_test filter \
  --ros-args --params-file src/filter_test/config/config.yaml

ros2 run filter_test filter_graph_optimizer \
  --ros-args --params-file src/filter_test/config/config.yaml

ros2 run filter_test rune_graph_optimizer \
  --ros-args --params-file src/filter_test/config/config.yaml
```

## 常用测试

装甲板和能量机关图优化核心测试：

```bash
LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH colcon test \
  --packages-select filter_test \
  --ctest-args -R test_auto_graph_optimizer --output-on-failure
```

仿真、噪声、PnP、rune 几何和前哨站几何测试：

```bash
colcon test --packages-select armor_simulation \
  --ctest-args -R test_noise_model --output-on-failure
```

完整 `colcon test --packages-select filter_test` 会跑 ament lint。当前仓库仍有既有 package metadata / 格式类问题，回归优先看上面的 gtest。

## 配置入口

优先改 YAML，不要直接改头文件默认值。

| 文件 | 主要节点 |
| --- | --- |
| `src/armor_simulation/config/simulation_config.yaml` | `/armor_simulation_node`、`/rune_simulation_node` |
| `src/filter_test/config/config.yaml` | `/filter`、`/graph_optimizer_test`、`/filter_graph_optimizer`、`/rune_graph_optimizer`、`/gimbal_simulation` |

常改参数：

- `armor_simulation_node.mode`: `standard` 或 `outpost`。
- `outpost.radius/dz_1/dz_2/armor_pitch`: 前哨站几何。
- `camera_info_url`、`camera_name`: 单一相机内参来源；默认指向 `armor_simulation/config/camera_info.yaml`。
- `tracker_backend`: launch 后端选择，`graph` 为纯图优化，`filter_graph` 为滤波前端 + 图优化后端。
- `pixel_noise_*`、`pixel_noise_common_ratio`: 像素噪声。
- `publish_gimbal_gt`: 全局唯一开关；仿真器直接向 `/send_pack` 发布真值 yaw/pitch，armor 瞄准机器人中心，rune 优先瞄准当前观测扇叶，其次 active blade 的击打中心。
- `rune_simulation_node.rune_mode`: `small` 或 `big`。
- `rune_simulation_node.blade_switch_interval`: 默认 3 秒；小符 1 片、大符 2 片随机 active blade 切换。
- `/rune_detector/rune_targets`: 只发布当前观测到的 active blade；物理 slot 由图优化内部 PnP+roll 匹配推断。
- `graph_optimizer_test.standard.pixel_noise_std`、`outpost.pixel_noise_std`、`standard.geo.*`、`outpost.geo.*`、`vel_sigma`、`vyaw_sigma`: 装甲板图优化基础噪声。
- `graph_optimizer_test.use_edge_reproj_factor`、`edge_reproj_sigma`、`edge_loss_slope_k`: auto_aim EdgeLoss 风格四边重投影约束。
- `graph_optimizer_test.standard.armor_pitch`、`outpost.armor_pitch`: direct edge 重投影使用的装甲板 pitch。
- `graph_optimizer_test.outpost.radius/dz_1/dz_2`: typed graph 前哨站静态几何先验。
- `graph_optimizer_test.frontend.*`: `filter_graph_optimizer` 使用的同步窗口和滤波器弱先验参数。
- `rune_graph_optimizer.initial_center`、`normal_yaw`、`normal_pitch`、`predict_dt`: 能量机关图优化初值、平面姿态和预测。
- `rune_graph_optimizer.use_direct_reproj_factor`、`direct_reproj_sigma`、`pnp_pose_prior_sigma`: 能量机关 PnP/直接重投影约束。
- `rune_graph_optimizer.match_max_roll_diff`、`match_max_center_distance`: 无编号观测匹配到 5 个物理 slot 的门限。
- `rune_graph_optimizer.big_fitter.*`: 大符 5 参数曲线拟合窗口和迭代次数。

## 原理入口

建议阅读顺序：

1. [当前工程架构总览](docs/current_project_architecture.html)：当前真实代码、仿真计算原理、图优化残差、TF 外参。
2. [图优化架构](docs/graph_optimizer_architecture.html)：装甲板 typed graph 的历史设计和细节。
3. [GTSAM 使用指南](docs/gtsam_graph_optimizer_guide.html)：GTSAM/iSAM2/fixed-lag 相关说明。
4. [因子图原理](docs/factor_graph_optimization.html)：因子图背景知识。
5. `.claude/skills/armor-graph-optimizer/SKILL.md`：给 agent 修改图优化代码时使用的约束。

## 已知边界

- 前哨站对外仍复用旧 `TrackerTarget.enemy.radius_1/radius_2/dz` 字段；typed graph 内部语义为
  `radius_1=radius`、`radius_2=radius`、`dz=dz_1`，另有内部 `outpost_dz_2`。
- 能量机关平面 pitch 仍是固定参数；当前只估计 `normal_yaw`，不估计 pitch。仿真器会把 `normal_yaw/normal_pitch`
  写入 `RuneGroundTruth`、`RuneInfo` 和 marker/target pose orientation。
- 装甲板匹配仍是启发式代价，不是马氏距离。
- `radius_2` 仍可能过冲，需要继续调参。
- `filter_test` 和 `auto_aim_interfaces` 的 `package.xml` 仍保留 TODO license/description。

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

ROS2 **Humble** / C++17 / GCC 11 工作区，用于实时装甲板跟踪实验。核心方向是传统
EKF/UKF 滤波和 GTSAM typed 因子图优化；当前分支也在接入能量机关图优化和前哨站三装甲板模型。
`src/` 下有三个 package：

1. `armor_simulation`：3D 运动仿真、装甲板/能量机关/前哨站几何、相机投影、像素噪声、IPPE PnP、云台 TF、角度解算。
2. `filter_test`：传统滤波器、typed GTSAM 装甲板图优化器、`RuneCvGraph` 能量机关图模型、jlu tracker 适配层、ROS 节点、配置和测试。
3. `auto_aim_interfaces`：共享自瞄消息接口，包括 `Point2d`、`RuneTarget(s)`、`RuneInfo`。

优先阅读：

- `README.md` — 面向人类使用者的完整中文说明。
- `AGENTS.md` — 给 agent 使用的高信噪比摘要。
- `.claude/skills/armor-filter/SKILL.md` — 传统滤波器专项知识。
- `.claude/skills/armor-graph-optimizer/SKILL.md` — 图优化专项知识。
- `.claude/skills/armor-simulator/SKILL.md` — 仿真器专项知识。

## 构建命令

```bash
# 基础构建
colcon build --packages-select auto_aim_interfaces armor_simulation filter_test

# 构建并启用测试
colcon build --packages-select auto_aim_interfaces armor_simulation filter_test --cmake-args -DBUILD_TESTING=ON

# 本机内存紧张时用低并发构建
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --parallel-workers 1 \
  --packages-select auto_aim_interfaces armor_simulation filter_test \
  --cmake-args -DBUILD_TESTING=ON

# 环境设置
source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH  # GTSAM 4.3 在 /usr/local/lib

# 运行 filter_test 测试
colcon test --packages-select filter_test
colcon test-result --verbose

# 运行单个测试
colcon test --packages-select filter_test --ctest-args -R "filter_test" --output-on-failure
colcon test --packages-select filter_test --ctest-args -R "test_auto_graph_optimizer" --output-on-failure

# 仿真器噪声/PnP 测试
colcon test --packages-select armor_simulation --ctest-args -R "test_noise_model" --output-on-failure
```

测试入口：

- `src/filter_test/test/filter_test.cpp`
- `src/filter_test/test/test_auto_graph_optimizer.cpp`
- `src/armor_simulation/test/test_noise_model.cpp`

## 运行方式

```bash
# 当前默认只启动 graph_optimizer_test
ros2 launch filter_test filter_test.launch.py

# 仅仿真器
ros2 launch armor_simulation simulation.launch.py

# 单独节点
ros2 run armor_simulation armor_simulation_node \
  --ros-args --params-file src/armor_simulation/config/simulation_config.yaml
ros2 run armor_simulation gimbal_simulation \
  --ros-args --params-file src/filter_test/config/config.yaml
ros2 run armor_simulation angle_solver
ros2 run filter_test filter \
  --ros-args --params-file src/filter_test/config/config.yaml
ros2 run filter_test graph_optimizer_test \
  --ros-args --params-file src/filter_test/config/config.yaml
ros2 run filter_test jlu_tracker \
  --ros-args --params-file src/filter_test/config/config.yaml
```

`filter_test.launch.py` 已定义仿真器、云台、angle_solver、传统滤波器、图优化器和
jlu tracker，但当前默认只启用 `graph_optimizer_test`。完整闭环需要手动打开对应节点。

## 项目结构

```text
src/
├── armor_simulation/
│   ├── include/armor_simulation/
│   │   ├── armor_simulation.hpp      # 主仿真节点声明
│   │   ├── camera_model.hpp          # 投影、畸变、IPPE PnP、外参
│   │   ├── armor_geometry.hpp        # 装甲板角点几何
│   │   ├── rune_geometry.hpp         # 能量机关和前哨站 typed 几何 helper
│   │   ├── detection_noise.hpp       # 像素噪声和检测概率
│   │   └── pnp_pose_utils.hpp        # PnP RPY 提取和双解 yaw 修正
│   ├── src/
│   │   ├── armor_simulation.cpp      # 运动、投影、噪声、PnP、marker、image
│   │   ├── gimbal_simulation.cpp     # S-curve 云台、TF、/recieve_pack
│   │   └── angle_solver.cpp          # /tracker/target -> /send_pack
│   ├── config/simulation_config.yaml
│   └── msg/GroundTruth.msg
│
├── filter_test/
│   ├── include/filter_test/
│   │   ├── filters/
│   │   │   ├── kalman_filter.hpp
│   │   │   ├── extended_kalman.hpp
│   │   │   ├── unscented_kalman.hpp
│   │   │   ├── cv_model.hpp
│   │   │   └── singer_model.hpp
│   │   ├── graph_optimizer/
│   │   │   ├── graph_core.hpp        # auto_graph::GraphOptimizer, Var<T>, SolveResult
│   │   │   ├── graph_math.hpp        # 角度、logistic、Eigen/GTSAM helper
│   │   │   ├── armor_model.hpp       # 因子、匹配、状态、ArmorCvPixelGraph
│   │   │   ├── armor_tracker.hpp     # ArmorGraphTracker, FrameTimeTracker
│   │   │   └── rune_model.hpp        # RuneCvGraph, 能量机关 factors/state
│   │   ├── jlu_tracker/              # jlu_vision_26 tracker 适配层
│   │   ├── filter.hpp                # ArmorFilter 业务逻辑
│   │   ├── filter_test.hpp           # ArmorTest ROS 节点
│   │   ├── graph_optimizer_test.hpp  # GraphOptimizerTest ROS 节点外壳
│   │   └── common.hpp
│   ├── src/
│   │   ├── filter.cpp
│   │   ├── filter_test.cpp
│   │   ├── graph_optimizer/
│   │   │   ├── graph_core.cpp
│   │   │   ├── armor_model.cpp
│   │   │   ├── armor_tracker.cpp
│   │   │   └── rune_model.cpp
│   │   ├── graph_optimizer_test.cpp
│   │   └── jlu_tracker_node.cpp
│   ├── config/config.yaml
│   └── launch/filter_test.launch.py
│
└── auto_aim_interfaces/
    └── msg/
```

## 传统滤波器

主要文件：

- `filters/kalman_filter.hpp`：模板 KF 基类。
- `filters/extended_kalman.hpp`：动态维度 EKF，Ceres Jet 自动微分雅可比。
- `filters/unscented_kalman.hpp`：动态维度 UKF。
- `filters/cv_model.hpp`：CV 11D 预测和观测模型。
- `filters/singer_model.hpp`：Singer 15D 预测和观测模型。
- `filter.hpp` / `filter.cpp`：`ArmorFilter` 业务流程。

传统路径：

```text
/detector/armors
  -> ArmorTest
  -> ArmorFilter
  -> EKF 或 UKF
  -> CV 或 Singer
  -> /track_result, /track_result/marker
```

`ArmorFilter::update()` 主要流程：

1. 空观测直接返回上一状态。
2. 根据消息时间戳计算 `dt`。
3. 用预测状态做装甲板匹配。
4. 将观测 xyz 转为 yaw/pitch/distance。
5. 执行正式 predict + update。
6. 对半径和 `dz` 做约束。
7. 返回 11D 兼容输出。

## typed 图优化器

当前主线已经是 typed GTSAM 实现，不再使用历史图优化分层或旧后端语义。

主链路：

```text
GraphOptimizerTest ROS node
  -> ArmorGraphTracker
  -> ArmorCvPixelGraph
  -> auto_graph::GraphOptimizer
  -> GTSAM iSAM2 / fixed-lag smoother
```

职责划分：

- `auto_graph::GraphOptimizer`：泛型 typed GTSAM 生命周期。负责变量声明、key 映射、
  initial values、factor graph、iSAM2/fixed-lag solve、estimate、reset。不要加入
  armor/camera/ROS/匹配语义。
- `ArmorCvPixelGraph`：装甲板 CV + 像素观测图。负责初始化、常速预测、装甲板匹配、
  运动因子、像素重投影因子、几何因子、auto_aim 风格边缘重投影因子和输出转换。
- `ArmorGraphTracker`：ROS-facing 薄封装。负责空观测、拒绝帧、失败 reset 和
  `TrackerUpdateResult` 兼容语义。
- `GraphOptimizerTest`：ROS 节点外壳。负责参数、TF、订阅 `/detector/armors`，
  发布 `/graph_optimizer/armors`、`/tracker/target`、`/graph_optimizer/marker`。

typed 变量：

| 变量 | 类型 | 动态 | key 前缀 | 含义 |
| --- | --- | --- | --- | --- |
| `center` | `gtsam::Point3` | 是 | `x` | 机器人中心 |
| `velocity` | `gtsam::Vector3` | 是 | `v` | 中心速度 |
| `yaw` | `gtsam::Rot2` | 是 | `r` | 中心 yaw，流形变量 |
| `vyaw` | `double` | 是 | `w` | yaw rate |
| `radius_a` | `double` | 否 | `a` | 偶数装甲板半径 logistic 状态 |
| `radius_b` | `double` | 否 | `b` | 奇数装甲板半径 logistic 状态 |
| `dz` | `double` | 否 | `z` | 奇偶装甲板高度差 |

动态变量用当前 frame id；静态变量 key 固定 frame `0`，fixed-lag smoother 下会刷新
timestamp 避免被边际化。

运动因子：

- `TranslationFactor`：`center(k) - center(k-1) - velocity(k-1) * dt`
- `VelocityFactor`：速度随机游走。
- `YawFactor`：`Rot2` yaw 积分，处理 `+-pi` 跨界。
- `VyawFactor`：yaw-rate 随机游走。

像素观测写图：

1. 每个匹配 armor 生成一个辅助 `Pose3` key：`h/j/k/l + frame`。
2. 插入辅助 `Pose3` 初值，并添加 pose prior。
3. 每个角点添加一个 `ArmorTypedReprojFactor`。
4. 偶数装甲板添加 `ArmorRadiusCenterZFactor`。
5. 奇数装甲板添加 `ArmorRadiusDZFactor`。
6. 四个角点齐全时，默认添加 direct edge reprojection factor：
   - 偶数：`ArmorEdgeCenterZReprojFactor` 连接 `radius_a/yaw/center`；
   - 奇数：`ArmorEdgeDZReprojFactor` 连接 `radius_b/dz/yaw/center`；
   - 残差参考 auto_aim `EdgeLoss`，每条边 1 维，混合归一化顶点/长度误差和边方向角度误差。
   - 标准装甲板使用 `standard.armor_pitch=+15deg`，`number=="outpost"` 时使用 `outpost.armor_pitch=-0.26rad`。
   - `number=="outpost"` 时走前哨站三槽位分支：`0/120/240deg` yaw offset，
     共用 `outpost.radius`，index 1/2 分别使用 `outpost.dz_1/outpost.dz_2`。

匹配代价：

```text
cost = yaw_diff + 3.0 * position_error
same as last_armor_index -> cost *= 0.85
cost > match_max_cost -> reject
```

`matchArmorIndicesUnique()` 保证同一帧最多一个观测占用同一个物理 index；未匹配观测标为
`-1`，写图时跳过，避免辅助 Pose3 key 冲突。

## 能量机关 typed 图模型

`RuneCvGraph` 位于 `graph_optimizer/rune_model.hpp/.cpp`，复用
`auto_graph::GraphOptimizer`。ROS 接入节点为 `src/rune_graph_optimizer.cpp`，
默认 launch 中保持注释，按需启用。

typed 变量：

| 变量 | 类型 | 动态 | key 前缀 | 含义 |
| --- | --- | --- | --- | --- |
| `center` | `gtsam::Point3` | 是 | `c` | 能量机关中心 |
| `roll` | `gtsam::Rot2` | 是 | `r` | 当前叶片 roll，流形变量 |
| `vroll` | `double` | 是 | `w` | roll 角速度 |
| `normal_yaw` | `gtsam::Rot2` | 否 | `n` | 能量机关平面法线 yaw |

写图流程：

1. `RuneRollFactor` 和 `RuneVrollFactor` 维护 CV roll 运动模型。
2. `RuneTargets` 是真实 detector 风格的紧凑观测数组；`RuneTarget` 只携带五点像素，不携带物理扇叶编号。
3. 先用 5 点做 PnP；图优化用观测 roll/center 与预测 5 个 slot 做门限匹配，匹配成功后才写图。
4. 每个匹配成功的物理叶片对应一个叶片辅助 `Pose3` key：`q/s/t/u/y + frame`。
5. PnP 成功时用 PnP pose 初始化/约束辅助 `Pose3`，失败时可用 direct reproj fallback 回退预测 pose。
6. 每个叶片写入 5 个 `RuneBladeReprojFactor`，像素顺序为
   `r_center, near_point, left_point, far_point, right_point`。
7. `RuneBladeGeometryFactor` 连接辅助 `Pose3`、`center`、`roll`、`normal_yaw`，
   残差包含位置、roll 和 yaw。
8. `RuneBladeDirectReprojFactor` 直接连接 `center/roll/normal_yaw`，使用
   `near/left/far/right` 两条对角线的角度差和归一化长度差。
9. `RuneGraphOutput` 输出优化状态、`observed_count`、内部匹配 slot 和基于当前 roll 的目标点；
   `rune_graph_optimizer` 再用 auto_aim 5 参数曲线拟合大符 roll 序列。

消息接口：

- `Point2d`：二维像素点。
- `RuneTarget`：单叶片检测结果和 5 个像素点，不包含物理扇叶编号。
- `RuneTargets`：观测到的叶片紧凑数组；小符通常最多 1 片，大符通常最多 2 片。
- `RuneInfo`：优化状态、平面 `normal_yaw/normal_pitch`、预测 roll、角速度、速度曲线参数和目标点。

## 状态向量索引

**CV 模型 (11D)**：

```text
[xc, vxc, yc, vyc, za, vza, yaw, vyaw, r1_u, r2_u, dz]
  0    1    2    3    4    5    6     7     8     9    10
```

- 位置：`0, 2, 4`
- 速度：`1, 3, 5`
- yaw / yaw rate：`6, 7`
- typed 图中半径使用 logistic 无界状态，物理范围由 `kRadiusMin/kRadiusMax` 限制。

**Singer 模型 (15D)**：

```text
[x, vx, ax, y, vy, ay, z, vz, az, yaw, vyaw, ayaw, r1, r2, dz]
 0   1   2  3   4   5  6   7   8   9    10    11   12  13  14
```

这些索引在模型、测试和文档中多处引用；修改前必须全局搜索并补测试。

## ROS2 接口

订阅：

- `/detector/armors` (`auto_aim_interfaces/msg/Armors`)：仿真器或检测器观测。
- `/rune_detector/rune_targets` (`auto_aim_interfaces/msg/RuneTargets`)：能量机关检测观测，供 `rune_graph_optimizer` 使用。
- `/detector/rune_targets` (`auto_aim_interfaces/msg/RuneTargets`)：能量机关仿真兼容旧话题。
- `/tracker/target` (`auto_aim_interfaces/msg/TrackerTarget`)：angle solver 输入。
- `/send_pack` (`auto_aim_interfaces/msg/SendData`)：云台仿真输入。
- `/recieve_pack` (`auto_aim_interfaces/msg/RecieveData`)：angle solver 反馈输入。

发布：

- `/detector/armors`：`armor_simulation_node` 发布带噪观测。
- `/armor_simulation/ground_truth`：`armor_simulation_node` 发布真值。
- `/simulation/marker`、`/simulation/image`：仿真可视化。
- `/track_result`、`/track_result/marker`：传统滤波器输出。
- `/graph_optimizer/armors`、`/graph_optimizer/marker`：图优化器或 jlu tracker 输出。
- `/tracker/target`：图优化器或 jlu tracker 发布给 angle solver。
- `/rune_graph_optimizer/rune_info` (`auto_aim_interfaces/msg/RuneInfo`)：能量机关图优化状态。
- `/rune_graph_optimizer/target_pose` (`auto_aim_interfaces/msg/TargetPose`)：能量机关目标位姿。
  orientation 使用符面 `normal_yaw/normal_pitch`，不是只写水平 yaw。
- `/rune_graph_optimizer/marker` (`visualization_msgs/msg/MarkerArray`)：能量机关图优化可视化；
  使用 `rune_center`、`rune_target`、`rune_status`、`rune` 和 `observed_position`。
- `/rune_simulation/ground_truth` (`armor_simulation/msg/RuneGroundTruth`)：能量机关中心 `x/y/z`、
  `normal_yaw/normal_pitch`、`roll/vroll`、模式和当前 active blade。
- `/rune_simulation/marker` (`visualization_msgs/msg/MarkerArray`)：能量机关仿真可视化；
  与图优化保持同类型，使用 `rune_center`、`rune_target`、`rune_status`、`rune` 和 `observed_position`。
- `/send_pack`：angle solver 或仿真真值瞄准模式发布给云台。
- `/recieve_pack`：云台仿真反馈。

## 仿真与相机约定

- 装甲板局部坐标：`X=法线, Y=宽度, Z=高度`。
- 角点顺序：`[左下, 左上, 右上, 右下]`。
- `armor_simulation_node` 支持 `mode: standard|outpost`。`standard` 使用四装甲板车辆几何；
  `outpost` 使用三装甲板前哨站几何，参数在 `outpost.*` 下。
- 能量机关 helper 的检测点顺序是 `r_center, near_point, left_point, far_point, right_point`；
  `far_point` 是目标框远端检测点，击打中心单独取半径 `0.700 m`，五叶片间隔为 `2*pi/5`。
- 装甲板位置：

```text
pos = center - r * [cos(armor_yaw), sin(armor_yaw), 0]
```

- marker/观测 pitch 统一为 `+15 deg`。
- `CameraModel::estimatePose()` 使用 OpenCV `SOLVEPNP_IPPE`。
- 平面 PnP 仍可能有两个相近候选；仿真侧用 `correctPlanarPnPAmbiguity()` 按 yaw 先验修正。
- 像素噪声：U 形距离模型 + 装甲板级相关噪声，`p_i' = p_i + n_c + n_i`。
- 全局 `publish_gimbal_gt: true` 时，`armor_simulation_node` 或 `rune_simulation_node` 直接向
  `/send_pack` 发布真值 yaw/pitch，绕过 tracker/graph_optimizer -> angle_solver 闭环；
  armor 瞄准机器人中心，rune 优先瞄准当前观测扇叶，其次 active blade 的击打中心。

## 配置文件

原则：优先改 YAML，不改头文件默认值。

| 文件 | 控制节点 | 关键参数 |
| --- | --- | --- |
| `src/filter_test/config/config.yaml` | `/filter`, `/graph_optimizer_test`, `/jlu_tracker`, `/gimbal_simulation` | 滤波器选择、过程/观测噪声、图优化噪声、smoother、`camera_info_url`、云台发布率。 |
| `src/armor_simulation/config/simulation_config.yaml` | `/armor_simulation_node`, `/rune_simulation_node` | 初始状态、运动限制、几何参数、像素噪声、离群点、`camera_info_url`、图像发布、`publish_gimbal_gt`。 |

图优化关键参数：

- `cold_start_frames`
- `smoother_lag`
- `smoother_type`
- `extra_iterations`
- `relinearize_threshold`
- `s2qxy`, `s2qz`, `s2qyaw`
- `vel_sigma`, `vyaw_sigma`
- `match_max_cost`
- `standard.pixel_noise_std`, `outpost.pixel_noise_std`
- `use_edge_reproj_factor`
- `edge_reproj_sigma`
- `edge_loss_slope_k`
- `standard.armor_pitch`, `outpost.armor_pitch`
- `outpost.radius`, `outpost.dz_1`, `outpost.dz_2`
- `standard.geo.*`, `outpost.geo.*`
- `camera_info_url`, `camera_name`

当前 typed pixel graph 路径固定启用 2D 角点观测。

## 依赖项

- ROS2 Humble (`rclcpp`, `tf2_ros`, `geometry_msgs`, `visualization_msgs`, `sensor_msgs`)
- Eigen3
- OpenCV
- Ceres Solver
- GTSAM 4.3，默认在 `/usr/local/lib`
- TBB
- `auto_aim_interfaces`

## 约定

- 头文件布局：`include/<package>/...`，include 时使用 `"<package>/..."`。
- launch 中节点输出使用 `output='screen'`。
- 构建产物 (`build/`, `install/`, `log/`) 和 `.vscode/` 已被 git 忽略，不要提交。
- ROS 消息生成在 `install/.../include/...`，不在 `src/`。
- `.vscode/c_cpp_properties.json` 硬编码 `/opt/ros/humble`；换 ROS 发行版需要同步修改。
- 没有 CI；验证方式是 `colcon test` 加必要的手动 launch。
- 工作区可能有用户未提交改动，修改前先看 `git status --short`，不要回退他人改动。

## 已知问题

- `radius_2` 仍可能过冲，需要继续调参。
- 传统滤波路径中 yaw 仍有按标量处理的遗留点；typed 图中 yaw 使用 `Rot2`。
- 匹配是启发式代价，不是马氏距离。
- UKF 收敛速度可能慢于 EKF。
- Singer 模型在部分参数组合下可能产生 NaN。
- `filter_test` 和 `auto_aim_interfaces` 的 package metadata 仍有 TODO license/description。

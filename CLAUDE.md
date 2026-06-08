# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

ROS2 **Humble** / C++17 / GCC 11 工作区，用于实时装甲板跟踪实验。核心方向是传统
EKF/UKF 滤波和 GTSAM typed 因子图优化。`src/` 下有三个 package：

1. `armor_simulation`：3D 运动仿真、装甲板几何、相机投影、像素噪声、IPPE PnP、云台 TF、角度解算。
2. `filter_test`：传统滤波器、typed GTSAM 图优化器、jlu tracker 适配层、ROS 节点、配置和测试。
3. `auto_aim_interfaces`：共享自瞄消息接口。

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
│   │   │   └── armor_tracker.hpp     # ArmorGraphTracker, FrameTimeTracker
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
│   │   │   └── armor_tracker.cpp
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

当前主线已经是 typed GTSAM 实现，不再使用旧 `auto_graph_optimizer/` 或
`tracker_core/observation_backend` 分层。

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
  运动因子、像素重投影因子、几何因子和输出转换。
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

匹配代价：

```text
cost = yaw_diff + 3.0 * position_error
same as last_armor_index -> cost *= 0.85
cost > match_max_cost -> reject
```

`matchArmorIndicesUnique()` 保证同一帧最多一个观测占用同一个物理 index；未匹配观测标为
`-1`，写图时跳过，避免辅助 Pose3 key 冲突。

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
- `/send_pack`：angle solver 或仿真真值瞄准模式发布给云台。
- `/recieve_pack`：云台仿真反馈。

## 仿真与相机约定

- 装甲板局部坐标：`X=法线, Y=宽度, Z=高度`。
- 角点顺序：`[左下, 左上, 右上, 右下]`。
- 装甲板位置：

```text
pos = center - r * [cos(armor_yaw), sin(armor_yaw), 0]
```

- marker/观测 pitch 统一为 `+15 deg`。
- `CameraModel::estimatePose()` 使用 OpenCV `SOLVEPNP_IPPE`。
- 平面 PnP 仍可能有两个相近候选；仿真侧用 `correctPlanarPnPAmbiguity()` 按 yaw 先验修正。
- 像素噪声：U 形距离模型 + 装甲板级相关噪声，`p_i' = p_i + n_c + n_i`。
- `publish_gimbal_gt: true` 时，`armor_simulation_node` 直接向 `/send_pack` 发布真值 yaw/pitch，
  绕过 tracker -> angle_solver 闭环；`gimbal_simulation` 仍只订阅 `/send_pack` 并发布 TF。

## 配置文件

原则：优先改 YAML，不改头文件默认值。

| 文件 | 控制节点 | 关键参数 |
| --- | --- | --- |
| `src/filter_test/config/config.yaml` | `/filter`, `/graph_optimizer_test`, `/jlu_tracker`, `/gimbal_simulation` | 滤波器选择、过程/观测噪声、图优化噪声、smoother、相机内参、云台发布率。 |
| `src/armor_simulation/config/simulation_config.yaml` | `/armor_simulation_node` | 初始状态、运动限制、几何参数、像素噪声、离群点、相机内参、图像发布、`publish_gimbal_gt`。 |

图优化关键参数：

- `cold_start_frames`
- `smoother_lag`
- `smoother_type`
- `extra_iterations`
- `relinearize_threshold`
- `s2qxy`, `s2qz`, `s2qyaw`
- `vel_sigma`, `vyaw_sigma`
- `match_max_cost`
- `pixel_noise_std`
- `geo_tangential`, `geo_radial`, `geo_height`, `geo_yaw`
- `camera_fx/fy/cx/cy`, `distortion_k1/k2/p1/p2`

`use_2d_observation` 参数只为旧配置兼容保留；当前 typed pixel graph 路径固定启用。

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

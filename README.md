# filter_test - ROS2 装甲板跟踪与图优化实验工作区

这是一个 ROS2 Humble / C++17 工作区，用于验证实时装甲板跟踪中的两类方案：
传统 EKF/UKF 滤波器，以及基于 GTSAM 的 typed 因子图优化器。工作区同时包含 3D
运动仿真、相机投影、像素噪声、PnP、云台 TF 仿真、角度解算和共享自瞄消息。

当前主线图优化链路是：

```text
/detector/armors
  -> GraphOptimizerTest ROS 节点
  -> ArmorGraphTracker
  -> ArmorCvPixelGraph
  -> auto_graph::GraphOptimizer
  -> GTSAM iSAM2 / fixed-lag smoother
  -> /graph_optimizer/armors, /tracker/target, /graph_optimizer/marker
```

`filter_test.launch.py` 当前默认只启动 `graph_optimizer_test`。仿真器、云台仿真、
传统滤波器、角度解算和 jlu tracker 都已经在 launch 文件中定义，但默认注释，
需要完整闭环时再按需打开。

## 工作区包

| 包名 | 作用 |
| --- | --- |
| `auto_aim_interfaces` | 共享消息接口，包括 `Armors`、`Armor`、`TrackerTarget`、串口收发和调试消息。 |
| `armor_simulation` | 3D 目标运动、装甲板几何、相机投影、像素噪声、IPPE PnP、真值发布、云台仿真和角度解算。 |
| `filter_test` | EKF/UKF 传统滤波器、typed GTSAM 图优化器、jlu tracker 适配层、ROS 节点、配置、launch 和测试。 |

## 主要能力

- 传统装甲板跟踪：支持 EKF 或 UKF。
- 两套传统运动模型：CV 11D 和 Singer 15D。
- typed GTSAM 图优化：支持 iSAM2 和 fixed-lag smoother。
- 像素级观测图：
  - 每块匹配装甲板引入一个辅助 `Pose3`；
  - 每个可见角点添加 2D 重投影因子；
  - 再通过几何因子连接到中心、yaw、半径和高度差主状态。
- 3D 仿真：含相机投影、U 形距离像素噪声、装甲板级相关角点噪声、离群点和 IPPE PnP。
- 云台仿真：S-curve yaw/pitch 动力学、`odom -> gimbal_link -> camera_optical_frame` TF。
- RViz 可视化：仿真真值、观测装甲板、图优化预测装甲板、中心点。
- 单元测试：覆盖滤波器、typed 图核心、因子残差/Jacobian、匹配、冷启动、失败恢复和 tracker 生命周期。

## 环境要求

项目按以下环境开发：

- Ubuntu 22.04
- ROS2 Humble
- GCC 11
- C++17
- Eigen3
- OpenCV
- Ceres Solver
- GTSAM 4.3
- TBB

GTSAM 当前默认安装在 `/usr/local/lib`，运行节点或测试前需要设置：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

如果没有这个环境变量，编译可能成功，但运行 `graph_optimizer_test` 或图优化测试时会找不到 GTSAM 动态库。

## 构建

从工作区根目录执行：

```bash
colcon build --packages-select auto_aim_interfaces armor_simulation filter_test
source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

启用测试构建：

```bash
colcon build --packages-select auto_aim_interfaces armor_simulation filter_test --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

只构建图优化相关包时也建议包含接口包，避免消息头来自旧的 install 空间：

```bash
colcon build --packages-select auto_aim_interfaces filter_test --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

## 运行

### 当前默认 launch

```bash
ros2 launch filter_test filter_test.launch.py
```

当前 launch 默认启动：

```text
filter_test/graph_optimizer_test
```

launch 文件中还定义了以下节点，但默认注释：

```text
armor_simulation/armor_simulation_node
armor_simulation/gimbal_simulation
armor_simulation/angle_solver
filter_test/filter
filter_test/jlu_tracker
```

### 仅启动仿真器

```bash
ros2 launch armor_simulation simulation.launch.py
```

该 launch 使用 `src/armor_simulation/config/simulation_config.yaml`。

### 单独运行节点

```bash
# 仿真观测源：运动 + 相机投影 + 像素噪声 + PnP
ros2 run armor_simulation armor_simulation_node \
  --ros-args --params-file src/armor_simulation/config/simulation_config.yaml

# 云台动力学 + TF 广播
ros2 run armor_simulation gimbal_simulation \
  --ros-args --params-file src/filter_test/config/config.yaml

# 角度解算：/tracker/target -> /send_pack
ros2 run armor_simulation angle_solver

# 传统 EKF/UKF 滤波器
ros2 run filter_test filter \
  --ros-args --params-file src/filter_test/config/config.yaml

# typed GTSAM 图优化器
ros2 run filter_test graph_optimizer_test \
  --ros-args --params-file src/filter_test/config/config.yaml

# jlu_vision_26 tracker 适配层
ros2 run filter_test jlu_tracker \
  --ros-args --params-file src/filter_test/config/config.yaml
```

图优化器在初始化后需要 TF 中存在 `odom -> camera_optical_frame`。正常仿真闭环中由
`gimbal_simulation` 发布该 TF。

## 配置文件

优先改 YAML，不要直接改头文件默认值。

| 文件 | 控制节点 | 关键内容 |
| --- | --- | --- |
| `src/filter_test/config/config.yaml` | `/filter`、`/graph_optimizer_test`、`/jlu_tracker`、`/gimbal_simulation` | 滤波器选择、图优化噪声、smoother、相机内参、云台发布率。 |
| `src/armor_simulation/config/simulation_config.yaml` | `/armor_simulation_node` | 初始状态、运动限幅、几何参数、相机参数、像素噪声、离群点、图像发布。 |

### `/filter` 关键参数

- `use_ekf`：`true` 使用 EKF，`false` 使用 UKF。
- `use_cv_model`：`true` 使用 CV 11D，`false` 使用 Singer 15D。
- `s2qxy_cv`、`s2qz_cv`、`s2qyaw_cv`、`s2qr_cv`、`s2qdz_cv`：CV 过程噪声。
- `s2qxy_singer`、`s2qz_singer`、`s2qyaw_singer`、`s2qr_singer`、`s2qdz_singer`、`tau_singer`：Singer 过程噪声和时间常数。
- `ukf_alpha`、`ukf_beta`、`ukf_kappa`：UKF sigma 点参数。
- `r_pose_det`、`r_distance_det`、`r_yaw_det`：检测器/PnP 观测噪声。
- `init_r`：初始半径参数。

### `/graph_optimizer_test` 关键参数

求解器和运行行为：

- `verbose`：是否打印图优化调试信息。
- `cold_start_frames`：冷启动累计帧数；早期帧可接受但不一定发布优化结果。
- `smoother_lag`：`0.0` 表示普通 iSAM2；大于 0 时启用 fixed-lag smoother。
- `smoother_type`：`"incremental"` 或 `"batch"`。
- `extra_iterations`：每帧 `solve()` 内部 GTSAM update 总次数。ROS 参数名为兼容旧配置保留，内部映射为 `update_iterations`。
- `relinearize_threshold`：iSAM2 重线性化阈值。

运动和平滑噪声：

- `s2qxy`、`s2qz`、`s2qyaw`：中心位置和 yaw 运动噪声方差。
- `vel_sigma`：速度随机游走标准差。
- `vyaw_sigma`：yaw-rate 随机游走标准差。

匹配和观测噪声：

- `match_max_cost`：匹配拒绝阈值，代价为 `yaw_diff + 3 * position_error`。
- `pixel_noise_std`：角点 2D 像素残差标准差。
- `geo_tangential`、`geo_radial`、`geo_height`、`geo_yaw`：几何约束因子标准差。

相机参数：

- `camera_fx`、`camera_fy`、`camera_cx`、`camera_cy`
- `distortion_k1`、`distortion_k2`、`distortion_p1`、`distortion_p2`

相机外参来自运行时 TF，不从 YAML 读取。

## 架构

### 传统滤波路径

```text
/detector/armors
  -> filter_test::ArmorTest
  -> ArmorFilter
  -> EKF 或 UKF
  -> CV 或 Singer 状态模型
  -> /track_result, /track_result/marker
```

核心文件：

- `src/filter_test/include/filter_test/filter.hpp`
- `src/filter_test/include/filter_test/filters/cv_model.hpp`
- `src/filter_test/include/filter_test/filters/singer_model.hpp`
- `src/filter_test/include/filter_test/filters/extended_kalman.hpp`
- `src/filter_test/include/filter_test/filters/unscented_kalman.hpp`

### typed 图优化路径

```text
/detector/armors
  -> GraphOptimizerTest
  -> ArmorGraphTracker
  -> ArmorCvPixelGraph
  -> auto_graph::GraphOptimizer
  -> GTSAM
```

核心文件：

- `src/filter_test/include/filter_test/graph_optimizer/graph_core.hpp`
- `src/filter_test/include/filter_test/graph_optimizer/graph_math.hpp`
- `src/filter_test/include/filter_test/graph_optimizer/armor_model.hpp`
- `src/filter_test/include/filter_test/graph_optimizer/armor_tracker.hpp`
- `src/filter_test/src/graph_optimizer/graph_core.cpp`
- `src/filter_test/src/graph_optimizer/armor_model.cpp`
- `src/filter_test/src/graph_optimizer/armor_tracker.cpp`
- `src/filter_test/src/graph_optimizer_test.cpp`

`auto_graph::GraphOptimizer` 只负责通用 typed GTSAM 生命周期：

- 声明动态/静态 typed 变量；
- typed `Var<T>` 到 GTSAM key 的映射；
- 初始化、帧推进、初值插入和因子插入；
- iSAM2 或 fixed-lag smoother 求解；
- 读取最新 estimate；
- reset 和失败恢复。

装甲板业务逻辑保留在 `ArmorCvPixelGraph`：

- 首帧初始化；
- 根据上一帧状态做常速预测；
- 单帧装甲板 index 唯一匹配；
- 添加运动因子；
- 添加像素重投影因子；
- 添加中心/yaw/半径/dz 几何因子；
- 生成发布层需要的状态和 marker 数据。

`ArmorGraphTracker` 是 ROS-facing 的薄封装，处理空观测、拒绝帧、失败恢复和
`TrackerUpdateResult` 结果包装。

### typed 图变量

| 变量 | 类型 | 动态 | key 前缀 | 含义 |
| --- | --- | --- | --- | --- |
| `center` | `gtsam::Point3` | 是 | `x` | 机器人中心位置 |
| `velocity` | `gtsam::Vector3` | 是 | `v` | 中心速度 |
| `yaw` | `gtsam::Rot2` | 是 | `r` | 中心 yaw，流形变量 |
| `vyaw` | `double` | 是 | `w` | yaw 角速度 |
| `radius_a` | `double` | 否 | `a` | 偶数装甲板半径的 logistic 无界状态 |
| `radius_b` | `double` | 否 | `b` | 奇数装甲板半径的 logistic 无界状态 |
| `dz` | `double` | 否 | `z` | 奇偶装甲板高度差 |

动态变量使用当前 frame id。静态变量的 GTSAM key 固定在 frame `0`，但 fixed-lag
smoother 中会刷新 timestamp，避免静态几何参数被边际化。

每帧每个物理装甲板还会生成辅助 `Pose3` key：

```text
index 0 -> h<frame>
index 1 -> j<frame>
index 2 -> k<frame>
index 3 -> l<frame>
```

### 运动因子

typed 图优化器当前将 CV 运动拆为小因子：

- `TranslationFactor`：`center(k) - center(k-1) - velocity(k-1) * dt`
- `VelocityFactor`：`velocity(k) - velocity(k-1)`
- `YawFactor`：`Rot2` yaw 积分，使用最短角残差处理 `+-pi` 跨界
- `VyawFactor`：`vyaw(k) - vyaw(k-1)`

`dt` 属于 tracker/运动因子构造，不放进 `GraphOptimizer::beginFrame()`。

### 像素观测模型

对每个匹配成功的装甲板观测：

1. 将观测装甲板位姿从 odom 转到 camera frame。
2. 插入该物理装甲板 slot 的辅助 `Pose3` 初值。
3. 添加一个 `Pose3` prior 作为局部锚点。
4. 对每个检测到的角点添加一个 `ArmorTypedReprojFactor`。
5. 添加一个几何因子把辅助 `Pose3` 连接回主状态：
   - 偶数 index 使用 `ArmorRadiusCenterZFactor`；
   - 奇数 index 使用 `ArmorRadiusDZFactor`。

这种两级结构让像素误差和车辆几何约束保持小维度、可测试，也避免把所有主状态直接塞进一个大观测因子。

## 状态向量

### CV 模型 11D

```text
[xc, vxc, yc, vyc, za, vza, yaw, vyaw, r1_u, r2_u, dz]
  0    1    2    3    4    5    6     7     8     9    10
```

说明：

- 位置索引：`0, 2, 4`
- 速度索引：`1, 3, 5`
- yaw / yaw rate：`6, 7`
- typed 图中半径使用 logistic 无界状态，物理半径范围由 `kRadiusMin/kRadiusMax` 限制。
- `dz` 是奇偶装甲板高度差。

### Singer 模型 15D

```text
[x, vx, ax, y, vy, ay, z, vz, az, yaw, vyaw, ayaw, r1, r2, dz]
 0   1   2  3   4   5  6   7   8   9    10    11   12  13  14
```

这些索引在多个模型文件、测试和文档中被引用，修改前需要全局搜索并同步测试。

## ROS2 接口

### 主要订阅

| 话题 | 消息 | 使用者 |
| --- | --- | --- |
| `/detector/armors` | `auto_aim_interfaces/msg/Armors` | 传统滤波器、图优化器、jlu tracker。 |
| `/tracker/target` | `auto_aim_interfaces/msg/TrackerTarget` | angle solver。 |
| `/recieve_pack` | `auto_aim_interfaces/msg/RecieveData` | angle solver 反馈输入。 |
| `/send_pack` | `auto_aim_interfaces/msg/SendData` | 云台仿真控制输入。 |

### 主要发布

| 话题 | 消息 | 发布者 |
| --- | --- | --- |
| `/detector/armors` | `auto_aim_interfaces/msg/Armors` | `armor_simulation_node`。 |
| `/armor_simulation/ground_truth` | `armor_simulation/msg/GroundTruth` | `armor_simulation_node`。 |
| `/simulation/marker` | `visualization_msgs/msg/MarkerArray` | `armor_simulation_node`。 |
| `/simulation/image` | `sensor_msgs/msg/Image` | `armor_simulation_node`，需启用图像发布。 |
| `/track_result` | `filter_test/msg/Result` | 传统滤波器。 |
| `/track_result/marker` | `visualization_msgs/msg/MarkerArray` | 传统滤波器。 |
| `/graph_optimizer/armors` | `auto_aim_interfaces/msg/Armors` | 图优化器或 jlu tracker。 |
| `/graph_optimizer/marker` | `visualization_msgs/msg/MarkerArray` | 图优化器或 jlu tracker。 |
| `/tracker/target` | `auto_aim_interfaces/msg/TrackerTarget` | 图优化器或 jlu tracker。 |
| `/send_pack` | `auto_aim_interfaces/msg/SendData` | angle solver 或仿真真值瞄准模式。 |
| `/recieve_pack` | `auto_aim_interfaces/msg/RecieveData` | 云台仿真。 |

## 仿真约定

- 装甲板局部坐标：`X = 法线`，`Y = 宽度`，`Z = 高度`。
- 角点顺序：`[左下, 左上, 右上, 右下]`。
- 装甲板位置模型：

```text
armor_position = center - radius * [cos(armor_yaw), sin(armor_yaw), 0]
```

- 偶数 index 使用 `radius_1` 且 `dz = 0`。
- 奇数 index 使用 `radius_2` 且高度加 `dz`。
- 可视化 pitch 固定为 `+15 deg`。
- 相机投影使用与 GTSAM/OpenCV 兼容的内参和畸变。
- PnP 使用 OpenCV `SOLVEPNP_IPPE`。平面 PnP 双解歧义在仿真侧用 yaw 先验修正后再发布。
- 像素噪声使用距离相关 U 形模型，并叠加装甲板级相关噪声：

```text
p_i' = p_i + n_common + n_independent_i
```

`pixel_noise_common_ratio` 越大，四个角点共同平移越多，矩形形状破坏越少，PnP yaw 通常越稳。

## 测试

构建并运行 `filter_test` 全部测试：

```bash
colcon build --packages-select auto_aim_interfaces filter_test --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
colcon test --packages-select filter_test
colcon test-result --verbose
```

只运行图优化测试：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
colcon test --packages-select filter_test \
  --ctest-args -R test_auto_graph_optimizer --output-on-failure
```

只运行传统滤波器测试：

```bash
colcon test --packages-select filter_test \
  --ctest-args -R filter_test --output-on-failure
```

只运行仿真噪声/PnP 测试：

```bash
colcon build --packages-select armor_simulation --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select armor_simulation \
  --ctest-args -R test_noise_model --output-on-failure
```

测试入口：

- `src/filter_test/test/filter_test.cpp`
- `src/filter_test/test/test_auto_graph_optimizer.cpp`
- `src/armor_simulation/test/test_noise_model.cpp`

## 仓库结构

```text
.
├── README.md
├── AGENTS.md
├── CLAUDE.md
├── docs/
├── .claude/skills/
│   ├── armor-filter/
│   ├── armor-graph-optimizer/
│   └── armor-simulator/
├── src/
│   ├── auto_aim_interfaces/
│   ├── armor_simulation/
│   └── filter_test/
├── test_graph_optimizer.sh
└── singer1970.pdf
```

`build/`、`install/`、`log/`、`.vscode/` 已被 git 忽略，不应提交。

## 常见问题

### 运行时报 GTSAM 动态库找不到

设置：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

然后重新运行节点或测试。

### 消息头或接口类型找不到

重新构建接口包并 source install 空间：

```bash
colcon build --packages-select auto_aim_interfaces
source install/setup.bash
```

### 图优化收到观测但不立即发布

检查：

- `cold_start_frames`：冷启动帧可能只累计图而不发布优化结果。
- TF：初始化之后需要 `odom -> camera_optical_frame`。
- `match_max_cost`：观测可能被判为离群并标记为 `-1`。
- 是否所有观测都未匹配：当前 tracker 会 reset 并等待重新初始化。

### 仿真有观测但图优化缺 TF

启动 `gimbal_simulation`，或在 `filter_test.launch.py` 中打开对应节点。正常图优化需要相机到 odom 的实时外参。

## 已知问题

- `radius_2` 仍可能过冲，当前主要依赖几何约束、匹配质量和像素观测质量，需要继续调参。
- 匹配使用启发式代价，不是基于协方差的马氏距离。
- UKF 收敛速度可能慢于 EKF。
- Singer 模型在部分参数组合下可能出现 NaN。
- `filter_test` 和 `auto_aim_interfaces` 的 package metadata 仍有 TODO license/description。

## 更多文档

- `AGENTS.md`：给 agent 使用的高信噪比摘要。
- `CLAUDE.md`：Claude Code 工程上下文和架构说明。
- `.claude/skills/armor-filter/SKILL.md`：传统滤波器专项 skill。
- `.claude/skills/armor-graph-optimizer/SKILL.md`：图优化专项 skill。
- `.claude/skills/armor-simulator/SKILL.md`：仿真器专项 skill。
- `docs/graph_optimizer_architecture.html`：图优化架构说明。
- `docs/gtsam_graph_optimizer_guide.html`：GTSAM 图优化相关说明。
- `docs/factor_graph_optimization.html`：因子图优化背景。

## 许可证

`armor_simulation` 当前声明 MIT；`filter_test` 和 `auto_aim_interfaces` 的
`package.xml` 仍保留 TODO license 字段，发布前需要统一整理。

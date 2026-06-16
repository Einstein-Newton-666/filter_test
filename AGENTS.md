# AGENTS.md — filter_test

ROS2 C++ 工作区：基于 EKF/UKF + typed GTSAM 因子图优化的实时装甲板跟踪；当前分支也在接入
能量机关图优化和前哨站三装甲板模型。`src/` 下三个包：

| 包名                   | 作用                                                             |
|------------------------|------------------------------------------------------------------|
| `armor_simulation`     | 3D 运动仿真 + 装甲板/能量机关/前哨站几何 + 相机投影 + 噪声 + PnP + 云台仿真 |
| `filter_test`          | EKF/UKF (`filter`) + 装甲板 iSAM2 图优化 + `RuneCvGraph` 能量机关图模型 |
| `auto_aim_interfaces`  | 共享的 `Armors` / `Armor` / `TrackerTarget` / `RuneTarget(s)` / `RuneInfo` 消息 |

详细架构/用法见 `README.md`、`CLAUDE.md` 和 `.claude/skills/armor-graph-optimizer/SKILL.md`。
本文件是高信噪比的精简摘要。

## 编译与运行

```bash
# 编译
colcon build --packages-select armor_simulation filter_test auto_aim_interfaces

# 编译并启用测试
colcon build --packages-select armor_simulation filter_test --cmake-args -DBUILD_TESTING=ON

# 本机内存紧张时低并发构建
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --parallel-workers 1 --packages-select armor_simulation filter_test auto_aim_interfaces --cmake-args -DBUILD_TESTING=ON

# 运行测试
colcon test --packages-select filter_test
colcon test-result --verbose

# 必填：GTSAM 4.3 装在 /usr/local/lib (不在默认链接路径)
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
source install/setup.bash

# 当前 launch 返回列表默认启用 sim + gimbal + filter；
# graph_optimizer/filter_graph_optimizer/angle_solver 等按需取消注释或单独运行
ros2 launch filter_test filter_test.launch.py

# 单独运行节点
ros2 run armor_simulation armor_simulation_node
ros2 run armor_simulation gimbal_simulation
ros2 run filter_test filter
ros2 run filter_test graph_optimizer_test --ros-args --params-file src/filter_test/config/config.yaml
ros2 run filter_test filter_graph_optimizer --ros-args --params-file src/filter_test/config/config.yaml
```

ROS 发行版：**Humble** (`.vscode/c_cpp_properties.json` 硬编码 `/opt/ros/humble` 路径)。
C++17，GCC 11 (Ubuntu 22.04)。

## 状态向量索引 (filter 与 graph_optimizer 通用)

**CV 模型 (11D)** — `[xc, vxc, yc, vyc, za, vza, yaw, vyaw, r1_u, r2_u, dz]`
- 位置: 0,2,4  速度: 1,3,5  Yaw: 6  vyaw: 7
- 半径 (logistic 编码, 静态): 8,9  dz (静态): 10

**Singer 模型 (15D)** — `[x, vx, ax, y, vy, ay, z, vz, az, yaw, vyaw, ayaw, r1, r2, dz]`

这些索引在多个头文件和测试中被引用 (`cv_model.hpp`、`singer_model.hpp`、
`graph_optimizer/armor_model.hpp` 等) — 修改需格外小心。

## 传统滤波器结构

核心文件：

- `filters/extended_kalman.hpp`、`filters/unscented_kalman.hpp` — 底层 EKF/UKF。
- `filters/cv_model.hpp`、`filters/singer_model.hpp` — CV/Singer 状态转移、观测和 Q/R。
- `filters/filter_common.hpp` — 角度、状态枚举和 ypd 工具。
- `filter.hpp` + `src/filters/filter.cpp` — `ArmorFilter` 初始化、匹配、预测更新和输出映射。
- `filter_test.hpp` + `src/filter_test.cpp` — `/filter` ROS 节点入口、参数、订阅发布和 marker。
- `src/filter_graph_optimizer.cpp` — 同步 `/detector/armors` 与 `/track_result`，将滤波器结果作为图优化初值/弱先验。

## 图优化框架 — typed GTSAM

当前主链路：

```text
GraphOptimizerTest -> ArmorGraphTracker -> ArmorCvPixelGraph -> auto_graph::GraphOptimizer -> GTSAM
```

核心文件：

- `graph_optimizer/graph_core.hpp/.cpp` — typed `Var<T>`、key 映射、iSAM2/fixed-lag 生命周期
- `graph_optimizer/armor_model.hpp/.cpp` — 状态变量、匹配、运动因子、像素重投影和几何因子
- `graph_optimizer/armor_tracker.hpp/.cpp` — 空观测、reset、求解结果兼容语义
- `graph_optimizer/rune_model.hpp/.cpp` — `RuneCvGraph`、能量机关 typed 变量、roll 运动因子、叶片重投影/几何因子
- `graph_optimizer_test.cpp` — ROS 参数、TF、订阅和发布
- `filter_graph_optimizer.cpp` — 滤波器前端 + 图优化后端 ROS 节点

typed 变量：
`center` (Point3, 动态)、`velocity` (Vector3, 动态)、`yaw` (Rot2, 动态)、
`vyaw` (double, 动态)、`radius_a/radius_b` (double, 静态 logistic)、`dz` (double, 静态)，
以及前哨站静态变量 `outpost_radius/outpost_dz_1/outpost_dz_2`。

2D 像素观测固定启用。每个匹配装甲板：

1. 插入辅助 `Pose3` key (`h/j/k/l + frame`) 和 pose prior
2. `ArmorTypedReprojFactor` — 1-key Pose3(camera) → 每个角点 2D 像素误差
3. `ArmorRadiusCenterZFactor` / `ArmorRadiusDZFactor` — 辅助 Pose3 ↔ center/yaw/radius/dz 几何约束
4. `ArmorEdgeCenterZReprojFactor` / `ArmorEdgeDZReprojFactor` — 直接连接主状态，
   参考 auto_aim `EdgeLoss` 对四条边计算角度差和归一化长度/顶点位移误差

匹配代价为 `yaw_diff + 3 * position_error`，`matchArmorIndicesUnique()` 保证单帧 index 唯一；
未匹配观测标为 `-1` 并跳过写图，避免重复辅助 key。

`Armor.number == "outpost"` 时切换为三槽位前哨站模型：yaw offset 为
`0/2π/3/4π/3`，三块共用 `outpost_radius`，index 1/2 分别使用
`outpost_dz_1/outpost_dz_2`。仿真器将物理 index 写入 `Armor.priority`，
首帧初始化会用它反推中心 yaw 和中心 z。

能量机关图模型复用同一 `auto_graph::GraphOptimizer` core：
`RuneCvGraph -> auto_graph::GraphOptimizer -> GTSAM`。typed 变量为
`center` (Point3, 动态)、`roll` (Rot2, 动态)、`vroll` (double, 动态)、
`normal_yaw` (Rot2, 静态)。`RuneTargets` 是真实 detector 风格的紧凑五点观测数组，
不携带物理扇叶编号；`RuneCvGraph` 先做 5 点 PnP，再用预测 roll 和中心位置把观测匹配到 5 个内部 slot。
每个匹配成功的物理叶片使用辅助 `Pose3` key (`q/s/t/u/y + frame`)。5 点 PnP 成功时作为辅助 pose 初值和 prior；
失败时回退预测 pose。随后按 `r_center/near/left/far/right` 五点写
`RuneBladeReprojFactor`，再用 `RuneBladeGeometryFactor` 连接 center/roll/normal_yaw。
额外的 `RuneBladeDirectReprojFactor` 直接连接 `center/roll/normal_yaw`，使用
auto_aim 风格 `RuneRollLoss`：只用 `near/left/far/right` 四点的两条对角线角度差和
归一化长度差。`rune_graph_optimizer` 还维护 auto_aim 5 参数大符曲线拟合器，
用图优化后的连续 roll 序列输出拟合/预测 roll 和角速度。
`rune_simulation_node` 发布 `/rune_detector/rune_targets`、`/detector/rune_targets`、
`/rune_simulation/ground_truth` 和 `/rune_simulation/marker`，`rune_graph_optimizer`
发布 `/rune_graph_optimizer/rune_info`、`/rune_graph_optimizer/target_pose` 和 marker；
默认 launch 中仍保持注释，按需开启。
`normal_yaw/normal_pitch` 表示符面姿态：仿真器写入 `RuneGroundTruth` 和 `RuneInfo`，
图优化节点写入 `RuneInfo` 和 `TargetPose.orientation`。

`armor_simulation_node` 和 `rune_simulation_node` 通过全局 `publish_gimbal_gt: true` 直接向
`/send_pack` 发布真值 yaw/pitch，绕过 tracker/graph_optimizer → angle_solver 闭环；
armor 瞄准机器人中心，rune 优先瞄准当前观测扇叶，其次 active blade 的击打中心。`gimbal_simulation`
仍订阅 `/send_pack` 并发布 TF。

## 仿真/PnP/可视化约定

- 装甲板局部坐标：`X=法线, Y=宽度, Z=高度`；角点顺序 `[左下, 左上, 右上, 右下]`
- `armor_simulation_node` 支持 `mode: standard|outpost`；`outpost` 为三装甲板前哨站几何，
  参数在 `outpost.radius/dz_1/dz_2/armor_pitch`
- 能量机关 helper：5 点顺序 `r_center, near_point, left_point, far_point, right_point`；
  `far_point` 是目标框远端检测点，击打中心单独取半径 `0.700 m`；叶片间隔 `2*pi/5`
- `rune_simulation_node` 小符每次随机发布 1 片扇叶，大符每次随机发布 2 片扇叶；
  默认 3 秒切换一组 active blade，`RuneTargets` 只包含观测到的 active blade 五点，不发布物理编号；未 active 的扇叶只画真值 marker
- 能量机关仿真器和 `rune_graph_optimizer` 使用同一套 marker 类型：
  `rune_center` 中心球、`rune_target` 目标点球、`rune_status` 文本、
  `rune` 五片扇叶球/半径线、`observed_position` 观测扇叶球
- 仿真几何：`pos = center - r * [cos(armor_yaw), sin(armor_yaw)]`；marker/观测 pitch 统一为 `+15°`
- `CameraModel::estimatePose()` 使用 OpenCV `SOLVEPNP_IPPE`。IPPE 仍可能有两个相近平面候选；
  仿真侧用 `correctPlanarPnPAmbiguity()` 按 yaw 先验修正分支，并在 `success=false` 时跳过观测
- 像素噪声：U 形距离模型 + 装甲板级相关噪声
  `p_i' = p_i + n_c + n_i`，`pixel_noise_common_ratio` 越大，四角点共同平移越多，PnP yaw 越稳

## 配置文件 (修改这些，而不是头文件默认值)

- `src/filter_test/config/config.yaml` — 顶层 `tracker_backend`、`/filter` (CV/Singer、EKF/UKF、R 参数)、
  `/graph_optimizer_test` / `/filter_graph_optimizer` (像素 σ、几何噪声、前端弱先验、
  auto_aim edge 重投影、`vel_sigma/vyaw_sigma`、smoother、`camera_info_url`) 和 `/rune_graph_optimizer`
- `src/armor_simulation/config/simulation_config.yaml` — 初始状态、运动限制、几何参数、
  `mode: standard|outpost`、前哨站 `outpost.*`、像素噪声 U 形模型、
  `pixel_noise_common_ratio`、离群点、`camera_info_url`

## 已知问题 (未经重新调参请勿"修复")

- `r2` (奇数装甲板半径) 过冲仍需调参；当前 typed 图已有
  `VelocityFactor` / `VyawFactor`，但半径仍依赖几何因子和观测质量
- `yaw` 在传统滤波路径仍按标量处理；typed 图优化主 yaw 使用 `gtsam::Rot2`
- 装甲板匹配使用启发式代价，而非马氏距离
- UKF 收敛速度比 EKF 慢
- Singer 模型在某些参数下可能产生 NaN

## 约定

- 头文件布局：`include/<package>/...` → 以 `"<package>/..."` 引入
- 所有 exec 输出：`output='screen'` (在 launch 文件中)
- 构建产物 (`build/`、`install/`、`log/`、`.vscode/`) 已被 git 忽略 — 不要提交
- 没有 CI 工作流；验证方式 = `colcon test` + 手动 launch
- 测试入口：`src/filter_test/test/filter_test.cpp` 和
  `src/filter_test/test/test_auto_graph_optimizer.cpp`
- ROS 消息生成在 `install/.../include/...` 下 (不在 `src/` 中)。
  `.vscode/c_cpp_properties.json` 中的 include 路径必须与构建输出位置匹配

---
name: armor-graph-optimizer
description: Use when working with this repository's GTSAM graph optimizer, graph_optimizer_test node, ArmorGraphTracker, ArmorCvPixelGraph, RuneCvGraph, typed variables, custom GTSAM factors, armor/rune observations, cold start behavior, fixed-lag smoother settings, or graph optimizer tuning
---

# typed 图优化器

## 当前链路

当前图优化链路是 typed GTSAM 框架：

```text
GraphOptimizerTest ROS node
  -> ArmorGraphTracker
  -> ArmorCvPixelGraph
  -> auto_graph::GraphOptimizer
  -> GTSAM iSAM2 / fixed-lag smoother
```

不要按历史图优化分层或旧后端语义查代码；当前实现以 typed GTSAM core 为准。

能量机关方向复用同一个 typed core：

```text
RuneCvGraph
  -> auto_graph::GraphOptimizer
  -> GTSAM iSAM2 / fixed-lag smoother
```

当前 `RuneCvGraph`、消息/因子模型和 ROS `rune_graph_optimizer` 节点都已接入；
默认 launch 仍只启动装甲板图优化，能量机关节点按需取消注释。

## 代码入口

```text
src/filter_test/
├── include/filter_test/graph_optimizer/
│   ├── graph_core.hpp       auto_graph::GraphOptimizer, Var<T>, SolveResult
│   ├── graph_math.hpp       angle/logistic/Eigen<->GTSAM helpers
│   ├── armor_model.hpp      typed variables, factors, matching, ArmorCvPixelGraph
│   ├── armor_tracker.hpp    ArmorGraphTracker, FrameTimeTracker, update result
│   └── rune_model.hpp       RuneCvGraph, rune variables, factors, output
├── src/graph_optimizer/
│   ├── graph_core.cpp       iSAM2/fixed-lag lifecycle
│   ├── armor_model.cpp      graph initialization, factors, matching, output
│   ├── armor_tracker.cpp    empty-frame handling, reset, result wrapping
│   └── rune_model.cpp       rune motion, reprojection and geometry factors
├── src/graph_optimizer_test.cpp
├── src/rune_graph_optimizer.cpp
└── test/test_auto_graph_optimizer.cpp
```

## 构建和测试

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

colcon build --packages-select auto_aim_interfaces filter_test --cmake-args -DBUILD_TESTING=ON
source install/setup.bash

colcon test --packages-select filter_test \
  --ctest-args -R test_auto_graph_optimizer --output-on-failure
```

本机内存紧张时限制并发：

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --parallel-workers 1 \
  --packages-select auto_aim_interfaces armor_simulation filter_test \
  --cmake-args -DBUILD_TESTING=ON
```

默认 launch 当前只启动 `graph_optimizer_test`：

```bash
ros2 launch filter_test filter_test.launch.py
```

单独运行：

```bash
ros2 run filter_test graph_optimizer_test \
  --ros-args --params-file src/filter_test/config/config.yaml
```

## 核心职责边界

`auto_graph::GraphOptimizer` 只做通用 typed GTSAM 生命周期：

- 声明动态/静态 typed 变量；
- 将 `Var<T>` 映射到 GTSAM key；
- 管理 `beginInit()`、`finishInit()`、`beginFrame()`、`solve()`；
- 写入 initial values、prior 和 factor；
- 驱动 iSAM2 或 fixed-lag smoother；
- 读取 latest estimate；
- reset 和异常恢复。

不要把装甲板、相机、匹配、ROS、仿真语义放进 `GraphOptimizer`。

`ArmorCvPixelGraph` 持有装甲板业务逻辑：

- 首帧从第一块 armor 初始化中心、yaw、半径和 dz；
- 根据上一帧状态做常速预测；
- 对同一帧观测做唯一 index 匹配；
- 添加 `TranslationFactor`、`VelocityFactor`、`YawFactor`、`VyawFactor`；
- 为每块匹配 armor 添加辅助 `Pose3`、像素重投影和几何因子；
- 从优化结果生成 `TrackerState`、预测 armor 和匹配结果。

`ArmorGraphTracker` 是 ROS-facing 薄层：处理空观测、拒绝帧、求解失败 reset、
`TrackerUpdateResult` 兼容语义。

`RuneCvGraph` 持有能量机关业务逻辑：

- 初始化中心、法线 yaw、roll 和 roll 角速度；
- 用 `RuneRollFactor` / `RuneVrollFactor` 维护 CV roll 运动；
- `RuneTargets` 使用真实 detector 风格的紧凑五点观测数组，不携带物理扇叶编号；
- 先用 5 点 PnP 得到观测叶片 pose，再按 roll/center 门限匹配到 5 个内部物理 slot；
- 对每个匹配成功的物理叶片添加叶片辅助 `Pose3`、五点像素重投影因子和几何因子；
- 输出 `RuneGraphOutput`，包含优化状态、匹配叶片数、内部匹配 slot 和目标点。

## typed 变量和 key

| 变量 | 类型 | 动态 | 前缀 | 含义 |
| --- | --- | --- | --- | --- |
| `center` | `gtsam::Point3` | yes | `x` | 机器人中心 |
| `velocity` | `gtsam::Vector3` | yes | `v` | 中心速度 |
| `yaw` | `gtsam::Rot2` | yes | `r` | 中心 yaw，流形变量 |
| `vyaw` | `double` | yes | `w` | yaw rate |
| `radius_a` | `double` | no | `a` | 偶数装甲板半径的 logistic 状态 |
| `radius_b` | `double` | no | `b` | 奇数装甲板半径的 logistic 状态 |
| `dz` | `double` | no | `z` | 奇偶装甲板高度差 |
| `outpost_radius` | `double` | no | `o` | 前哨站三装甲板共用半径 |
| `outpost_dz_1` | `double` | no | `p` | 前哨站 index 1 高度偏移 |
| `outpost_dz_2` | `double` | no | `q` | 前哨站 index 2 高度偏移 |

动态变量使用当前 frame id。静态变量 key 固定在 frame `0`，但 fixed-lag smoother
会刷新 timestamp，避免半径和 dz 被边际化。

辅助装甲板 pose key 每帧最多 5 个：

```text
index 0 -> h<frame>
index 1 -> j<frame>
index 2 -> k<frame>
index 3 -> l<frame>
index 4 -> m<frame>
```

## 像素观测写图

对每个匹配成功的 armor：

1. 将观测 pose 从 odom 转到 camera frame。
2. 插入该 index 的辅助 `Pose3` 初值。
3. 添加辅助 pose prior，作为局部锚点。
4. 对每个检测角点添加 `ArmorTypedReprojFactor`。
5. 添加几何因子：
   - 偶数 index：`ArmorRadiusCenterZFactor`；
   - 奇数 index：`ArmorRadiusDZFactor`。
6. 四角点齐全时默认添加 auto_aim EdgeLoss 风格 direct reprojection：
   - 偶数 index：`ArmorEdgeCenterZReprojFactor` 直接连接 `radius_a/yaw/center`；
   - 奇数 index：`ArmorEdgeDZReprojFactor` 直接连接 `radius_b/dz/yaw/center`；
   - 每条边输出 1 维残差，混合边方向角度误差、顶点位移和边长差的归一化误差。

`Armor.number=="outpost"` 时切换为前哨站三装甲板模型：

- yaw offset 为 `0/2π/3/4π/3`，三块共用 `outpost_radius`。
- index 1/2 分别使用 `outpost_dz_1/outpost_dz_2`，index 0 与中心同高。
- 几何因子使用 `ArmorOutpostGeometryFactor`。
- direct edge 使用 `ArmorEdgeOutpostReprojFactor` 和 `outpost_armor_pitch`。
- 仿真器将物理 index 写入 `Armor.priority`；初始化时用它反推 center yaw 和 center z。

角点约定：

```text
local frame: X=normal, Y=width, Z=height
corner order: left-bottom, left-top, right-top, right-bottom
```

几何约定：

```text
armor_position = center - radius * [cos(armor_yaw), sin(armor_yaw), 0]
index 0 yaw = center_yaw
index 1/2/3 yaw = center_yaw + 90/180/270 deg
outpost index 0/1/2 yaw = center_yaw + 0/120/240 deg
```

## 匹配

匹配是启发式，不使用图协方差：

```text
cost = yaw_diff + 3.0 * position_error
if candidate index == last_armor_index:
    cost *= 0.85
reject if cost > match_max_cost
```

`matchArmorIndicesUnique()` 保证单帧内不会把多个观测分配到同一个物理 index。
未匹配观测标为 `-1`，写图时跳过，避免辅助 Pose3 key 冲突。

不要直接替换成马氏距离，除非同时定义 covariance ownership、调用层语义和测试。

## 能量机关图模型

文件：`graph_optimizer/rune_model.hpp/.cpp`

typed 变量：

| 变量 | 类型 | 动态 | 前缀 | 含义 |
| --- | --- | --- | --- | --- |
| `center` | `gtsam::Point3` | yes | `c` | 能量机关中心 |
| `roll` | `gtsam::Rot2` | yes | `r` | 当前叶片 roll |
| `vroll` | `double` | yes | `w` | roll 角速度 |
| `normal_yaw` | `gtsam::Rot2` | no | `n` | 平面法线 yaw |

辅助叶片 pose key 每帧最多 5 个：

```text
index 0 -> q<frame>
index 1 -> s<frame>
index 2 -> t<frame>
index 3 -> u<frame>
index 4 -> y<frame>
```

`RuneTarget` 像素顺序固定为：

```text
r_center, near_point, left_point, far_point, right_point
```

每个有效叶片：

1. 用 5 点 `r_center/near/left/far/right` 做 PnP。
2. 冷启动时第一片有效观测锚定为内部 slot 0；同帧其他观测按相对 roll 推断 slot。
3. 已有状态时用观测 roll/center 与预测的 5 个 slot 做门限匹配；同一帧 slot 必须唯一。
4. PnP 成功时用 PnP pose 插入辅助 `Pose3`
   并添加较强 prior，失败时用预测 pose 和较弱 prior。
5. 5 个点分别添加 `RuneBladeReprojFactor`。
6. 添加 `RuneBladeGeometryFactor` 连接辅助 pose、`center`、`roll`、`normal_yaw`，
   残差包含位置、roll 和 yaw；`normal_pitch` 仍是固定配置，不是图变量。
7. 默认添加 `RuneBladeDirectReprojFactor`，直接连接 `center/roll/normal_yaw`。
   该因子参考 auto_aim `RuneRollLoss`，只用 `near/left/far/right` 四点的两条对角线：
   角度差约束 roll，归一化长度差约束尺度/yaw。

大符预测：

- `RuneCvGraph` 输出优化后的连续 roll 和 vroll。
- `rune_graph_optimizer` 节点维护 `RuneBigCurveFitter`，拟合
  `angle(t) = -a*cos(w*(t+t0)) + b*t + c`。
- 拟合尚不可用时使用 `roll + vroll * predict_dt`；拟合可用后用曲线预测 roll 和角速度。
- `/rune_graph_optimizer/marker` 发布三类可视化：
  `rune_center` 用 `SPHERE` 表示估计中心，`rune_target` 用 `SPHERE` 表示预测目标点，
  `rune` 用 5 组 `SPHERE + LINE_STRIP` 表示五片拟合扇叶，
  `observed_position` 用 `SPHERE` 表示观测扇叶，`rune_status` 用 `TEXT_VIEW_FACING`
  显示 roll 和观测数量。
  `rune_center/rune_target` 的 orientation 使用 `normal_yaw` 和固定 `normal_pitch`。
  `rune_simulation_node` 的 marker 应与这套 namespace/type 保持一致。
- `/rune_graph_optimizer/target_pose` 的 orientation 同样使用 `normal_yaw/normal_pitch`。

运动因子：

- `RuneRollFactor`：`roll(k-1) + vroll(k-1) * dt -> roll(k)`，使用 `Rot2` 最短角残差。
- `RuneVrollFactor`：`vroll(k) - vroll(k-1)`。

## ROS 参数

Namespace：`/graph_optimizer_test`

求解器：

- `verbose`
- `cold_start_frames`
- `smoother_lag`
- `smoother_type`: `"incremental"` 或 `"batch"`
- `extra_iterations`: ROS 名称，内部映射为 `update_iterations`
- `relinearize_threshold`

运动和平滑：

- `s2qxy`
- `s2qz`
- `s2qyaw`
- `vel_sigma`
- `vyaw_sigma`

观测：

- `match_max_cost`
- `pixel_noise_std`
- `use_edge_reproj_factor`
- `edge_reproj_sigma`
- `edge_loss_slope_k`
- `standard_armor_pitch`
- `outpost_armor_pitch`
- `outpost_radius`
- `outpost_dz_1`
- `outpost_dz_2`
- `geo_tangential`
- `geo_radial`
- `geo_height`
- `geo_yaw`

相机内参：

- `camera_name`
- `camera_info_url`

外参来自 TF：`odom -> camera_optical_frame`。`use_2d_observation` 参数只为旧配置兼容保留；
当前 typed pixel graph 路径固定启用。

## 修改规则

- 保持 `GraphOptimizer` 泛型，不引入 armor/camera/ROS 依赖。
- `dt` 留在 tracker 或运动因子构造中，不放进 `beginFrame()`。
- 静态半径和 dz key 固定 frame 0；改 fixed-lag 行为时必须确认 timestamp 刷新。
- 保留 ROS 参数名，除非同步提供迁移方案。
- 偶数和奇数几何因子 key arity 不同，不要强行合并成一个带零 Jacobian key 的大因子。
- 偶数和奇数 direct edge reprojection 因子 key arity 也不同；不要把 `dz` 挂到偶数装甲板上。
- auto_aim EdgeLoss 只移植残差设计，不要把 Ceres optimizer 塞进 `GraphOptimizer` core。
- 写新因子时先在 `test_auto_graph_optimizer.cpp` 覆盖残差、角度 wrap 和必要 Jacobian。
- 改匹配逻辑时覆盖唯一匹配、离群拒绝、overflow 观测标 `-1`、tracker reset 语义。
- 改发布行为时确认 `TrackerUpdateResult::accepted_frame/solved/cold_start/solve_failed` 兼容。

## 常见故障

- 缺少 `/usr/local/lib`：GTSAM 动态库加载失败。
- 缺少 `odom -> camera_optical_frame`：初始化之后图优化跳过更新。
- `cold_start_frames > 0`：早期帧可能 accepted 但不发布优化结果。
- 全部观测 unmatched：tracker reset，等待重新初始化。
- 同一帧重复辅助 Pose3 key：GTSAM insert/key conflict。
- `geo_yaw` 过小：可能导致 yaw 约束过强、条件数变差或奇异。
- `edge_reproj_sigma` 过小：direct edge loss 可能压过 pose/几何因子，先增大或关闭 `use_edge_reproj_factor` 对比。
- `match_max_cost` 过小：有效观测被拒绝，半径/dz 难收敛。

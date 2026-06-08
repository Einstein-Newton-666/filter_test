---
name: armor-graph-optimizer
description: Use when working with this repository's GTSAM armor graph optimizer, graph_optimizer_test node, ArmorGraphTracker, ArmorCvPixelGraph, typed variables, custom GTSAM factors, armor matching, cold start behavior, fixed-lag smoother settings, or graph optimizer tuning
---

# 装甲板图优化器

## 当前链路

当前图优化链路是 typed GTSAM 框架：

```text
GraphOptimizerTest ROS node
  -> ArmorGraphTracker
  -> ArmorCvPixelGraph
  -> auto_graph::GraphOptimizer
  -> GTSAM iSAM2 / fixed-lag smoother
```

不要按旧的 `auto_graph_optimizer/`、`tracker_core.hpp`、`observation_backend.hpp`
分层查代码；这些名字属于历史实现或旧文档。

## 代码入口

```text
src/filter_test/
├── include/filter_test/graph_optimizer/
│   ├── graph_core.hpp       auto_graph::GraphOptimizer, Var<T>, SolveResult
│   ├── graph_math.hpp       angle/logistic/Eigen<->GTSAM helpers
│   ├── armor_model.hpp      typed variables, factors, matching, ArmorCvPixelGraph
│   └── armor_tracker.hpp    ArmorGraphTracker, FrameTimeTracker, update result
├── src/graph_optimizer/
│   ├── graph_core.cpp       iSAM2/fixed-lag lifecycle
│   ├── armor_model.cpp      graph initialization, factors, matching, output
│   └── armor_tracker.cpp    empty-frame handling, reset, result wrapping
├── src/graph_optimizer_test.cpp
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

动态变量使用当前 frame id。静态变量 key 固定在 frame `0`，但 fixed-lag smoother
会刷新 timestamp，避免半径和 dz 被边际化。

辅助装甲板 pose key 每帧最多 4 个：

```text
index 0 -> h<frame>
index 1 -> j<frame>
index 2 -> k<frame>
index 3 -> l<frame>
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
- `geo_tangential`
- `geo_radial`
- `geo_height`
- `geo_yaw`

相机内参：

- `camera_fx`
- `camera_fy`
- `camera_cx`
- `camera_cy`
- `distortion_k1`
- `distortion_k2`
- `distortion_p1`
- `distortion_p2`

外参来自 TF：`odom -> camera_optical_frame`。`use_2d_observation` 参数只为旧配置兼容保留；
当前 typed pixel graph 路径固定启用。

## 修改规则

- 保持 `GraphOptimizer` 泛型，不引入 armor/camera/ROS 依赖。
- `dt` 留在 tracker 或运动因子构造中，不放进 `beginFrame()`。
- 静态半径和 dz key 固定 frame 0；改 fixed-lag 行为时必须确认 timestamp 刷新。
- 保留 ROS 参数名，除非同步提供迁移方案。
- 偶数和奇数几何因子 key arity 不同，不要强行合并成一个带零 Jacobian key 的大因子。
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
- `match_max_cost` 过小：有效观测被拒绝，半径/dz 难收敛。

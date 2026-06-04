# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

ROS2 **Humble** C++17 项目 (GCC 11, Ubuntu 22.04)，用于使用卡尔曼滤波和因子图优化进行实时装甲板跟踪。包含三个主要 package：

1. **armor_simulation**：独立仿真器包（3D 运动 + 相机投影 + 噪声 + PnP + 云台）
2. **filter_test**：滤波器和图优化测试包（EKF/UKF + GTSAM 因子图优化）
3. **auto_aim_interfaces**：装甲板消息接口定义

**详细文档：**
- `docs/auto_graph_optimizer_architecture.md` — 图优化框架架构与原理
- `docs/auto_graph_optimizer_usage.md` — 图优化框架使用指南
- `AGENTS.md` — 高信噪比精简摘要（给其他 AI agent 用）
- `.claude/skills/armor-filter/SKILL.md` — 滤波器专项知识
- `.claude/skills/armor-simulator/SKILL.md` — 仿真器专项知识

## 构建命令

```bash
# 基础构建
colcon build --packages-select armor_simulation filter_test auto_aim_interfaces

# 构建并启用测试
colcon build --packages-select armor_simulation filter_test --cmake-args -DBUILD_TESTING=ON

# 运行测试（两个测试可执行文件）
colcon test --packages-select filter_test
colcon test-result --verbose      # 查看失败详情

# 运行单个测试
colcon test --packages-select filter_test --ctest-args -R "filter_test"
colcon test --packages-select filter_test --ctest-args -R "test_auto_graph_optimizer"

# 环境设置
source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH  # GTSAM 4.3 装在 /usr/local/lib
```

**测试入口：**
- `src/filter_test/test/filter_test.cpp` — 滤波器单元测试
- `src/filter_test/test/test_auto_graph_optimizer.cpp` — 图优化器测试（当前仅测试不依赖 GTSAM 的部分）

## 运行方式

```bash
# 全启动 (仿真器 + 云台 + 滤波器 + 图优化器 + angle_solver)
ros2 launch filter_test filter_test.launch.py

# 带参数
ros2 launch filter_test filter_test.launch.py use_graph_optimizer:=true

# 仅仿真器 (不含滤波器和图优化)
ros2 launch armor_simulation simulation.launch.py

# 单独节点
ros2 run armor_simulation armor_simulation_node
ros2 run armor_simulation gimbal_simulation
ros2 run filter_test filter
ros2 run filter_test graph_optimizer_test --ros-args -p use_2d_observation:=true
```

## 架构设计

### 项目结构

```
src/
├── armor_simulation/              # 仿真器 package (独立)
│   ├── include/armor_simulation/
│   │   ├── armor_simulation.hpp   # 3D 运动仿真器
│   │   ├── camera_model.hpp       # 相机投影 + IPPE PnP
│   │   ├── armor_geometry.hpp     # 装甲板角点几何
│   │   └── detection_noise.hpp    # 检测噪声 (U 形距离模型)
│   ├── src/
│   │   ├── armor_simulation.cpp   # 主仿真节点
│   │   ├── gimbal_simulation.cpp  # 云台仿真 (S-curve + TF)
│   │   ├── angle_solver.cpp       # 角度解算器
│   ├── config/                    # 仿真参数
│   └── msg/GroundTruth.msg        # 真值消息
│
├── filter_test/                   # 滤波器 + 图优化
│   ├── include/filter_test/
│   │   ├── auto_graph_optimizer/  # 图优化框架 (6 文件)
│   │   │   ├── graph_optimizer.hpp        # VariableLayout + 因子 + GraphOptimizer
│   │   │   ├── auto_graph_optimizer.hpp   # umbrella header
│   │   │   ├── models/
│   │   │   │   ├── motion_model.hpp       # MotionModel CRTP 基类
│   │   │   │   └── measure_model.hpp      # MeasureModel CRTP 基类
│   │   │   └── utils/
│   │   │       ├── helpers.hpp            # logistic 函数, 角度工具
│   │   │       └── types.hpp              # 类型定义
│   │   ├── graph_optimizer/       # graph_optimizer_test 内部 tracker core
│   │   │   ├── tracker_core.hpp           # ArmorGraphTracker: init/match/build/solve
│   │   │   ├── observation_backend.hpp    # 2D/YPD 观测后端接口
│   │   │   ├── factors.hpp                # ArmorReproj/Center/Smooth 因子
│   │   │   ├── motion_models.hpp          # 2D/YPD 模式使用的 CRTP 模型
│   │   │   ├── tracker_math.hpp           # 半径、yaw、几何/匹配工具
│   │   │   └── tracker_types.hpp          # tracker 输入/输出/配置结构
│   │   ├── filters/               # 传统滤波器
│   │   │   ├── kalman_filter.hpp          # KF 模板基类 <N_X, N_Z>
│   │   │   ├── extended_kalman.hpp        # EKF (Ceres autodiff)
│   │   │   ├── unscented_kalman.hpp       # UKF
│   │   │   ├── cv_model.hpp              # CV 模型 (11D)
│   │   │   └── singer_model.hpp          # Singer 模型 (15D)
│   │   ├── filter.hpp            # ArmorFilter 业务逻辑类
│   │   ├── filter_test.hpp        # ArmorTest ROS2 节点
│   │   ├── graph_optimizer_test.hpp       # GraphOptimizerTest ROS 节点外壳 + 兼容别名
│   │   └── common.hpp            # 枚举 / 角度工具
│   └── src/
│       ├── filter.cpp            # 传统滤波器入口
│       ├── graph_optimizer/      # tracker core / backend 源文件
│       └── graph_optimizer_test.cpp       # 图优化 ROS 节点入口
│
└── auto_aim_interfaces/           # 消息定义 (Armors, Armor, TrackerTarget 等)
```

### 核心组件

#### 传统滤波器 (EKF/UKF)
- **`KalmanFilter<N_X, N_Z>`** (`filters/kalman_filter.hpp`): KF 模板基类，predict/update 接口
- **`ExtendedKalmanFilter`** / **`UnscentedKalmanFilter`**: EKF/UKF 实现
- **CV 模型** (`cv_model.hpp`): 11D 状态 `[xc,vxc,yc,vyc,za,vza,yaw,vyaw,r1,r2,dz]`
- **Singer 模型** (`singer_model.hpp`): 15D 状态
- **`ArmorFilter`** (`filter.hpp`): 滤波器业务逻辑（init/predict/update/matchArmor）

#### 图优化框架

**`GraphOptimizer`** (`graph_optimizer.hpp`) — iSAM2 增量因子图优化

核心设计：
- `VariableLayout` — 全状态向量 ↔ GTSAM 子变量拆分 (gather/scatter)
- 当前 11D 状态 → 4 组: `pos_vel`(6D), `yaw_vyaw`(2D), `radius`(2D,静态), `dz`(1D,静态)
- `SingleMotionFactor` — per-group 2-key 运动因子 (autodiff)
- `AutoMotionFactor` — 全状态 2-key 运动因子 (autodiff, 兼容 YPD 模式)
- `AutoMeasureFactor` — 全状态 N-key 观测因子 (autodiff)
- 2D 模式额外有 `VelSmoothFactor` / `VyawSmoothFactor`，由 `vel_sigma` / `vyaw_sigma` 控制速度平滑

**`graph_optimizer_test` 节点结构**：
- `GraphOptimizerTest` (`graph_optimizer_test.cpp/.hpp`) 只负责参数、订阅、TF、发布，外部 executable/node/topic/参数 namespace 保持不变
- `ArmorGraphTracker` (`graph_optimizer/tracker_core.hpp`) 持有 `auto_graph::GraphOptimizer`，管理初始化、frame id、装甲板匹配、运动因子、观测因子和 solve
- `ObservationBackend` (`graph_optimizer/observation_backend.hpp`) 隔离两种观测路径；`PixelObservationBackend` 添加 Pose3 初值、角点重投影、中心几何和速度平滑因子，`YpdObservationBackend` 保留单板/双板 YPD 观测
- `tracker_math.hpp` 中的 `matchArmorIndicesUnique()` 保证单帧最多 4 个有效装甲板 index；超出的观测标为 `-1`，2D 后端跳过以避免重复 Pose3 key
- `GraphOptimizer::solve()` 返回 `SolveResult`：冷启动为 `cold_start=true, optimized=false` 且继续累计因子；iSAM2 异常为 `failed=true` 并清空本轮增量图。`TrackerUpdateResult::solved` 表示该帧可按兼容策略发布，`solve_failed=true` 的帧不发布目标/marker。

两级像素观测 (对齐 jlu_vision_26)：
- **第一级**: `ArmorReprojFactor` — 1-key Pose3(camera 系) → 2D 像素误差 (每角点), GTSAM Cal3DS2 投影+畸变
- **第二级**: `ArmorCenterFactor` — 5-key 几何约束 [切向, 径向, z, yaw], armor_yaw 定义切向/径向方向, T_camera_to_odom

API 流程：
```
// YPD 模式 (单层)
opt.predict<ArmorCVMotionModel>(dt, Q);
opt.update<ArmorCVMeasureYPD>(z, R);
opt.solve();

// 2D 像素模式 (两级)
opt.advanceFrame(dt);
opt.addMotionFactor<TranslationModel>("pos_vel", model, Q_trans);
opt.addMotionFactor<YawModel>("yaw_vyaw", model, Q_yaw);
opt.addCustomFactor<ArmorReprojFactor>(...);   // 每个角点
opt.addCustomFactor<ArmorCenterFactor>(...);   // 每个装甲板
opt.solve();
```

自定义模型只需定义 CRTP `operator()<T>` 模板，Ceres Jet 自动求导。

#### 云台仿真
- `gimbal_simulation.cpp` — S-curve 动力学 + TF 广播 (odom→gimbal_link→camera_optical_frame)
- `use_ground_truth_tracking: true` — 真值追踪模式，绕过 tracker 闭环，直接订阅 GroundTruth 计算云台角度

### 状态向量索引

**CV 模型 (11D)**: `[xc, vxc, yc, vyc, za, vza, yaw, vyaw, r1_u, r2_u, dz]`
- 位置: x[0], x[2], x[4]; 速度: x[1], x[3], x[5]
- yaw: x[6], vyaw: x[7]; 半径(logistic): x[8], x[9]; dz: x[10]

**Singer 模型 (15D)**: `[x, vx, ax, y, vy, ay, z, vz, az, yaw, vyaw, ayaw, r1, r2, dz]`

> ⚠️ 这些索引在 `cv_model.hpp`、`singer_model.hpp`、`graph_optimizer.hpp::VariableLayout` 等多个文件中被引用，修改需格外小心并全局搜索。

### ROS2 接口

**订阅**:
- `/detector/armors` (Armors): 检测器/仿真器数据 (GraphOptimizerTest 和 ArmorTest 订阅)
- `/armor_simulation/ground_truth` (GroundTruth): 机器人真值 (云台真值追踪用)

**发布**:
- `/graph_optimizer/armors` (Armors): 图优化跟踪结果
- `/tracker/target` (TrackerTarget): 跟踪目标 (供 angle_solver 使用)
- `/track_result` (Result): 滤波器跟踪结果
- `/track_result/marker` (MarkerArray): RViz 可视化

### 配置文件

> **原则：修改配置文件而非头文件默认值。**

| 文件 | 控制节点 | 关键参数 |
|------|---------|---------|
| `src/filter_test/config/config.yaml` | `/filter`, `/graph_optimizer_test`, `/gimbal_simulation` | 滤波器选择、过程/观测噪声、相机内参、2D/YPD 模式 |
| `src/armor_simulation/config/simulation_config.yaml` | `/armor_simulation_node` | 初始状态、运动噪声、径向距离约束、运动限幅、几何参数、像素噪声 U 形模型、离群点 |

**图优化关键参数** (`config.yaml` - `/graph_optimizer_test`)：
- `use_2d_observation`: 观测模式 (true=两级像素, false=YPD)
- `cold_start_frames`: 冷启动帧数 (前 N 帧只累积不优化)
- `s2qxy/s2qz/s2qyaw/s2qr/s2qdz`: 过程噪声
- `s2qvel/s2qvyaw`: per-group 运动因子中的速度方差
- `vel_sigma/vyaw_sigma`: `VelSmoothFactor` / `VyawSmoothFactor` 标准差
- `r_pose/r_distance/r_yaw`: YPD 观测噪声
- `pixel_noise_std`: 像素噪声 σ
- `geo_tangential/radial/height/yaw`: 几何约束噪声
- `camera_fx/fy/cx/cy`, `distortion_k1/k2/p1/p2`: 相机参数 (外参从 TF 动态获取)

**仿真/PnP 当前约定**：
- 装甲板局部坐标: `X=法线, Y=宽度, Z=高度`; 角点顺序 `[左下, 左上, 右上, 右下]`
- 仿真装甲板位置: `pos = center - r * [cos(armor_yaw), sin(armor_yaw)]`, 可视化 pitch 为 `+15°`
- `CameraModel::estimatePose()` 使用 OpenCV `SOLVEPNP_IPPE`; 平面 PnP 仍可能有两个相近候选，仿真侧用 `correctPlanarPnPAmbiguity()` 按 yaw 先验修正
- 像素噪声为 U 形距离模型 + 装甲板级相关噪声: `p_i' = p_i + n_c + n_i`, `pixel_noise_common_ratio` 越大 yaw 越稳

## 依赖项

- ROS2 Humble (rclcpp, tf2, geometry_msgs, visualization_msgs)
- Eigen3, OpenCV, Ceres Solver
- GTSAM 4.3 (iSAM2, Cal3DS2, PinholeCamera) — 装在 `/usr/local/lib`
- auto_aim_interfaces

## 约定

- 头文件布局：`include/<package>/...` → 以 `"<package>/..."` 引入
- 所有 launch 文件中 exec 输出：`output='screen'`
- 构建产物 (`build/`、`install/`、`log/`) 和 `.vscode/` 已被 git 忽略 — 不要提交
- 没有 CI 工作流；验证方式 = `colcon test` + 手动 launch
- `.vscode/c_cpp_properties.json` 硬编码了 `/opt/ros/humble` 路径，如果 ROS 发行版不同需修改
- 当前 git 状态有未跟踪文件 (`AGENTS.md`, `docs/`, `.bak`) 和未提交修改

## 已知问题

- r2 过冲仍需重新调参；当前已有 `VelSmoothFactor`/`VyawSmoothFactor`，但半径估计仍依赖观测质量和几何约束
- yaw 在部分传统滤波路径仍按平坦标量处理；图优化 `ArmorCenterFactor` 已对 yaw residual 做角度包裹
- 匹配为启发式代价 (非马氏距离)
- UKF 收敛速度比 EKF 慢
- Singer 模型可能产生 NaN 值

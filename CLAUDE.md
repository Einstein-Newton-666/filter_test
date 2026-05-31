# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

ROS2 C++项目，用于使用卡尔曼滤波和因子图优化进行实时装甲板跟踪。包含三个主要 package：

1. **armor_simulation**：独立仿真器包（3D 运动 + 相机投影 + 噪声 + PnP + 云台）
2. **filter_test**：滤波器和图优化测试包（EKF/UKF + GTSAM 因子图优化）
3. **auto_aim_interfaces**：装甲板消息接口定义

## 构建命令

```bash
colcon build --packages-select armor_simulation filter_test auto_aim_interfaces
colcon build --packages-select armor_simulation filter_test --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select filter_test
source install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH  # GTSAM 4.3
```

## 运行方式

```bash
# 全启动 (仿真器 + 云台 + 滤波器 + 图优化器)
ros2 launch filter_test filter_test.launch.py

# 带参数
ros2 launch filter_test filter_test.launch.py use_graph_optimizer:=true

# 单独节点
ros2 run armor_simulation armor_simulation_node
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
│   │   ├── camera_model.hpp       # 相机投影 + PnP
│   │   ├── armor_geometry.hpp     # 装甲板角点几何
│   │   └── detection_noise.hpp    # 检测噪声 (U 形距离模型)
│   ├── src/
│   │   ├── armor_simulation.cpp   # 主仿真节点
│   │   ├── gimbal_simulation.cpp  # 云台仿真 (S-curve + TF)
│   │   ├── angle_solver.cpp       # 角度解算器
│   │   └── camera_simulator.cpp   # 2D 相机仿真器
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
│   │   ├── filters/               # 传统滤波器
│   │   │   ├── kalman_filter.hpp          # KF 模板基类 <N_X, N_Z>
│   │   │   ├── extended_kalman.hpp        # EKF (Ceres autodiff)
│   │   │   ├── unscented_kalman.hpp       # UKF
│   │   │   ├── cv_model.hpp              # CV 模型 (11D)
│   │   │   └── singer_model.hpp          # Singer 模型 (15D)
│   │   ├── filter.hpp            # ArmorFilter 业务逻辑类
│   │   ├── filter_test.hpp        # ArmorTest ROS2 节点
│   │   ├── graph_optimizer_test.hpp       # GraphOptimizerTest + 因子定义
│   │   └── common.hpp            # 枚举 / 角度工具
│   └── src/
│       ├── filter.cpp            # 传统滤波器入口
│       └── graph_optimizer_test.cpp       # 图优化入口
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

### ROS2 接口

**订阅**:
- `/detector/armors` (Armors): 检测器/仿真器数据 (GraphOptimizerTest 和 ArmorTest 订阅)
- `/armor_simulation/ground_truth` (GroundTruth): 机器人真值 (云台真值追踪用)

**发布**:
- `/graph_optimizer/armors` (Armors): 图优化跟踪结果
- `/tracker/target` (TrackerTarget): 跟踪目标 (供 angle_solver 使用)
- `/track_result` (Result): 滤波器跟踪结果
- `/track_result/marker` (MarkerArray): RViz 可视化

### 配置参数

#### 图优化 (`config.yaml` - /graph_optimizer_test)
- `use_2d_observation`: 观测模式 (true=两级像素, false=YPD)
- `cold_start_frames`: 冷启动帧数 (前 N 帧只累积不优化)
- `s2qxy/s2qz/s2qyaw/s2qr/s2qdz`: 过程噪声
- `r_pose/r_distance/r_yaw`: YPD 观测噪声
- `pixel_noise_std`: 像素噪声 σ
- `geo_tangential/radial/height/yaw`: 几何约束噪声
- `camera_fx/fy/cx/cy`, `distortion_k1/k2/p1/p2`: 相机参数

#### 云台 (`config.yaml` - /gimbal_simulation)
- `use_ground_truth_tracking`: 真值追踪模式

#### 仿真器 (`simulation_config.yaml`)
- `initial_state`: 机器人初始位置/速度/加速度
- `process_noise_xy/yaw`: 随机运动噪声
- `geometry`: r1/r2/z1/z2 几何参数
- `pixel_noise_*`, `detection_probability`, `use_outliers`

## 依赖项

- ROS2 (rclcpp, tf2, geometry_msgs, visualization_msgs)
- Eigen3, OpenCV, Ceres Solver
- GTSAM 4.3 (iSAM2, Cal3DS2, PinholeCamera)
- auto_aim_interfaces

## 已知问题

- r2 过冲 (缺少独立 VelocityFactor 速度正则化)
- yaw 为平坦标量 (无 SO(2) 角度包裹)
- 匹配为启发式代价 (非马氏距离)
- UKF 收敛速度比 EKF 慢
- Singer 模型可能产生 NaN 值

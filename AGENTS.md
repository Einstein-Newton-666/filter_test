# AGENTS.md — filter_test

ROS2 C++ 工作区：基于 EKF/UKF + GTSAM 因子图优化的实时装甲板跟踪。`src/` 下三个包：

| 包名                   | 作用                                                             |
|------------------------|------------------------------------------------------------------|
| `armor_simulation`     | 3D 运动仿真 + 相机投影 + 噪声 + PnP + 云台仿真                  |
| `filter_test`          | EKF/UKF (`filter`) + iSAM2 图优化 (`graph_optimizer_test`)      |
| `auto_aim_interfaces`  | 共享的 `Armors` / `Armor` / `TrackerTarget` 消息                |

详细架构/用法见 `CLAUDE.md` 和 `docs/auto_graph_optimizer_*.md`。
本文件是高信噪比的精简摘要。

## 编译与运行

```bash
# 编译
colcon build --packages-select armor_simulation filter_test auto_aim_interfaces

# 编译并启用测试
colcon build --packages-select armor_simulation filter_test --cmake-args -DBUILD_TESTING=ON

# 运行测试
colcon test --packages-select filter_test
colcon test-result --verbose

# 必填：GTSAM 4.3 装在 /usr/local/lib (不在默认链接路径)
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
source install/setup.bash

# 完整启动 (sim + gimbal + filter + graph_optimizer + angle_solver)
ros2 launch filter_test filter_test.launch.py

# 单独运行节点
ros2 run armor_simulation armor_simulation_node
ros2 run armor_simulation gimbal_simulation
ros2 run filter_test filter
ros2 run filter_test graph_optimizer_test --ros-args -p use_2d_observation:=true
```

ROS 发行版：**Humble** (`.vscode/c_cpp_properties.json` 硬编码 `/opt/ros/humble` 路径)。
C++17，GCC 11 (Ubuntu 22.04)。

## 状态向量索引 (filter 与 graph_optimizer 通用)

**CV 模型 (11D)** — `[xc, vxc, yc, vyc, za, vza, yaw, vyaw, r1_u, r2_u, dz]`
- 位置: 0,2,4  速度: 1,3,5  Yaw: 6  vyaw: 7
- 半径 (logistic 编码, 静态): 8,9  dz (静态): 10

**Singer 模型 (15D)** — `[x, vx, ax, y, vy, ay, z, vz, az, yaw, vyaw, ayaw, r1, r2, dz]`

这些索引在多个头文件中被引用 (`cv_model.hpp`、`singer_model.hpp`、
`graph_optimizer.hpp::VariableLayout` 等) — 修改需格外小心。

## 图优化框架 — `auto_graph_optimizer/`

两种观测模式 (在 `config.yaml` 中通过 `use_2d_observation` 切换)：

- **YPD 模式** (`false`)：单层 3-key 因子，使用 Y/P/D 观测量
- **2D 像素模式** (`true`)：两级因子图
  1. `ArmorReprojFactor` — 1-key Pose3(camera) → 每个角点的 2D 像素误差，使用 GTSAM
     `Cal3DS2` 投影 + 畸变
  2. `ArmorCenterFactor` — 5-key 几何约束 `[tangential, radial, z, yaw]`，
     `armor_yaw` 决定切向/径向方向，camera→odom 坐标变换

`VariableLayout` 将 11D 状态拆分为 GTSAM 子变量：
`pos_vel` (6D, 动态)、`yaw_vyaw` (2D, 动态)、`radius` (2D, 静态)、`dz` (1D, 静态)。

自定义运动/观测模型遵循 **CRTP**：定义 `operator()<T>`，Ceres Jet 自动微分处理雅可比。

`gimbal_simulation` 读取 `use_ground_truth_tracking: true` 来绕过跟踪器闭环，
直接从 GroundTruth 话题驱动。

## 配置文件 (修改这些，而不是头文件默认值)

- `src/filter_test/config/config.yaml` — `/filter` (CV/Singer、EKF/UKF、R 参数) 和
  `/graph_optimizer_test` (观测模式、像素 σ、几何噪声、相机内参)
- `src/armor_simulation/config/simulation_config.yaml` — 初始状态、运动限制、几何参数、
  像素噪声 U 形模型、离群点、相机内参

## 已知问题 (未经重新调参请勿"修复")

- `r2` (奇数装甲板半径) 过冲 — 需要独立的 `VelocityFactor` (尚未实现；通过
  `s2qvel` 进行速度正则化是部分替代方案)
- `yaw` 按标量处理 — 没有 SO(2) 角度环绕
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

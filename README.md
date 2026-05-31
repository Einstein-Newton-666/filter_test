# filter_test - 装甲板跟踪滤波器测试包

基于ROS2的装甲板跟踪滤波器测试平台，实现了EKF和UKF滤波算法，支持CV（匀速）和Singer两种运动模型。

## 功能特性

- 支持两种滤波算法：EKF（扩展卡尔曼滤波）和UKF（无迹卡尔曼滤波）
- 支持两种运动模型：CV（11维状态）和Singer（15维状态）
- 支持模拟器和真实检测器两种数据源
- 自适应量测噪声（仿真模式固定R，检测器模式距离相关R）
- 可视化：RViz中显示跟踪结果（位置、速度、装甲板）
- 提供误差分析：滤波结果与真值的差值计算

## 依赖项

- ROS2 (Humble/Iron/Jazzy)
- Eigen3
- OpenCV
- Ceres Solver
- auto_aim_interfaces（自定义消息包）

## 安装

```bash
# 创建工作空间
mkdir -p ~/filter_ws/src
cd ~/filter_ws/src

# 克隆代码
git clone <your-repo-url> filter_test
git clone <auto-aim-interfaces-repo> auto_aim_interfaces

# 安装依赖
cd ~/filter_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --packages-select auto_aim_interfaces filter_test

# 加载环境
source install/setup.bash
```

## 使用方法

### 启动滤波器和模拟器

```bash
ros2 launch filter_test filter_test.launch.py
```

### 单独运行节点

```bash
# 运行滤波器（使用配置文件）
ros2 run filter_test filter --ros-args --params-file src/filter_test/config/config.yaml

# 运行模拟器
ros2 run filter_test armor_simulation --ros-args --params-file src/filter_test/config/config.yaml
```

### 配置参数编辑

编辑 `src/filter_test/config/config.yaml` 文件：

```yaml
/filter:
  ros__parameters:
    # 数据源选择
    use_simulation: true   # true=模拟器, false=检测器

    # 滤波器选择
    use_ekf: false         # true=EKF, false=UKF
    use_cv_model: false    # true=CV模型, false=Singer模型

    # 过程噪声参数（CV模型）
    s2qxy_cv: 0.1
    s2qz_cv: 0.1
    s2qyaw_cv: 0.1
    s2qr_cv: 10.0
    s2qdz_cv: 0.1

    # 量测噪声参数
    r_pose_sim: 0.001     # 仿真模式
    r_distance_sim: 0.001
    r_yaw_sim: 0.001
    r_pose_det: 0.01      # 检测器模式
    r_distance_det: 0.01
    r_yaw_det: 0.01
```

## ROS2 话题

### 订阅

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/armor_simulation` | `filter_test/Simulation` | 模拟器数据（含真值） |
| `/detector/armors` | `auto_aim_interfaces/Armors` | 检测器数据 |

### 发布

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/track_result` | `filter_test/Result` | 滤波后的状态估计 |
| `/track_result/marker` | `visualization_msgs/MarkerArray` | RViz可视化 |

## 运行测试

```bash
# 编译测试
colcon build --packages-select filter_test --cmake-args -DBUILD_TESTING=ON

# 运行测试
colcon test --packages-select filter_test
colcon test-result --verbose
```

## 状态向量说明

### CV模型（11维）

```
[xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r1, r2, dz]
 0    1     2    3     4   5     6    7      8   9   10
```

### Singer模型（15维）

```
[x, vx, ax, y, vy, ay, z, vz, az, yaw, v_yaw, a_yaw, r1, r2, dz]
 0  1   2   3  4   5   6  7   8   9    10     11    12  13  14
```

## 已知问题

- UKF收敛速度比EKF慢，需要更多迭代才能稳定
- Singer模型在某些参数下可能产生NaN值
- 模拟器偶尔出现数值跳变（已部分修复）

## 相关论文

- Singer模型参考：Singer, R. A. (1970). Estimating optimal tracking filter performance for manned maneuvering targets.
- UKF参考：Wan, E. A., & Van Der Merwe, R. (2000). The unscented Kalman filter.

## License

TODO

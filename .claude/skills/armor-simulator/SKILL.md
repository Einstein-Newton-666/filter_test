---
name: armor-simulator
description: Use when working with the armor/rune simulation pipeline — launching nodes, configuring noise models, debugging filter chains, marker output, ground truth, or understanding TF/camera data flow
---

# 装甲板 / 能量机关仿真器

## 代码结构

```
src/armor_simulation/                    (namespace armor_sim)
├── include/armor_simulation/
│   ├── armor_simulation.hpp            ArmorSimulation : rclcpp::Node
│   ├── camera_model.hpp               CameraModel (投影/PnP/setExtrinsics)
│   ├── armor_geometry.hpp             computeArmorCorners/computeArmorPose
│   ├── detection_noise.hpp            DetectionNoise (像素噪声/检测概率)
│   └── pnp_pose_utils.hpp             PnP RPY 提取 + 平面双解 yaw 修正
├── src/
│   ├── armor_simulation.cpp           装甲板/前哨站仿真节点 (运动+相机+噪声+PnP)
│   ├── rune_simulation.cpp            能量机关仿真节点 (五点观测+active blade+marker)
│   ├── gimbal_simulation.cpp          云台仿真 (S-curve+TF+反馈)
│   └── angle_solver.cpp              角度解算 (目标位置→云台角度)
├── config/simulation_config.yaml      所有参数 (/armor_simulation_node)
├── launch/simulation.launch.py        仅仿真器
├── msg/GroundTruth.msg                装甲板真值: x,y,z,yaw,vx,vy,vz,vyaw,r1,r2,dz
└── msg/RuneGroundTruth.msg            能量机关真值: center, normal_yaw/pitch, roll, vroll, mode, active_blades
```

## 构建与启动

```bash
colcon build --packages-select armor_simulation
source install/setup.bash

# 完整仿真 (3 节点)
ros2 run armor_simulation armor_simulation_node \
  --ros-args --params-file src/armor_simulation/config/simulation_config.yaml &
ros2 run armor_simulation gimbal_simulation &
ros2 run armor_simulation angle_solver
```

## 话题

| 话题 | 消息类型 | 发布者 | 说明 |
|------|---------|--------|------|
| `/detector/armors` | Armors | armor_sim_node | 带噪声观测 (4 角点像素 + 3D 位姿 + yaw) |
| `/armor_simulation/ground_truth` | GroundTruth | armor_sim_node | 11 维真值 |
| `/rune_detector/rune_targets` | RuneTargets | rune_sim_node | 能量机关观测紧凑数组，只包含五点像素，不带物理编号 |
| `/detector/rune_targets` | RuneTargets | rune_sim_node | 兼容旧话题，同上 |
| `/rune_simulation/ground_truth` | RuneGroundTruth | rune_sim_node | 能量机关中心点、normal_yaw/pitch、roll/vroll、模式和 active blade |
| `/send_pack` | SendData | angle_solver | 目标 yaw/pitch (度) |
| `/recieve_pack` | RecieveData | gimbal_sim | 当前 yaw/pitch + shoot_speed |
| `/simulation/marker` | MarkerArray | armor_sim_node | 青色球=真值, 紫色立方=装甲板 |
| `/rune_simulation/marker` | MarkerArray | rune_sim_node | 与图优化一致：中心球、目标球、状态文本、五片扇叶和观测扇叶 |

一条 `/detector/armors` 中 Armor 消息:

```
detected_points[0..3] = {x: u, y: v}   # 左下 左上 右上 右下 (带噪声)
pose = {position, orientation}           # odom 系 PnP 3D 位姿
yaw                                     # PnP/IPPE 后经平面双解修正的朝向 (rad)
area                                    # 像素面积 (px²)
```

## 数据生成流程

每帧 `publishSimulation()` 的步骤:

1. **过程噪声** — CWNA 模型: `a_rand = sqrt(q/dt) * N(0,1)`, `ax_eff = x_a + ax_noise`
2. **径向距离约束** — 超界时直接反向加速度，和 linear_limit 一致: 若 `r > max` → `x_a = -x/|x| * linear_acc` (指向原点刹车); 若 `r < min` → `x_a = +x/|x| * linear_acc` (背离原点); 共用 `linear_acceleration` 参数
3. **运动更新** — `pos+=v*dt+0.5*a*dt²`, `v+=a*dt`
4. **TF 查询** — `lookupTransform(odom, camera_optical_frame, now, 100ms)` → 更新相机外参
5. **遍历 4 块装甲板**:
   - 3D 位姿: `armor_yaw = yaw + i*π/2`, `pos = center - r*(cos,sin)`, `pitch = +15°`
   - 过滤链: 朝向 → 检测概率 → 4角点投影+可见性 → 装甲板级相关像素噪声 → IPPE PnP → yaw 先验修正双解
6. **Marker 管理** — 每帧先 DELETE 全部 obs marker (ID 0-3)，再 ADD 可见装甲板 (lifetime 0.1s) → 掉检即时消失
7. **发布** — `/detector/armors` + `/ground_truth` + `/simulation/marker`

## 能量机关仿真输出约定

`rune_simulation_node` 不移植 detector，直接从 3D 五点模型投影出像素观测：

```text
RuneTarget pixel order = r_center, near_point, left_point, far_point, right_point
```

- 小符每 3 秒随机选择 1 片 active blade。
- 大符每 3 秒随机选择 2 片 active blade。
- `/rune_detector/rune_targets` 和 `/detector/rune_targets` 只发布观测到的 active blade；
  未 active 或不可见/掉检的叶片不进入检测消息。
- 物理扇叶编号只保留在 ground truth 和 marker 调试语义中，不进入 `RuneTarget`。
- `/rune_simulation/ground_truth` 只发布中心点 `x/y/z`、符面 `normal_yaw/normal_pitch`、
  `roll/vroll`、`mode` 和 `active_blades`，
  不发布每片叶片的 3D 点。
- `RuneInfo` 同步发布 `normal_yaw/normal_pitch`；marker 中心球和目标球的 orientation
  也使用这两个姿态参数。

Marker 固定使用 `odom` frame，`lifetime=0.1s`，并和 `/rune_graph_optimizer/marker`
保持同一套 namespace/type：

| namespace | 类型 | 含义 |
|-----------|------|------|
| `rune_center` | `SPHERE` | 能量机关中心点，绿色，`scale=0.06` |
| `rune_target` | `SPHERE` | 当前目标点，红色，`scale=0.08` |
| `rune` | `SPHERE` + `LINE_STRIP` | 5 片扇叶位置和中心到扇叶的半径线，半透明绿色 |
| `observed_position` | `SPHERE` | 当前观测到的 active blade，半透明紫色 |
| `rune_status` | `TEXT_VIEW_FACING` | `roll` 和观测数量 |

## 核心接口

### CameraModel (camera_model.hpp)

```cpp
Vector2d project3Dto2D(Vector3d p_odom)         // odom→pixel (含畸变)
bool isInImage(Vector2d pixel, double margin)    // 可见性检查
double computeArea(array<Vector3d,4>& corners)   // 投影后鞋带公式求面积
PnPResult estimatePose(array<Vector2d,4>& pixels, w, h)  // PnP: pixels→odom位姿
void setExtrinsics(Matrix3d R, Vector3d t)       // 动态更新外参 (云台用)
static CameraExtrinsics fromEulerAngles(r,p,y, tx,ty,tz) // ZYX欧拉角→外参
```

投影链: `p_cam = T_camera_odom * p_odom` → `x/z, y/z` → Brown-Conrady畸变 → `K * [xd, yd, 1]`

CameraIntrinsics 结构: fx, fy, cx, cy, k1, k2, p1, p2, image_width, image_height

PnP 细节:
- 局部 3D 点与 `computeArmorCorners()` 对齐: `X=法线, Y=宽度, Z=高度`, 顺序 `[左下, 左上, 右上, 右下]`
- `estimatePose()` 使用 `cv::SOLVEPNP_IPPE`; OpenCV `solvePnP(IPPE)` 返回按重投影误差选出的一个平面候选
- IPPE 仍可能有两个相近候选；仿真侧用真值 `armor_yaw` 调 `correctPlanarPnPAmbiguity()` 修正 yaw 分支
- `PnPResult.success=false` 时跳过该观测，避免退化像素产生无效 `rvec/tvec`

### DetectionNoise (detection_noise.hpp)

```cpp
Vector2d addPixelNoise(Vector2d pixel, double distance)
    // σ(d) = σ_min + k*(d-d_opt)², 下限保护 σ≥0.5px
    // 5%概率 σ=outlier_std (模拟反光/遮挡误匹配)
array<Vector2d,4> addArmorPixelNoise(array<Vector2d,4> pixels, double distance)
    // p_i' = p_i + n_c + n_i
    // n_c ~ N(0,(ρσ)^2 I), n_i ~ N(0,((sqrt(1-ρ²))σ)^2 I)
bool shouldDetect(double distance, double area)
    // P = detect_prob * sigmoid(0.01*(area-100)) * sigmoid(1.0*(8-dist))
    // 先过 miss_prob 基础漏检
```

当前仿真节点使用 `addArmorPixelNoise()`，不是四角点完全独立噪声。`pixel_noise_common_ratio=ρ` 越大，共同平移成分越多，矩形形状破坏越少，PnP yaw 抖动越小；非离群点满足单点总方差仍为 `σ²I`。

**当前默认量级** (`σ_min=0.5`, `d_opt=5m`, `k=0.025`): 最优距离约 0.5px；噪声增大时平面 PnP 两个候选误差更接近，yaw 更容易跳。

**设计原则**: 唯一噪声源是像素角点噪声。3D 位姿通过 PnP 从带噪像素求解，噪声自然传播，保证 pixel↔pose 一致性。

### computeArmorCorners (armor_geometry.hpp)

```cpp
// 4 角点: [左下, 左上, 右上, 右下]
array<Vector3d,4> computeArmorCorners(position, orientation, width, height)
// 局部坐标: {(0,-w/2,-h/2), (0,-w/2,h/2), (0,w/2,h/2), (0,w/2,-h/2)}
// 坐标约定: X=装甲板法线, Y=宽度方向, Z=高度方向
// 世界坐标 = orientation * corner + position
```

标准装甲板尺寸: SMALL_ARMOR_WIDTH=0.135, SMALL_ARMOR_HEIGHT=0.125

### gimbal_simulation 节点

```cpp
// 闭环: 订阅 /send_pack (目标角度), 发布 /recieve_pack + TF
// S-curve: s(t) = 3t²-2t³, 100ms 内完成
// TF: odom→gimbal_link (RPY 0,pitch,yaw) + gimbal_link→camera_optical_frame (RPY -90°,0,0)
```

### 云台真值追踪

```yaml
/**:
  ros__parameters:
    publish_gimbal_gt: true  # 仿真器直接发布 /send_pack，绕过 tracker→angle_solver 闭环
```

`gimbal_simulation` 本身仍只订阅 `/send_pack`；真值 yaw/pitch 由
当前启动的 `armor_simulation_node` 或 `rune_simulation_node` 计算并发布。
armor 瞄准机器人中心；rune 优先瞄准当前观测扇叶，其次 active blade 的击打中心。

### angle_solver 节点

```cpp
// 订阅 /tracker/target (跟踪结果), 订阅 /recieve_pack (反馈), 发布 /send_pack
// target_yaw = atan2(x, -y), target_pitch = -atan2(z, √(x²+y²))
```

## 坐标系

```
Odom: X→前 Y→左 Z→上      Camera: X→右 Y→下 Z→前
变换: p_cam = R_camera_odom × p_odom + t_camera_odom
     R = Rz(yaw) × Ry(pitch) × Rx(roll)   (ZYX 欧拉角)
```

## 过滤链调试

装甲板经过 4 级过滤, `/detector/armors` 为空时逐级加日志排查:

| 级别 | 检查 | 正常结果 |
|------|------|---------|
| ① 朝向 | `dot(normal, pos) < 0` | 2 过 2 不过 |
| ② 概率 | `shouldDetect(d, area)` | 约 90% 通过 |
| ③ 可见性 | 4 角点全在 `isInImage(px, 10)` | 取决于位置 |
| ④ PnP | `estimatePose()` | 正常角点 success=true；退化/面积过小会跳过 |

常见 0 检测原因: 云台未启动 → TF 查询失败 → camera 外参错误; 机器人初始位置不在相机视野内。

## 配置

完整参数见 `config/simulation_config.yaml`。关键参数:

```yaml
/armor_simulation_node:
  ros__parameters:
    publish_rate: 100
    process_noise_xy: 0.0; process_noise_yaw: 0.0     # CWNA 过程噪声

    # 径向距离约束
    radial_min: 4.0; radial_max: 6.0                  # 最近/最远可检测距离 (m)
    linear_acceleration: 0.3                           # 边界刹车加速度 (m/s²)
    linear_speed_limit: 1.2                            # 线速度限幅 (m/s)

    initial_state: {x:0.0, y:-4.0, yaw:0.1, x_velocity:0.0, y_velocity:0.0, yaw_velocity:2.0}
    geometry: {z1:0.0, z2:0.15, r1:0.28, r2:0.38}
    camera_name: narrow_stereo
    camera_info_url: package://armor_simulation/config/camera_info.yaml
    # 外参硬编码在源码中: roll=-1.571(-90°), ty=-0.045, tz=0.08557
    # 外参在运行时通过 TF odom→camera_optical_frame 动态更新

    # 像素噪声 (U 形距离模型 + 装甲板级相关噪声)
    pixel_noise_enabled: true
    pixel_noise_optimal: 0.5; pixel_noise_optimal_distance: 5.0; pixel_noise_curvature: 0.025
    pixel_noise_common_ratio: 0.7
    use_outliers: false; outlier_probability: 0.05; outlier_std: 10.0

    # 检测概率
    detection_probability: 1.0; miss_probability: 0.0
    min_detectable_area: 100.0; max_detectable_distance: 8.0
```

## 添加新噪声模型

1. 修改 `DetectionNoiseParams` 添加参数
2. 修改 `DetectionNoise` 的 `addPixelNoise()` / `shouldDetect()` 方法
3. 在 `armor_simulation.cpp` 构造函数中 `declare_parameter` 新参数
4. 在 `simulation_config.yaml` 设置默认值

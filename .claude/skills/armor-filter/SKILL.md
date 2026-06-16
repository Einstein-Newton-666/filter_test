---
name: armor-filter
description: Use when working with EKF/UKF armor tracking filters — Kalman filter implementation, process/measurement noise tuning, CV/Singer motion models, Ceres autodiff Jacobians, or state vector index reference
---

# 装甲板跟踪滤波器

## 代码结构

```
src/filter_test/
├── include/filter_test/
│   ├── filters/
│   │   ├── kalman_filter.hpp          template<int N_X,N_Z> KalmanFilter 基类
│   │   ├── extended_kalman.hpp        EKF — Ceres Jet autodiff, 动态维度
│   │   ├── unscented_kalman.hpp       UKF — sigma 点传播, 动态维度
│   │   ├── cv_model.hpp              CV Predict / MeasureSingle / MeasureDouble / predict_q / measure_r
│   │   ├── singer_model.hpp          Singer 15D Predict / Measure / predict_q / measure_r
│   │   └── filter_common.hpp         枚举 / 角度工具函数 / ceres_xyz_to_ypd
│   ├── filter.hpp                    ArmorFilter 类 (init/update)
│   └── filter_test.hpp               ArmorTest : rclcpp::Node
├── src/
│   ├── filters/filter.cpp            ArmorFilter 实现
│   └── filter_test.cpp               main + ArmorTest
└── config/config.yaml                /filter + /graph_optimizer_test 参数
```

## 构建与启动

```bash
colcon build --packages-select filter_test
source install/setup.bash

ros2 run filter_test filter \
  --ros-args --params-file src/filter_test/config/config.yaml
```

`filter_graph_optimizer` 会订阅 `/track_result`，因此使用滤波器前端 + 图优化后端模式时，
需要同时运行 `/filter` 节点和 `filter_graph_optimizer` 节点。

## 核心接口

### KalmanFilter 基类 (kalman_filter.hpp)

```cpp
template<int N_X, int N_Z>
class KalmanFilter {
    MatrixXX A, H, P, K; MatrixX1 x_e;  // 静态固定维度
public:
    void init(const VectorXd& x0);                    // x=x0, P=I*INF
    void setState(const VectorXd& x0);
    VectorXd getState() const;

    // 动态维度重载 (接受 Eigen::MatrixXd)
    VectorXd predict(FuncA&& f, const MatrixXd& Q);   // f(A) 填入 A → x=A*x, P=A*P*A'+Q
    VectorXd update(FuncH&& f, const VectorXd& z, const MatrixXd& R);
    // K = P*H'*(H*P*H'+R)^-1, x+=K*(z-H*x), P=(I-K*H)*P

    // 静态维度重载 (接受 MatrixXX/MatrixZ1/MatrixZZ, 保留兼容)
    MatrixX1 predict(FuncA&& f, const MatrixXX& Q);
    MatrixX1 update(FuncH&& f, const MatrixZ1& z, const MatrixZZ& R);
};
```

### ExtendedKalmanFilter (extended_kalman.hpp)

**动态维度**，使用 Ceres Jet 做 autodiff 计算雅可比。

```cpp
class ExtendedKalmanFilter {
    VectorXd x_e; MatrixXd P_mat;
public:
    void init(const VectorXd& x0);          // x=x0, P=I (Identity)

    // 预测 — 返回结果但不修改状态 (用于匹配)
    struct PredictResult { VectorXd x_pri; MatrixXd F; };
    template<class PredictFunc>
    PredictResult predict(PredictFunc&& f);  // Ceres Jet autodiff → x_pri, F

    // 预测并更新状态和协方差
    template<class PredictFunc>
    void predict_forward(PredictFunc&& f, const MatrixXd& Q);
    // 内部: predict(f) → x_e=x_pri, P=F*P*F'+Q

    // 观测 — 返回结果但不修改状态
    struct MeasureResult { VectorXd z_pri; MatrixXd H; };
    template<class MeasureFunc>
    MeasureResult measure(MeasureFunc&& h);  // Ceres Jet autodiff → z_pri, H

    // 观测更新并修改状态和协方差
    template<class MeasureFunc>
    void update_forward(MeasureFunc&& h, const VectorXd& z, const MatrixXd& R);
    // 内部: measure(h) → K=P*H'*(H*P*H'+R)^-1, x+=K*(z-z_pri), P=(I-K*H)*P
};
```

**UKF** 接口类似: `predict_forward(f, Q)` / `update_forward(h, z, R)`，采样 2n+1 个 sigma 点传播。

### ArmorFilter (filter.hpp)

```cpp
class ArmorFilter {
    ExtendedKalmanFilter ekf;   // 动态维度 EKF
    UnscentedKalmanFilter ukf;  // 动态维度 UKF
public:
    ArmorFilter(
        bool use_ekf = true, bool use_cv_model = true,
        // CV 过程噪声
        double s2qxy_cv = 0.5, double s2qz_cv = 1.0, double s2qyaw_cv = 0.5,
        double s2qr_cv = 10.0, double s2qdz_cv = 1.0,
        // Singer 过程噪声
        double s2qxy_singer = 0.5, double s2qz_singer = 1.0, double s2qyaw_singer = 0.5,
        double s2qr_singer = 10.0, double s2qdz_singer = 1.0,
        double tau_singer = 1.0,
        // UKF 参数
        double ukf_alpha = 0.001, double ukf_beta = 2.0, double ukf_kappa = 0.0);

    void init(Armors::SharedPtr& armors_msg);       // 首次观测→初始状态
    VectorXd update(const Armors::SharedPtr& msg);  // 预测+匹配+更新 (一体)
    VectorXd get_last_result() const;               // 最近一次结果 (11D)
    void set_r_for_simulation(r_pose, r_dist, r_yaw);  // 固定 R (仿真)
    void set_r_for_detector(r_pose, r_dist, r_yaw);    // R ∝ d²·(yaw²+1) (检测器)
};
```

## 状态向量

**CV 模型 (11D):** `[xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r1, r2, dz]`

| idx | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|-----|---|---|---|---|---|---|---|---|---|---|----|
| 含义 | xc | v_xc | yc | v_yc | za | v_za | yaw | v_yaw | r1 | r2 | dz |

初始化: `xc = armor.x + init_r*cos(yaw)`, `yc = armor.y + init_r*sin(yaw)`, 速度=0, r1=r2=init_r=0.25, dz=0

**Singer 模型 (15D):** `[x, vx, ax, y, vy, ay, z, vz, az, yaw, vyaw, ayaw, r1, r2, dz]`

| idx | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 |
|-----|---|---|---|---|---|---|---|---|---|---|----|----|----|----|----|
| 含义 | x | vx | ax | y | vy | ay | z | vz | az | yaw | vyaw | ayaw | r1 | r2 | dz |

## CV 模型公式

**Predict:** `xc += v_xc*dt`, `yc += v_yc*dt`, `za += v_za*dt`, `yaw += v_yaw*dt`, 速度不变, r1/r2/dz 不变

**MeasureSingle (4D 观测, armor_index=I):**
```
armor_yaw = yaw + I*π/2
r  = (I%2==0) ? r1 : r2
armor_z = (I%2==0) ? za : za + dz
armor_pos = [xc - r*cos(armor_yaw),  yc - r*sin(armor_yaw),  armor_z]
z = [atan2(armor_y, armor_x),  atan2(armor_z, √(x²+y²)),  |armor|,  yaw+I*π/2]
```

**MeasureDouble (8D 观测):** 两块装甲板的 4D 拼接

## ArmorFilter::update 流程

1. **空观测**: 无 armor → 更新时间戳，返回当前状态
2. **dt** = (now - last_time).seconds()
3. **匹配用预测**: `ekf.predict(Predict(dt))` 返回 PredictResult (不修改状态和协方差)
4. **装甲板匹配**:
   - 单装甲板: 遍历 4 个候选索引，选 `|shortest_angular_distance(obs_yaw, pred_yaw+j*π/2)|` 最小的
   - 双装甲板: 遍历 4 对相邻 {0,1}{1,2}{2,3}{3,0}，两种顺序排列，选总 yaw 差最小的
5. **构建观测**: xyz → ypd (via `ceres_xyz_to_ypd`)
6. **正式预测+更新**: `predict_forward(Predict(dt), Q)` → `update_forward(MeasureSingle/Double, z_pyd, R)` (EKF) 或对应 UKF 调用
7. **状态约束**: r1/r2 clamp [0.15, 0.4], dz clamp [-0.1, 0.1]
8. **返回** 11D 结果: `[xc,yc,za, vxc,vyc,vza, yaw,vyaw, r1,r2,dz]`

## R 矩阵

```cpp
// measure_r() in cv_model.hpp / singer_model.hpp
// 每 4D 观测块: [r_pose, r_pose, r_distance, r_yaw]

// 仿真模式 (use_fixed_r=true): 固定对角 R = diag(r_pose, r_pose, r_distance, r_yaw)

// 检测器模式 (use_fixed_r=false): 距离+角度相关
//   R(2,2) = r_distance * z_dist² * (|yaw_deviation_in_rad|² + 1)
//   R(3,3) = log(|yaw_deviation_in_rad| + 1) * 0.01 + r_yaw
// yaw_deviation = |normalize_angle(obs_yaw - atan2(yc_center, xc_center))|
```

## Q 矩阵

```cpp
// CV: q_pos = s2q * dt⁴/4,  q_cross = s2q * dt³/2,  q_vel = s2q * dt²
// Q 为分块对角: 每对 (位置,速度) 共用上述公式, r1/r2/dz 只有对角项

// Singer: 标准 Singer 模型公式 (指数衰减 autocorrelation)
//   q11 = σ²·(1-e^{-2α·dt}+2α·dt+2α³·dt³/3-2α²·dt²-4α·dt·e^{-α·dt})/(2α⁵)
//   q22 = σ²·(4e^{-α·dt}-3-e^{-2α·dt}+2α·dt)/(2α³)  等
//   r1/r2/dz: 对角 q_r = s2qr * dt⁴/4
// α = 1/tau (机动频率)
```

## 角度工具 (filters/filter_common.hpp)

```cpp
normalize_angle(a)        → [-π, π]
shortest_angular_distance(from, to)
get_closest_angle(cur, tar) // 找 tar 最近的 cur 等效值 (处理 2π 环绕)
ceres_xyz_to_ypd(xyz, ypd)  // 笛卡尔 → [yaw, pitch, distance]

enum TrackerState { LOST, DETECTING, TRACKING, TEMP_LOST };
enum MatchState { MATCH_NONE, MATCH_SINGLE, MATCH_DOUBLE };
```

## 配置

```yaml
/filter:
  ros__parameters:
    use_ekf: false; use_cv_model: true
    s2qxy_cv: 0.1; s2qz_cv: 0.1; s2qyaw_cv: 0.5   # CV 过程噪声
    s2qr_cv: 10.0; s2qdz_cv: 0.1
    s2qxy_singer: 0.05; s2qz_singer: 0.05          # Singer 过程噪声
    s2qyaw_singer: 0.1; s2qr_singer: 10.0; s2qdz_singer: 0.01
    tau_singer: 2.0
    r_pose_det: 0.01; r_distance_det: 0.01          # 检测器 R (距离+角度相关)
    r_yaw_det: 0.05
    ukf_alpha: 0.01; ukf_beta: 2.0; ukf_kappa: 0.0
    init_r: 0.25                                    # 注: 当前代码硬编码为 0.25

/graph_optimizer_test:
  ros__parameters:
    s2qvel: 0.0001; s2qvyaw: 0.0025                 # 运动因子速度方差
    vel_sigma: 0.01; vyaw_sigma: 0.05               # VelSmooth/VyawSmooth 标准差
    geo_yaw: 0.05                                   # 几何 yaw 约束过强会导致数值问题
```

## 调优指南

| 问题 | 调整 |
|------|------|
| 滤波滞后 (跟不上) | 增大 Q (`s2qxy/s2qz/s2qyaw`) |
| 滤波振荡 (太敏感) | 减小 Q, 增大 R |
| yaw 发散 | 检查 `get_closest_angle` 环绕处理 |
| 初始化偏差大 | 调整 `init_r` (需改代码, 当前硬编码 0.25) |
| UKF 不收敛 | 减小 `ukf_alpha` (默认 0.01), 增大 `ukf_beta` |
| Singer NaN | 增大 `tau_singer` |
| r1/r2 漂移过大 | clamp 范围 [0.15, 0.4], 增大 s2qr |
| iSAM2 欠约束/速度漂移 | 调小 `vel_sigma/vyaw_sigma` 或检查 2D 模式的 Pose3 prior/几何因子 |

## 添加新运动/观测模型

1. 在 `filters/` 下新建头文件，定义 `Predict` / `Measure*` 结构体
2. `operator()(const T& x, T& x_next/z)` 模板化，兼容 `double` 和 `ceres::Jet`
3. 定义 `input_size` / `output_size` / `size` 成员供 EKF/UKF 使用
4. 实现 `predict_q()` / `measure_r()` 函数
5. 在 `ArmorFilter` 中添加对应的 predict/update 调用分支
6. 在 `filters/filter_common.hpp` 添加相关工具函数 (如需要)
7. `config.yaml` 添加模型专属参数

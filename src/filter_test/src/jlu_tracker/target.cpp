// Adapted from jlu_vision_26 auto_aim RobotTarget
#include "filter_test/jlu_tracker/target.hpp"
#include "filter_test/jlu_tracker/spherical_coordinate.hpp"
#include "filter_test/graph_optimizer/graph_math.hpp"

#include <gtsam/geometry/Rot2.h>
#include <array>
#include <cmath>
#include <algorithm>
#include <utility>
#include <vector>

namespace jlu {

// ═══════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════

// ZYX 欧拉角提取: R = Rz(yaw) * Ry(pitch) * Rx(roll)
// pitch 用 -R(2,0) 提取 (GTSAM Pose3 默认 z-forward 约定)
static Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R) {
  double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
  double roll  = std::atan2(R(2, 1), R(2, 2));
  double yaw   = std::atan2(R(1, 0), R(0, 0));
  return {roll, pitch, yaw};
}

static double limitRadian(double angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

// ═══════════════════════════════════════════════════════════════════════
// matchArmor (欧几里得距离 — 冷启动/恢复阶段使用)
//
// localCoordinates 是 Rot2 的 SO(2) 流形方法:
//   localCoordinates(other) 返回从 this 到 other 的最短角度差 (∈ [-π, π])
//   例如: Rot2(3.1).localCoordinates(Rot2(-3.1)) ≈ 0.08 (而非 6.2)
//   这保证了 yaw_diff ∈ [0, π], 在 ±π 边界不会出现 2π 的虚假大误差
// ═══════════════════════════════════════════════════════════════════════
std::vector<ArmorMatchResult> RobotTarget::matchArmor(
    const std::vector<ArmorPositionYaw>& armors, const ArmorPositionYaw& obs,
    double max_match_distance, double max_match_yaw_diff) {
  std::vector<ArmorMatchResult> results;
  std::size_t i = 0;
  for (const auto& armor : armors) {
    // 3D 欧几里得距离
    auto distance = (armor.position - obs.position).norm();
    // SO(2) 角度差 (最短弧长, 自动包裹)
    auto yaw_diff = std::abs(obs.yaw.localCoordinates(armor.yaw).x());
    if (distance <= max_match_distance && yaw_diff <= max_match_yaw_diff)
      results.emplace_back(static_cast<ArmorIndex>(i), distance, yaw_diff);
    ++i;
  }
  // 按 yaw_diff 升序 (最小角度差优先)
  std::sort(results.begin(), results.end(),
            [](const auto& a, const auto& b) { return a.yaw_diff < b.yaw_diff; });
  return results;
}

// ═══════════════════════════════════════════════════════════════════════
// matchArmor (马氏距离 — 正式跟踪阶段使用)
//
// 误差定义在 YPD 空间 (Yaw-Pitch-Distance + armor_yaw):
//   d_ypd  = cartesian2Spherical(obs.pos) - cartesian2Spherical(pred.pos)
//          = [Δyaw_obs, Δpitch_obs, Δdistance]  (3D 球坐标差)
//   dr     = pred.yaw.localCoordinates(obs.yaw)  (1D SO(2) 角度差)
//   error  = [d_ypd; dr]  (4D)
//
// 马氏距离: d² = errorᵀ * S⁻¹ * error
//   其中 S = P (预测协方差) + R (观测噪声), 通过 LDLT 分解求解
//   LDLT 比直接求逆更稳定, 适合在线实时计算
// ═══════════════════════════════════════════════════════════════════════
std::vector<ArmorMatchResult> RobotTarget::matchArmor(
    const std::vector<std::pair<ArmorPositionYaw, Eigen::Matrix4d>>& armors_covs,
    const ArmorPositionYaw& obs) {
  std::vector<ArmorMatchResult> results;
  std::size_t i = 0;
  for (const auto& [armor, cov] : armors_covs) {
    // YPD 观测残差: 笛卡尔坐标 → 球坐标 (保留局部几何非线性)
    Eigen::Vector3d dypd = cartesian2Spherical(obs.position) -
                           cartesian2Spherical(armor.position);
    // SO(2) 角度残差: 自动包裹到 [-π, π]
    auto dr = armor.yaw.localCoordinates(obs.yaw).x();
    Eigen::Vector4d error{Eigen::Vector4d::Zero()};
    error.head<3>() = dypd;   // YPD 残差: [Δyaw_sph, Δpitch, Δdistance]
    error(3) = dr;             // armor yaw 残差: SO(2) 最短弧长

    // 马氏距离: d² = errorᵀ * S⁻¹ * error
    // cov.ldlt().solve(error) = S⁻¹·error (通过 LDLT 分解, 比直接求逆稳定)
    double mahalanobis_distance = error.transpose() * cov.ldlt().solve(error);
    results.emplace_back(static_cast<ArmorIndex>(i), mahalanobis_distance, dr);
    ++i;
  }
  // 按马氏距离升序 (最小距离 → 最可能的匹配)
  std::sort(results.begin(), results.end(),
            [](const auto& a, const auto& b) { return a.distance < b.distance; });
  return results;
}

// ═══════════════════════════════════════════════════════════════════════
// 构造函数: 初始化状态为 LOST, 半径/dz 设为默认值
// ISAM2 和协方差通过 resetCovariances() 设初值
// ═══════════════════════════════════════════════════════════════════════
RobotTarget::RobotTarget(const RobotConfig& config, ArmorType type,
                         const cv::Mat& camera_matrix,
                         const cv::Mat& distortion_coefficients)
    : config_(config), camera_matrix_(camera_matrix),
      distortion_coefficients_(distortion_coefficients) {
  target_state_.type = type;
  target_state_.radius_a = config_.default_radius;
  target_state_.radius_b = config_.default_radius;
  target_state_.dz = config_.default_dz;
  track_state_.state = TrackState::State::LOST;
  track_state_.stamp_last_update = 0;
  track_state_.stamp_last_tracking = 0;
  track_state_.k = 0;
  this->resetCovariances();
}

// ── getArmorsFromTargetState ──
std::vector<ArmorPositionYaw> RobotTarget::getArmorsFromTargetState(
    const TargetState& state, double radius_a, double radius_b, double dz) {
  std::vector<ArmorPositionYaw> armors;
  for (auto i : {ArmorIndex::_0, ArmorIndex::_1, ArmorIndex::_2, ArmorIndex::_3}) {
    auto [r, dz_val] = (i == ArmorIndex::_0 || i == ArmorIndex::_2)
                           ? std::make_pair(radius_a, 0.0)
                           : std::make_pair(radius_b, dz);
    armors.emplace_back(state.center_position, state.center_yaw, r, dz_val, 4, i);
  }
  return armors;
}

std::vector<ArmorPositionYaw> RobotTarget::getArmorsFromTargetState(
    const RobotTargetState& state) const {
  return getArmorsFromTargetState(state, state.radius_a, state.radius_b, state.dz);
}

// ═══════════════════════════════════════════════════════════════════════
// 马氏距离装甲板匹配
//
// 协方差传播链 (5步):
//
// 1. 预测协方差 (CWNA 运动模型传播)
//    cov_X_pred = X_cov + V_cov*dt² + Q_translation*dt²
//    cov_R_pred = R_cov + W_cov*dt² + Q_yaw*dt²
//    (X_cov/V_cov 来自上一帧 iSAM2 的 marginalCovariance)
//
// 2. 中心协方差 (4×4)
//    center_cov = diag([cov_X_pred(3×3), cov_R_pred(1×1)])
//
// 3. 传播到每个装甲板 (雅可比变换)
//    armor = center + [-r·cos(yaw+I·π/2), -r·sin(yaw+I·π/2), dz_armor]
//    J = ∂armor/∂[center_x, center_y, center_z, center_yaw]
//      = [[1,0,0,  r·sin(yaw+I·π/2)],
//         [0,1,0, -r·cos(yaw+I·π/2)],
//         [0,0,1,  0                ]]
//    P_xyz(3×4) = J * center_cov  (位置部分)
//
// 4. 转换为 YPD 空间 (球坐标 + armor_yaw)
//    J_sph = cartesian2SphericalJacobian(armor.position)
//    P(3×3 top-left) = J_sph * P_xyz * J_sphᵀ  (YPD 协方差)
//    P(4,4) = center_cov(3,3)  (yaw 部分直达)
//
// 5. 加观测噪声 (距离/角度自适应)
//    R = diag([yaw_pitch_noise, yaw_pitch_noise, dist_noise, armor_yaw_noise])
//    dist_noise ∝ log(|倾角|+1) + baseline  (斜着噪声大)
//    S = P + R  (创新协方差)
//
// 6. 匹配: 对每个观测, 计算所有候选的 errorᵀ·S⁻¹·error,
//    取最小马氏距离的候选 (unique match per armor)
// ═══════════════════════════════════════════════════════════════════════
std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>
RobotTarget::matchArmors(
    const RobotTargetState& state, double dt,
    const std::vector<ArmorPositionRollPitchYawPoints>& obs_armors_camera,
    const std::vector<ArmorPositionRollPitchYawPoints>& obs_armors_odom) const {
  // ── 步骤 1: 运动预测协方差 (CWNA 离散化) ──
  Eigen::Matrix3d cov_X_pred = X_cov_ + V_cov_ * (dt * dt);
  cov_X_pred.diagonal() += Eigen::Vector3d{
      config_.translation_factor_noise.x() * dt * dt,
      config_.translation_factor_noise.y() * dt * dt,
      config_.translation_factor_noise.z() * dt * dt,
  };
  double cov_R_pred = R_cov_ + W_cov_ * dt * dt;
  cov_R_pred += config_.yaw_factor_noise * dt * dt;

  // ── 步骤 2: 组装 4×4 中心协方差 ──
  Eigen::Matrix4d center_cov = Eigen::Matrix4d::Zero();
  center_cov.topLeftCorner<3, 3>() = cov_X_pred;  // 位置 3×3
  center_cov(3, 3) = cov_R_pred;                   // yaw 1×1
  auto armors = getArmorsFromTargetState(state);
  std::vector<std::vector<std::pair<ArmorPositionYaw, Eigen::Matrix4d>>>
      armors_covs_vec;
  for (const auto& obs : obs_armors_odom) {
    std::vector<std::pair<ArmorPositionYaw, Eigen::Matrix4d>> armors_covs;
    for (const auto& armor : armors) {
      // ── 步骤 3: 雅可比变换, 中心协方差 → 装甲板位置协方差 ──
      // offset = armor - center = [-r·cos(yaw+I·π/2), -r·sin(yaw+I·π/2), dz]
      // ∂armor/∂center_yaw = [-r·sin(yaw+I·π/2), -r·cos(yaw+I·π/2)·(-1), 0]
      //                      = [offset.y, -offset.x, 0]
      Eigen::Vector3d offset = armor.position - state.center_position;
      Eigen::Matrix4d J = Eigen::Matrix4d::Identity();  // ∂armor/∂[X, yaw]
      J(0, 3) = -offset.y();  // ∂x/∂yaw
      J(1, 3) = offset.x();   // ∂y/∂yaw
      Eigen::Matrix4d P = J * center_cov * J.transpose();

      // ── 步骤 4: 转换为 YPD 球坐标 ──
      // J_sph = ∂[yaw_sph, pitch, distance]/∂[x, y, z]
      Eigen::Matrix3d P_xyz = P.topLeftCorner<3, 3>();
      Eigen::Matrix3d J_sph = cartesian2SphericalJacobian(armor.position);
      P.topLeftCorner<3, 3>() = J_sph * P_xyz * J_sph.transpose();

      // ── 步骤 5: 自适应观测噪声 ──
      // 倾角: armor 法线方向和视线方向的夹角, 斜着看噪声放大
      auto obs_center_yaw = std::atan2(obs.position.y(), obs.position.x());
      auto obs_incline_angle = limitRadian(obs.yaw.theta() - obs_center_yaw);
      Eigen::Vector4d R_dig;
      R_dig <<
          config_.armor_match_conf.ypd_conf.yaw_pitch_noise,   // yaw 观测噪声 (定值)
          config_.armor_match_conf.ypd_conf.yaw_pitch_noise,   // pitch 观测噪声 (定值)
          // 距离噪声: log(scale·|倾角|+1) + baseline, 倾角越大噪声指数增长
          std::log(config_.armor_match_conf.ypd_conf.distance_noise_log_scale *
                       std::abs(obs_incline_angle) + 1) +
              config_.armor_match_conf.ypd_conf.basic_distance_noise,
          // armor yaw 噪声: log(距离+1)/divisor + baseline, 越远 yaw 越不准
          std::log(std::abs(obs.position.norm()) + 1) /
                  config_.armor_match_conf.ypd_conf.armor_yaw_log_divisor +
              config_.armor_match_conf.ypd_conf.basic_armor_yaw_noise;
      Eigen::Matrix4d R_mat = R_dig.asDiagonal();
      Eigen::Matrix4d S = P + R_mat;  // 创新协方差
      armors_covs.emplace_back(armor, S);
    }
    armors_covs_vec.emplace_back(armors_covs);
  }
  // ── 步骤 6: Unique Match (每装甲板只保留最匹配的观测) ──
  // 4 个容器 (armor 0/1/2/3), 每个容器保留最小马氏距离的观测
  std::array<std::vector<std::pair<double, ArmorPositionRollPitchYawPoints>>, 4>
      index_vector_array;
  for (std::size_t i = 0;
       i < obs_armors_camera.size() && i < obs_armors_odom.size(); ++i) {
    const auto& obs = obs_armors_odom.at(i);
    auto result = matchArmor(armors_covs_vec.at(i), obs);
    if (result.empty()) continue;
    // 更新 NIS 滑动窗口 (每个观测只统计最优匹配的距离)
    this->updateNisFailureDeque(result.front().distance);
    // 门限检查: 马氏距离 < 20.0 才接受
    if (result.front().distance <
        config_.armor_match_conf.max_match_mahalanobis_distance) {
      // 存入对应装甲板的容器 (按 index 分组)
      index_vector_array.at(static_cast<int>(result.front().index))
          .emplace_back(result.front().distance, obs_armors_camera.at(i));
    }
  }
  // 每个容器取最小距离的观测 (unique match)
  std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>
      matched_armors;
  int idx = 0;
  for (auto& cost_armor_vec : index_vector_array) {
    std::sort(cost_armor_vec.begin(), cost_armor_vec.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    if (!cost_armor_vec.empty())
      matched_armors.emplace_back(
          cost_armor_vec.front().second, static_cast<ArmorIndex>(idx));
    idx++;
  }
  return matched_armors;
}

// ═══════════════════════════════════════════════════════════════════════
// NIS (Normalized Innovation Squared) 异常检测
//
// 滑动窗口内统计马氏距离超阈值 (nis_failure_thres=5.0) 的比例.
// 若 >60% 的匹配超限 → nisFailured=true → 触发 LOST 重置 ISAM2
// 作用: 防止错误匹配导致图优化发散 (类似 EKF 的 χ² 检验)
// ═══════════════════════════════════════════════════════════════════════
void RobotTarget::updateNisFailureDeque(double distance) const {
  // 距离 > 5.0 (χ²(4) 约 95% 分位点) → 标记为异常
  nis_failure_deque_.push_back(distance > config_.nis_conf.nis_failure_thres);
  // 滑动窗口: 只保留最近 10 帧
  if (nis_failure_deque_.size() > static_cast<size_t>(config_.nis_conf.nis_failure_window_size))
    nis_failure_deque_.pop_front();
}

std::optional<bool> RobotTarget::nisFailured() const {
  // 窗口未满 (冷启动) → 不确定, 返回 nullopt
  if (nis_failure_deque_.size() < static_cast<size_t>(config_.nis_conf.nis_failure_window_size))
    return std::nullopt;
  // 异常比例 > 60% → 判定系统发散, 触发 LOST 重置
  return (static_cast<double>(std::count(nis_failure_deque_.begin(), nis_failure_deque_.end(), true)) /
          config_.nis_conf.nis_failure_window_size) >
         (config_.nis_conf.reset_failure_percentage_thres / 100.0);
}

// ── getTargetStateFromArmor ──
RobotTargetState RobotTarget::getTargetStateFromArmor(
    const ArmorPositionYaw& armor) const {
  auto armor_x = armor.position.x();
  auto armor_y = armor.position.y();
  auto center_x = armor_x + config_.default_radius * std::cos(armor.yaw.theta());
  auto center_y = armor_y + config_.default_radius * std::sin(armor.yaw.theta());
  auto center_z = armor.position.z();
  RobotTargetState state;
  state.type = target_state_.type;
  state.center_position = Eigen::Vector3d{center_x, center_y, center_z};
  state.center_yaw = armor.yaw.theta();
  state.radius_a = config_.default_radius;
  state.radius_b = config_.default_radius;
  state.dz = config_.default_dz;
  return state;
}

// ── getTargetTrackState ──
std::pair<RobotTargetState, TrackState> RobotTarget::getTargetTrackState() const {
  std::scoped_lock lk{state_mtx_};
  auto state = RobotTargetState{target_state_.getStateWithArmorsFunc(
      [ra = target_state_.radius_a, rb = target_state_.radius_b,
       dz = target_state_.dz](const TargetState& s) {
        return getArmorsFromTargetState(s, ra, rb, dz);
      })};
  // RobotTargetState 的 radius_a/b/dz 在基类 TargetState 构造中被默认初始化,
  // 必须显式复制 iSAM2 优化后的值 (否则 publishMarkers 读到的是默认值)
  state.radius_a = target_state_.radius_a;
  state.radius_b = target_state_.radius_b;
  state.dz = target_state_.dz;
  return {state, track_state_};
}

// ── track (main entry) ──
TrackState::State RobotTarget::track(
    const std::vector<ArmorPositionRollPitchYawPoints>& armors,
    double stamp_sec, const Eigen::Isometry3d& T_camera_to_odom) {
  double dt = stamp_sec - track_state_.stamp_last_update;
  auto [estimated_target_state, updated_track_state] =
      update(armors, dt, T_camera_to_odom);

  if (updated_track_state == TrackState::State::TRACKING) {
    std::scoped_lock lk{state_mtx_};
    target_state_ = estimated_target_state;
    track_state_.state = updated_track_state;
    track_state_.stamp_last_tracking = stamp_sec;
    track_state_.stamp_last_update = stamp_sec;
    track_state_.k += 1;
  } else if (updated_track_state == TrackState::State::TEMPLOST) {
    std::scoped_lock lk{state_mtx_};
    target_state_ = estimated_target_state;
    track_state_.state = updated_track_state;
    track_state_.stamp_last_update = stamp_sec;
    track_state_.k += 1;
  } else {
    std::scoped_lock lk{state_mtx_};
    track_state_.state = TrackState::State::LOST;
    track_state_.k = 0;
    isam2_ = gtsam::ISAM2{};
    this->resetCovariances();
    this->nis_failure_deque_.clear();
  }
  return track_state_.state;
}

double RobotTarget::get(const std::string& key) const {
  std::scoped_lock lk{state_mtx_};
  if (key == "ra") return target_state_.radius_a;
  if (key == "rb") return target_state_.radius_b;
  if (key == "dz") return target_state_.dz;
  return 0;
}

// ═══════════════════════════════════════════════════════════════════════
// 运动因子: 每帧添加 4 个因子 (k>0) 或 4 个先验 (k=0)
//
// 因子图结构 (k>0):
//   X(k-1) ──→ V(k-1) ──→ X(k)    TranslationFactor (3-key, σ=0.001)
//   V(k-1) ──→ V(k)                VelocityFactor  (2-key, σ=0.01)
//   R(k-1) ──→ W(k-1) ──→ R(k)    YawFactor       (1-key Rot2, σ=0.005)
//   W(k-1) ──→ W(k)                VyawFactor      (2-key, σ=0.05)
//
// 变量布局 (7组, 稀疏):
//   X(k) = Point3  (3D 中心位置)
//   V(k) = Vector3 (3D 中心速度)
//   R(k) = Rot2    (1D 朝向, SO(2) 流形)
//   W(k) = double  (1D 角速度)
//   A(0) = double  (r1, logistic 空间, 静态)
//   B(0) = double  (r2, logistic 空间, 静态)
//   Z(0) = double  (dz, 静态)
//
// 关键设计: X/V 分离 + VelocityFactor 解决速度欠约束问题.
//   如果合并为 6D 变量, Cholesky 复杂度从 3³+3³=54 升到 6³=216,
//   数值误差累积快 4 倍, ~400 帧后 ISAM2 奇异.
// ═══════════════════════════════════════════════════════════════════════
void RobotTarget::addMotionValuesFactors(
    gtsam::Values& values, gtsam::NonlinearFactorGraph& graph,
    const TargetState& target_state, uint64_t k, double dt) const {
  // 插入当前帧变量的初始值 (ISAM2 增量更新需要)
  values.insert(X(k), target_state.center_position);
  values.insert(R(k), gtsam::Rot2::fromAngle(target_state.center_yaw));
  values.insert(V(k), target_state.center_velocity);
  values.insert(W(k), target_state.center_vyaw);
  if (k == 0) {
    // k=0: 添加先验因子 (锚定初始状态)
    graph.addPrior(X(0), target_state.center_position,
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.translation_prior_noise.x(),
                       config_.translation_prior_noise.y(),
                       config_.translation_prior_noise.z()}));
    graph.addPrior(
        R(0), gtsam::Rot2::fromAngle(target_state.center_yaw),
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_prior_noise));
    graph.addPrior(V(0), target_state.center_velocity,
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.velocity_prior_noise.x(),
                       config_.velocity_prior_noise.y(),
                       config_.velocity_prior_noise.z()}));
    graph.addPrior(
        W(0), target_state.center_vyaw,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.vyaw_prior_noise));
  } else {
    graph.add(TranslationFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
            config_.translation_factor_noise.x(),
            config_.translation_factor_noise.y(),
            config_.translation_factor_noise.z()}),
        X(k - 1), V(k - 1), X(k), dt});
    graph.add(YawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_factor_noise),
        R(k - 1), W(k - 1), R(k), dt});
    graph.add(VelocityFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
            config_.velocity_factor_noise.x(),
            config_.velocity_factor_noise.y(),
            config_.velocity_factor_noise.z()}),
        V(k - 1), V(k)});
    graph.add(VyawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.vyaw_factor_noise),
        W(k - 1), W(k)});
  }
}

// ═══════════════════════════════════════════════════════════════════════
// 第一级像素观测: 为单个装甲板添加 4 个角点重投影因子
//
// armor_pose_key = Symbol('h'+index, k) — 该装甲板的 Pose3 变量 (camera 系)
//   values.insert(armor_pose_key, Pose3(Rot3(armor_RPY), armor_pos))
//   ↑ 给 ISAM2 提供该变量的初始值, 从 PnP 观测直接获取
//
// 4 个角点: LeftBottom → LeftTop → RightTop → RightBottom
//   局部坐标 (autoaim 约定):
//     X = 前 (法线方向), Y = 左 (宽度方向), Z = 上 (高度方向)
//     LeftBottom  = {0, -w/2, -h/2} = {0, -0.0675, -0.0625}
//     LeftTop     = {0, -w/2,  h/2}
//     RightTop    = {0,  w/2,  h/2}
//     RightBottom = {0,  w/2, -h/2}
//
// 每个 ArmorReprojFactor (1-key Pose3 → 2D 像素, σ=1.0px):
//   误差 = project(armor_pose, corner_local) - pixel_observed
//   内部调用 GTSAM PinholeCamera<Cal3DS2>::Project (含畸变)
//
// 4 角点 × 2D = 8 个约束, 加上外部 Pose3 先验 6 个约束 = 14 约束
// 对 6D Pose3 变量 → 充分约束 (过约束)
// ═══════════════════════════════════════════════════════════════════════
void RobotTarget::addArmorReprojValuesFactors(
    gtsam::Values& values, gtsam::NonlinearFactorGraph& graph,
    gtsam::Key armor_pose_key,
    const ArmorPositionRollPitchYawPoints& armor, uint64_t k) const {
  // 插入装甲板 Pose3 初始值 (从 PnP 观测: camera 系位置 + RPY 旋转)
  values.insert(armor_pose_key,
                gtsam::Pose3{gtsam::Rot3{armor.getRotation()}, armor.position});
  // 遍历 4 个角点, 每个角点添加一个重投影因子
  for (auto position : {ArmorPointPosition::LeftBottom,
                        ArmorPointPosition::LeftTop,
                        ArmorPointPosition::RightTop,
                        ArmorPointPosition::RightBottom}) {
    graph.add(ArmorReprojFactor{
        // 像素噪声: σ = 1.0px (对角, x 和 y 方向等精度)
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2{
            config_.armor_observation_noise.pixel_error.x(),
            config_.armor_observation_noise.pixel_error.y()}),
        armor_pose_key,                    // 连接到该装甲板的 Pose3 变量
        camera_matrix_,                    // 相机内参 (fx, fy, cx, cy)
        distortion_coefficients_,          // 畸变系数 (k1, k2, p1, p2)
        target_state_.type,                // 装甲板类型 (Small/Large, 决定角点坐标)
        position,                          // 角点位置枚举 (确定局部坐标)
        armor.points.at(static_cast<int>(position))});  // 观测像素值 (含噪声)
  }
}

// ═══════════════════════════════════════════════════════════════════════
// 观测因子 (两级):
//
// 第一级 — ArmorReprojFactor (1-key Pose3 → 2D 像素):
//   每个角点一个因子, 每个装甲板 4 个因子
//   误差: project(armor_pose, corner_local) - pixel_observed
//
// 第二级 — 几何约束 (4D 误差):
//   ArmorRadiusCenterZFactor (armors 0,2, 4-key):
//     armor_pose + radius_A + center_yaw + center_pos → [切向, 径向, z, yaw]
//   ArmorRadiusDZFactor (armors 1,3, 5-key):
//     armor_pose + radius_B + dz + center_yaw + center_pos → [切向, 径向, z, yaw]
//
// 几何误差定义在 armor 局部坐标系 (每一块装甲板有自己的朝向):
//
//   法线方向 (nx, ny) = (cos(armor_yaw), sin(armor_yaw))
//     ↑ 装甲板"前面"的朝向. 中心在这个方向的反方向 (后方).
//
//   切线方向 (tx, ty) = (-ny, nx)
//     ↑ 垂直于法线, 沿装甲板宽度. 法线左转 90°.
//
//   (dx, dy) = (center_x - armor_x, center_y - armor_y)
//     ↑ 从装甲板指向中心的向量.
//
// 四个误差分量:
//   切向 = tx·dx + ty·dy     = 中心偏离法线的横向距离
//          理想值 ≈ 0 (中心正好在装甲板正后方)
//          如果中心偏左或偏右 → 切向 ≠ 0 → 优化器调整 xc,yc
//
//   径向 = nx·dx + ny·dy - r = 中心沿法线方向到装甲板的距离 - 半径
//          理想值 = r (中心到装甲板的距离等于估计的半径)
//          如果距离估计错了 → 径向 ≠ 0 → 优化器调整 xc,yc 和 r
//
//   z    = center_z - armor_z [+ dz]
//          armor 0,2: 和中心同高度 → 差值 ≈ 0
//          armor 1,3: 比中心高 dz → 差值 ≈ -dz, 所以加 dz 后 ≈ 0
//
//   yaw  = armor_yaw.localCoordinates(pred_yaw)
//          armor_yaw 来自 PnP 观测 (odom 系 RPY 提取)
//          pred_yaw = center_yaw + index·π/2  (4 块装甲板相差 90°)
//          localCoordinates 是 SO(2) 流形减法: ⊖ 而非 -
//          保证 yaw=179° 和 yaw=-179° 的误差是 2° 而非 358°
//
// k=0 (第一帧): 将半径和 dz 初始化为默认值,
//   并添加先验因子 (σ=0.5, 弱约束, 让后续观测主导估计).
// ═══════════════════════════════════════════════════════════════════════
void RobotTarget::addArmorValuesFactors(
    gtsam::Values& values, gtsam::NonlinearFactorGraph& graph,
    const std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>&
        armors_indexs,
    const Eigen::Isometry3d& T, uint64_t k) const {
  auto default_radius = auto_graph::logisticInverse(
      config_.default_radius, config_.radius_min, config_.radius_max);
  if (k == 0) {
    values.insert(A(0), default_radius);
    values.insert(B(0), default_radius);
    values.insert(Z(0), config_.default_dz);
    graph.addPrior(
        A(0), default_radius,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.radius_prior_noise));
    graph.addPrior(
        B(0), default_radius,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.radius_prior_noise));
    graph.addPrior(
        Z(0), config_.default_dz,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.dz_prior_noise));
  }
  for (const auto& [armor, index] : armors_indexs) {
    auto armor_pose_key = getArmorPoseKeyFromIndex(index, k);
    addArmorReprojValuesFactors(values, graph, armor_pose_key, armor, k);
    if (index == ArmorIndex::_0 || index == ArmorIndex::_2) {
      graph.add(ArmorRadiusCenterZFactor{
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4{
              config_.armor_observation_noise.tangential_error_m,
              config_.armor_observation_noise.radial_error_m,
              config_.armor_observation_noise.height_error_m,
              config_.armor_observation_noise.yaw_error_rad}),
          armor_pose_key, A(0), R(k), X(k), T, index,
          config_.radius_min, config_.radius_max});
    } else {
      graph.add(ArmorRadiusDZFactor{
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4{
              config_.armor_observation_noise.tangential_error_m,
              config_.armor_observation_noise.radial_error_m,
              config_.armor_observation_noise.height_error_m,
              config_.armor_observation_noise.yaw_error_rad}),
          armor_pose_key, B(0), Z(0), R(k), X(k), T, index,
          config_.radius_min, config_.radius_max});
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════
// 核心因子图优化 (每帧调用)
//
// 流程:
// 1. 异常检测: 超时? NIS 超限? → LOST 重置
// 2. 坐标系转换: 观测 armor 从 camera → odom (提取 RPY)
// 3. 运动预测: target_state.predict(dt) → 匀速外推
// 4. LOST 恢复: 首帧或冷启动期间, 用观测反推中心状态
// 5. 装甲板匹配: 马氏距离 unique match
// 6. 构建因子图: values + graph → 运动因子 + 观测因子
// 7. 冷启动阶段 (k < first_update_batch_size=5):
//    累积因子和初值, 不优化 (保证静态变量充分收敛)
// 8. 正式阶段: isam2_.update(graph, values) → 增量优化
//    提取估计 + 边际协方差 (供下一帧匹配用)
// ═══════════════════════════════════════════════════════════════════════
std::pair<RobotTargetState, TrackState::State> RobotTarget::update(
    const std::vector<ArmorPositionRollPitchYawPoints>& armors, double dt,
    const Eigen::Isometry3d& T) const {
  auto dt_tracking_to_update =
      track_state_.stamp_last_update - track_state_.stamp_last_tracking;
  if (track_state_.state != TrackState::State::LOST &&
      dt + dt_tracking_to_update > config_.lost_threshold_sec) {
    return {{}, TrackState::State::LOST};
  }
  if (auto nis_fail_opt = nisFailured();
      nis_fail_opt.has_value() && nis_fail_opt.value()) {
    return {{}, TrackState::State::LOST};
  }
  auto obs_armors_odom = armors;
  for (auto& armor : obs_armors_odom) {
    Eigen::Isometry3d armor_pose_camera{Eigen::Isometry3d::Identity()};
    armor_pose_camera.pretranslate(armor.position);
    armor_pose_camera.rotate(armor.getRotation());
    auto armor_pose_odom = T * armor_pose_camera;
    auto rpy = rotationMatrixToRPY(armor_pose_odom.rotation());
    armor.position = armor_pose_odom.translation();
    armor.roll = rpy.x();
    armor.pitch = rpy.y();
    armor.yaw = gtsam::Rot2::fromAngle(rpy.z());
  }

  auto target_state = target_state_.predict(dt);
  if (track_state_.state == TrackState::State::LOST) {
    if (obs_armors_odom.empty())
      return {{}, TrackState::State::LOST};
    else
      target_state = getTargetStateFromArmor(obs_armors_odom.front());
  }

  if (std::abs(target_state.center_vyaw) > config_.huge_vyaw_reset_thres) {
    return {{}, TrackState::State::LOST};
  }

  if (track_state_.k < config_.first_update_batch_size &&
      !obs_armors_odom.empty()) {
    target_state = getTargetStateFromArmor(obs_armors_odom.front());
  }

  auto matched_armors = matchArmors(target_state, dt, armors, obs_armors_odom);

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;

  if (track_state_.k <= config_.first_update_batch_size) {
    if (track_state_.k == 0) {
      this->initial_values_ = gtsam::Values{};
      this->initial_graph_ = gtsam::NonlinearFactorGraph{};
    }
    values = this->initial_values_;
    graph = this->initial_graph_;
  }
  addMotionValuesFactors(values, graph, target_state, track_state_.k, dt);
  addArmorValuesFactors(values, graph, matched_armors, T, track_state_.k);

  if (track_state_.k < config_.first_update_batch_size) {
    this->initial_values_ = values;
    this->initial_graph_ = graph;
    return {target_state,
            matched_armors.empty() ? TrackState::State::TEMPLOST
                                    : TrackState::State::TRACKING};
  }

  try {
    // ── ISAM2 增量优化 ──
    // update() 内部: 新因子插入 Bayes 树 → 标记受影响团 →
    //   从叶向根重新消元 → 从根向叶回代求解 → 更新线性化点
    this->isam2_.update(graph, values);

    // 从 ISAM2 提取优化后的估计值
    target_state.center_position =
        isam2_.calculateEstimate<gtsam::Point3>(X(track_state_.k));
    target_state.center_velocity =
        isam2_.calculateEstimate<gtsam::Vector3>(V(track_state_.k));
    // Rot2::theta() 返回标量角度 (单位: 弧度)
    target_state.center_yaw =
        isam2_.calculateEstimate<gtsam::Rot2>(R(track_state_.k)).theta();
    target_state.center_vyaw =
        isam2_.calculateEstimate<double>(W(track_state_.k));
    // logistic 空间 → 物理半径 (约束在 [radius_min, radius_max])
    target_state.radius_a = auto_graph::logisticFunction(
        isam2_.calculateEstimate<double>(A(0)), config_.radius_min,
        config_.radius_max);
    target_state.radius_b = auto_graph::logisticFunction(
        isam2_.calculateEstimate<double>(B(0)), config_.radius_min,
        config_.radius_max);
    target_state.dz = isam2_.calculateEstimate<double>(Z(0));

    // 提取边际协方差 (供下一帧的马氏距离匹配用)
    // X_cov: 3×3 位置协方差, V_cov: 3×3 速度协方差
    // R_cov: 1×1 yaw 协方差 (Rot2 的 marginalCovariance 返回 1×1)
    // W_cov: 1×1 vyaw 协方差
    this->X_cov_ = isam2_.marginalCovariance(X(track_state_.k));
    this->V_cov_ = isam2_.marginalCovariance(V(track_state_.k));
    this->R_cov_ = isam2_.marginalCovariance(R(track_state_.k))(0, 0);
    this->W_cov_ = isam2_.marginalCovariance(W(track_state_.k))(0, 0);

    return {target_state,
            matched_armors.empty() ? TrackState::State::TEMPLOST
                                    : TrackState::State::TRACKING};
  } catch (const std::exception& e) {
    // ISAM2 异常 (如 IndeterminantLinearSystemException) → 重置
    return {{}, TrackState::State::LOST};
  }
}

void RobotTarget::resetCovariances() {
  X_cov_ = Eigen::Matrix3d::Zero();
  X_cov_(0, 0) =
      config_.translation_prior_noise.x() * config_.translation_prior_noise.x();
  X_cov_(1, 1) =
      config_.translation_prior_noise.y() * config_.translation_prior_noise.y();
  X_cov_(2, 2) =
      config_.translation_prior_noise.z() * config_.translation_prior_noise.z();
  V_cov_ = Eigen::Matrix3d::Zero();
  V_cov_(0, 0) =
      config_.velocity_prior_noise.x() * config_.velocity_prior_noise.x();
  V_cov_(1, 1) =
      config_.velocity_prior_noise.y() * config_.velocity_prior_noise.y();
  V_cov_(2, 2) =
      config_.velocity_prior_noise.z() * config_.velocity_prior_noise.z();
  R_cov_ = config_.yaw_prior_noise * config_.yaw_prior_noise;
  W_cov_ = config_.vyaw_prior_noise * config_.vyaw_prior_noise;
}

}  // namespace jlu

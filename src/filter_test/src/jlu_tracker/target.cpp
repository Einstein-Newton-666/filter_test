// Adapted from jlu_vision_26 auto_aim RobotTarget
#include "filter_test/jlu_tracker/target.hpp"
#include "filter_test/jlu_tracker/spherical_coordinate.hpp"
#include "filter_test/auto_graph_optimizer/utils/helpers.hpp"

#include <gtsam/geometry/Rot2.h>
#include <array>
#include <cmath>
#include <algorithm>
#include <utility>
#include <vector>

namespace jlu {

// ── Helpers ──

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

// ── Target static matchArmor ──
std::vector<ArmorMatchResult> RobotTarget::matchArmor(
    const std::vector<ArmorPositionYaw>& armors, const ArmorPositionYaw& obs,
    double max_match_distance, double max_match_yaw_diff) {
  std::vector<ArmorMatchResult> results;
  std::size_t i = 0;
  for (const auto& armor : armors) {
    auto distance = (armor.position - obs.position).norm();
    auto yaw_diff = std::abs(obs.yaw.localCoordinates(armor.yaw).x());
    if (distance <= max_match_distance && yaw_diff <= max_match_yaw_diff)
      results.emplace_back(static_cast<ArmorIndex>(i), distance, yaw_diff);
    ++i;
  }
  std::sort(results.begin(), results.end(), [](const auto& a, const auto& b) { return a.yaw_diff < b.yaw_diff; });
  return results;
}

std::vector<ArmorMatchResult> RobotTarget::matchArmor(
    const std::vector<std::pair<ArmorPositionYaw, Eigen::Matrix4d>>& armors_covs,
    const ArmorPositionYaw& obs) {
  std::vector<ArmorMatchResult> results;
  std::size_t i = 0;
  for (const auto& [armor, cov] : armors_covs) {
    Eigen::Vector3d dypd = cartesian2Spherical(obs.position) -
                           cartesian2Spherical(armor.position);
    auto dr = armor.yaw.localCoordinates(obs.yaw).x();
    Eigen::Vector4d error{Eigen::Vector4d::Zero()};
    error.head<3>() = dypd;
    error(3) = dr;
    double mahalanobis_distance = error.transpose() * cov.ldlt().solve(error);
    results.emplace_back(static_cast<ArmorIndex>(i), mahalanobis_distance, dr);
    ++i;
  }
  std::sort(results.begin(), results.end(), [](const auto& a, const auto& b) { return a.distance < b.distance; });
  return results;
}

// ── RobotTarget constructor ──
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

// ── matchArmors (Mahalanobis) ──
std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>
RobotTarget::matchArmors(
    const RobotTargetState& state, double dt,
    const std::vector<ArmorPositionRollPitchYawPoints>& obs_armors_camera,
    const std::vector<ArmorPositionRollPitchYawPoints>& obs_armors_odom) const {
  Eigen::Matrix3d cov_X_pred = X_cov_ + V_cov_ * (dt * dt);
  cov_X_pred.diagonal() += Eigen::Vector3d{
      config_.translation_factor_noise.x() * dt * dt,
      config_.translation_factor_noise.y() * dt * dt,
      config_.translation_factor_noise.z() * dt * dt,
  };
  double cov_R_pred = R_cov_ + W_cov_ * dt * dt;
  cov_R_pred += config_.yaw_factor_noise * dt * dt;
  Eigen::Matrix4d center_cov = Eigen::Matrix4d::Zero();
  center_cov.topLeftCorner<3, 3>() = cov_X_pred;
  center_cov(3, 3) = cov_R_pred;
  auto armors = getArmorsFromTargetState(state);
  std::vector<std::vector<std::pair<ArmorPositionYaw, Eigen::Matrix4d>>>
      armors_covs_vec;
  for (const auto& obs : obs_armors_odom) {
    std::vector<std::pair<ArmorPositionYaw, Eigen::Matrix4d>> armors_covs;
    for (const auto& armor : armors) {
      Eigen::Vector3d offset = armor.position - state.center_position;
      Eigen::Matrix4d J = Eigen::Matrix4d::Identity();
      J(0, 3) = -offset.y();
      J(1, 3) = offset.x();
      Eigen::Matrix4d P = J * center_cov * J.transpose();
      Eigen::Matrix3d P_xyz = P.topLeftCorner<3, 3>();
      Eigen::Matrix3d J_sph = cartesian2SphericalJacobian(armor.position);
      P.topLeftCorner<3, 3>() = J_sph * P_xyz * J_sph.transpose();
      auto obs_center_yaw = std::atan2(obs.position.y(), obs.position.x());
      auto obs_incline_angle = limitRadian(obs.yaw.theta() - obs_center_yaw);
      Eigen::Vector4d R_dig;
      R_dig << config_.armor_match_conf.ypd_conf.yaw_pitch_noise,
               config_.armor_match_conf.ypd_conf.yaw_pitch_noise,
               std::log(config_.armor_match_conf.ypd_conf.distance_noise_log_scale *
                            std::abs(obs_incline_angle) +
                        1) +
                   config_.armor_match_conf.ypd_conf.basic_distance_noise,
               std::log(std::abs(obs.position.norm()) + 1) /
                       config_.armor_match_conf.ypd_conf.armor_yaw_log_divisor +
                   config_.armor_match_conf.ypd_conf.basic_armor_yaw_noise;
      Eigen::Matrix4d R_mat = R_dig.asDiagonal();
      Eigen::Matrix4d S = P + R_mat;
      armors_covs.emplace_back(armor, S);
    }
    armors_covs_vec.emplace_back(armors_covs);
  }
  std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>
      matched_armors;
  std::array<std::vector<std::pair<double, ArmorPositionRollPitchYawPoints>>, 4>
      index_vector_array;
  for (std::size_t i = 0;
       i < obs_armors_camera.size() && i < obs_armors_odom.size(); ++i) {
    const auto& obs = obs_armors_odom.at(i);
    auto result = matchArmor(armors_covs_vec.at(i), obs);
    if (result.empty()) continue;
    this->updateNisFailureDeque(result.front().distance);
    if (result.front().distance <
        config_.armor_match_conf.max_match_mahalanobis_distance) {
      index_vector_array.at(static_cast<int>(result.front().index))
          .emplace_back(result.front().distance, obs_armors_camera.at(i));
    }
  }
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

// ── NIS failure ──
void RobotTarget::updateNisFailureDeque(double distance) const {
  nis_failure_deque_.push_back(distance > config_.nis_conf.nis_failure_thres);
  if (nis_failure_deque_.size() > static_cast<size_t>(config_.nis_conf.nis_failure_window_size))
    nis_failure_deque_.pop_front();
}

std::optional<bool> RobotTarget::nisFailured() const {
  if (nis_failure_deque_.size() < static_cast<size_t>(config_.nis_conf.nis_failure_window_size))
    return std::nullopt;
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

// ── addMotionValuesFactors ──
void RobotTarget::addMotionValuesFactors(
    gtsam::Values& values, gtsam::NonlinearFactorGraph& graph,
    const TargetState& target_state, uint64_t k, double dt) const {
  values.insert(X(k), target_state.center_position);
  values.insert(R(k), gtsam::Rot2::fromAngle(target_state.center_yaw));
  values.insert(V(k), target_state.center_velocity);
  values.insert(W(k), target_state.center_vyaw);
  if (k == 0) {
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

// ── addArmorReprojValuesFactors ──
void RobotTarget::addArmorReprojValuesFactors(
    gtsam::Values& values, gtsam::NonlinearFactorGraph& graph,
    gtsam::Key armor_pose_key,
    const ArmorPositionRollPitchYawPoints& armor, uint64_t k) const {
  values.insert(armor_pose_key,
                gtsam::Pose3{gtsam::Rot3{armor.getRotation()}, armor.position});
  for (auto position : {ArmorPointPosition::LeftBottom,
                        ArmorPointPosition::LeftTop,
                        ArmorPointPosition::RightTop,
                        ArmorPointPosition::RightBottom}) {
    graph.add(ArmorReprojFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2{
            config_.armor_observation_noise.pixel_error.x(),
            config_.armor_observation_noise.pixel_error.y()}),
        armor_pose_key, camera_matrix_, distortion_coefficients_,
        target_state_.type, position,
        armor.points.at(static_cast<int>(position))});
  }
}

// ── addArmorValuesFactors ──
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

// ── update (core factor graph) ──
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
    this->isam2_.update(graph, values);
    target_state.center_position =
        isam2_.calculateEstimate<gtsam::Point3>(X(track_state_.k));
    target_state.center_velocity =
        isam2_.calculateEstimate<gtsam::Vector3>(V(track_state_.k));
    target_state.center_yaw =
        isam2_.calculateEstimate<gtsam::Rot2>(R(track_state_.k)).theta();
    target_state.center_vyaw =
        isam2_.calculateEstimate<double>(W(track_state_.k));
    target_state.radius_a = auto_graph::logisticFunction(
        isam2_.calculateEstimate<double>(A(0)), config_.radius_min,
        config_.radius_max);
    target_state.radius_b = auto_graph::logisticFunction(
        isam2_.calculateEstimate<double>(B(0)), config_.radius_min,
        config_.radius_max);
    target_state.dz = isam2_.calculateEstimate<double>(Z(0));
    this->X_cov_ = isam2_.marginalCovariance(X(track_state_.k));
    this->V_cov_ = isam2_.marginalCovariance(V(track_state_.k));
    this->R_cov_ = isam2_.marginalCovariance(R(track_state_.k))(0, 0);
    this->W_cov_ = isam2_.marginalCovariance(W(track_state_.k))(0, 0);
    return {target_state,
            matched_armors.empty() ? TrackState::State::TEMPLOST
                                    : TrackState::State::TRACKING};
  } catch (const std::exception& e) {
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

// Adapted from jlu_vision_26 auto_aim::RobotTarget
#pragma once

#include "filter_test/jlu_tracker/configs.hpp"
#include "filter_test/jlu_tracker/types.hpp"
#include "filter_test/jlu_tracker/factors.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

namespace jlu {

// GTSAM 4.3 key helpers (equivalent to symbol_shorthand in GTSAM develop)
inline gtsam::Key X(uint64_t k) { return gtsam::Symbol('x', k); }
inline gtsam::Key V(uint64_t k) { return gtsam::Symbol('v', k); }
inline gtsam::Key R(uint64_t k) { return gtsam::Symbol('r', k); }
inline gtsam::Key W(uint64_t k) { return gtsam::Symbol('w', k); }
inline gtsam::Key A(uint64_t k) { return gtsam::Symbol('a', k); }
inline gtsam::Key B(uint64_t k) { return gtsam::Symbol('b', k); }
inline gtsam::Key Z(uint64_t k) { return gtsam::Symbol('z', k); }

class RobotTarget {
 public:
  RobotTarget(const RobotConfig& config, ArmorType type,
              const cv::Mat& camera_matrix,
              const cv::Mat& distortion_coefficients);

  TrackState::State track(
      const std::vector<ArmorPositionRollPitchYawPoints>& armors,
      double stamp_sec, const Eigen::Isometry3d& T_camera_to_odom);

  std::pair<RobotTargetState, TrackState> getTargetTrackState() const;
  double get(const std::string& key) const;

  // Static helpers
  static std::vector<ArmorPositionYaw> getArmorsFromTargetState(
      const TargetState& state, double radius_a, double radius_b, double dz);

  static std::vector<ArmorMatchResult> matchArmor(
      const std::vector<ArmorPositionYaw>& armors, const ArmorPositionYaw& obs,
      double max_match_distance, double max_match_yaw_diff);

  static std::vector<ArmorMatchResult> matchArmor(
      const std::vector<std::pair<ArmorPositionYaw, Eigen::Matrix4d>>& armors_covs,
      const ArmorPositionYaw& obs);

 private:
  std::pair<RobotTargetState, TrackState::State> update(
      const std::vector<ArmorPositionRollPitchYawPoints>& armors, double dt,
      const Eigen::Isometry3d& T) const;

  std::vector<ArmorPositionYaw> getArmorsFromTargetState(
      const RobotTargetState& state) const;

  std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>> matchArmors(
      const RobotTargetState& state, double dt,
      const std::vector<ArmorPositionRollPitchYawPoints>& obs_armors_camera,
      const std::vector<ArmorPositionRollPitchYawPoints>& obs_armors_odom) const;

  RobotTargetState getTargetStateFromArmor(const ArmorPositionYaw& armor) const;

  void addMotionValuesFactors(gtsam::Values& values,
                              gtsam::NonlinearFactorGraph& graph,
                              const TargetState& target_state,
                              uint64_t k, double dt) const;
  void addArmorValuesFactors(
      gtsam::Values& values, gtsam::NonlinearFactorGraph& graph,
      const std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>&
          armors_indexs,
      const Eigen::Isometry3d& T, uint64_t k) const;
  void addArmorReprojValuesFactors(gtsam::Values& values,
                                   gtsam::NonlinearFactorGraph& graph,
                                   gtsam::Key armor_pose_key,
                                   const ArmorPositionRollPitchYawPoints& armor,
                                   uint64_t k) const;

  void resetCovariances();
  void updateNisFailureDeque(double distance) const;
  std::optional<bool> nisFailured() const;

  RobotConfig config_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  RobotTargetState target_state_;
  TrackState track_state_;
  mutable std::mutex state_mtx_;
  mutable gtsam::ISAM2 isam2_;
  mutable gtsam::Values initial_values_;
  mutable gtsam::NonlinearFactorGraph initial_graph_;
  mutable Eigen::Matrix3d X_cov_;
  mutable Eigen::Matrix3d V_cov_;
  mutable double R_cov_ = 0;
  mutable double W_cov_ = 0;
  mutable std::deque<bool> nis_failure_deque_;
};

}  // namespace jlu

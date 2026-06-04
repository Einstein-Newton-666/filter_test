// Adapted from jlu_vision_26 auto_aim::RobotTarget types
// Copyright (c) 2026 GuGuGaAaaaa. All Rights Reserved.
#pragma once

#include <auto_aim_interfaces/msg/armor.hpp>
#include <Eigen/Core>
#include <array>
#include <gtsam/geometry/Rot2.h>
#include <functional>
#include <vector>
#include <memory>

namespace jlu {

// ── ArmorType enum (simplified from jlu types::ArmorType) ──
enum class ArmorType { Small = 0, Big = 1, Outpost = 2, Base = 3 };

// ── ArmorIndex ──
enum class ArmorIndex { _0 = 0, _1, _2, _3 };

// ── ArmorPositionYaw ──
struct ArmorPositionYaw {
  ArmorPositionYaw() = default;
  explicit ArmorPositionYaw(const auto_aim_interfaces::msg::Armor& armor);
  ArmorPositionYaw(const Eigen::Vector3d& center_position, double center_yaw,
                   double radius, double dz, int armor_numbers,
                   ArmorIndex index);
  Eigen::Quaterniond getRotation(double pitch, double roll) const;
  Eigen::Vector3d position;
  gtsam::Rot2 yaw;
};

// ── ArmorPositionRollPitchYawPoints ──
struct ArmorPositionRollPitchYawPoints : public ArmorPositionYaw {
  ArmorPositionRollPitchYawPoints() = default;
  explicit ArmorPositionRollPitchYawPoints(
      const auto_aim_interfaces::msg::Armor& armor);
  Eigen::Quaterniond getRotation() const;
  std::array<Eigen::Vector2d, 4> points;
  double pitch = 0;
  double roll = 0;
};

// ── TargetState ──
struct TargetState {
  ArmorType type = ArmorType::Small;
  Eigen::Vector3d center_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_velocity = Eigen::Vector3d::Zero();
  double center_yaw = 0;
  double center_vyaw = 0;

  TargetState() = default;
  TargetState getStateWithArmorsFunc(
      std::function<std::vector<ArmorPositionYaw>(const TargetState& self)>
          get_armors_fn) const {
    auto state = *this;
    state.get_armors_ = std::move(get_armors_fn);
    return state;
  }
  TargetState predict(double dt) const;
  std::vector<ArmorPositionYaw> armors() const;

 private:
  std::function<std::vector<ArmorPositionYaw>(const TargetState& self)>
      get_armors_;
};

// ── RobotTargetState ──
struct RobotTargetState : public TargetState {
  RobotTargetState predict(double dt) const;
  double radius_a = 0.26;
  double radius_b = 0.26;
  double dz = 0.0;
};

// ── TrackState ──
struct TrackState {
  enum class State { LOST, TEMPLOST, TRACKING } state = State::LOST;
  uint64_t k = 0;
  double stamp_last_update = 0;     // seconds (rclcpp::Time)
  double stamp_last_tracking = 0;   // seconds
};

// ── ArmorMatchResult ──
struct ArmorMatchResult {
  ArmorIndex index;
  double distance = 0;  // mahalanobis
  double yaw_diff = 0;  // rad
  ArmorMatchResult() = default;
  ArmorMatchResult(ArmorIndex idx, double d, double y)
      : index(idx), distance(d), yaw_diff(y) {}
};

}  // namespace jlu

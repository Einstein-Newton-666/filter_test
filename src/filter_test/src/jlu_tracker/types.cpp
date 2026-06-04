// Adapted from jlu_vision_26 auto_aim tracker types
#include "filter_test/jlu_tracker/types.hpp"

#include <auto_aim_interfaces/msg/armor.hpp>
#include <gtsam/geometry/Rot2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <vector>

namespace jlu {

// ── Helper: extract RPY from ROS2 pose quaternion ──
static std::array<double, 3> getRpy(const geometry_msgs::msg::Quaternion& q_msg) {
  tf2::Quaternion q;
  tf2::fromMsg(q_msg, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}

static Eigen::Quaterniond rpyToQuaterniond(double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
}

// ── ArmorPositionYaw from ROS2 Armor msg ──
ArmorPositionYaw::ArmorPositionYaw(const auto_aim_interfaces::msg::Armor& armor) {
  this->position = Eigen::Vector3d(
      armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);
  auto [roll, pitch, yaw] = getRpy(armor.pose.orientation);
  this->yaw = gtsam::Rot2::fromAngle(yaw);
}

// ── ArmorPositionYaw from center state ──
ArmorPositionYaw::ArmorPositionYaw(const Eigen::Vector3d& center_position,
                                   double center_yaw, double radius, double dz,
                                   int armor_numbers, ArmorIndex index) {
  auto between_angle = (2 * M_PI) / armor_numbers;
  auto armor_yaw = center_yaw + static_cast<int>(index) * between_angle;
  auto armor_x = center_position.x() - radius * std::cos(armor_yaw);
  auto armor_y = center_position.y() - radius * std::sin(armor_yaw);
  auto armor_z = center_position.z() + dz;
  this->position = Eigen::Vector3d(armor_x, armor_y, armor_z);
  this->yaw = gtsam::Rot2::fromAngle(armor_yaw);
}

Eigen::Quaterniond ArmorPositionYaw::getRotation(double pitch,
                                                  double roll) const {
  return rpyToQuaterniond(roll, pitch, yaw.theta());
}

// ── ArmorPositionRollPitchYawPoints from ROS2 Armor msg ──
ArmorPositionRollPitchYawPoints::ArmorPositionRollPitchYawPoints(
    const auto_aim_interfaces::msg::Armor& armor)
    : ArmorPositionYaw(armor) {
  auto [r, p, y] = getRpy(armor.pose.orientation);
  this->pitch = p;
  this->roll = r;
  // Copy detected points (up to 4)
  for (size_t i = 0; i < 4 && i < armor.detected_points.size(); ++i) {
    this->points[i] = Eigen::Vector2d(
        armor.detected_points[i].x, armor.detected_points[i].y);
  }
}

Eigen::Quaterniond ArmorPositionRollPitchYawPoints::getRotation() const {
  return static_cast<const ArmorPositionYaw*>(this)->getRotation(pitch, roll);
}

// ── TargetState predict ──
TargetState TargetState::predict(double dt) const {
  TargetState state = *this;
  state.center_position = this->center_position + this->center_velocity * dt;
  state.center_yaw = this->center_yaw + this->center_vyaw * dt;
  return state;
}

RobotTargetState RobotTargetState::predict(double dt) const {
  RobotTargetState state = *this;
  state.center_position = this->center_position + this->center_velocity * dt;
  state.center_yaw = this->center_yaw + this->center_vyaw * dt;
  return state;
}

std::vector<ArmorPositionYaw> TargetState::armors() const {
  return get_armors_(*this);
}

}  // namespace jlu

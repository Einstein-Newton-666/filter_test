#pragma once

#include <cmath>
#include <functional>
#include <random>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>
#include <armor_simulation/msg/ground_truth.hpp>

#include "armor_simulation/camera_model.hpp"
#include "armor_simulation/armor_geometry.hpp"
#include "armor_simulation/detection_noise.hpp"

using namespace std::chrono_literals;

namespace armor_sim {

/**
 * 装甲板仿真器节点
 *
 * 运动模拟 → TF查询相机位姿 → 装甲板位姿 → 相机投影 → 像素噪声 → PnP → 发布
 *
 * 相机外参从 TF odom→camera_optical_frame 动态获取 (由 gimbal_simulation 节点提供)
 */
class ArmorSimulation : public rclcpp::Node {
public:
  ArmorSimulation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publishSimulation();

  // ── 运动学状态 ──
  double x, y;
  double z1, z2;
  double yaw;
  double x_v, y_v, yaw_v;
  double x_a, y_a, yaw_a;
  double r1, r2;

  // ── 相机模型 (外参每帧从 TF 更新) ──
  CameraModel camera_model_;
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

  // ── 噪声模型 ──
  std::unique_ptr<DetectionNoise> noise_model_;
  std::string enemy_color_;

  // ── 发布者 ──
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr detector_pub_;
  rclcpp::Publisher<armor_simulation::msg::GroundTruth>::SharedPtr ground_truth_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // ── 定时器 ──
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_t;

  // ── 参数 ──
  double publish_rate;
  double linear_acc, angle_acc;
  double linear_limit, angle_limit;
  double linear_speed_limit, angle_speed_limit;
  double process_noise_xy_, process_noise_yaw_;
  int noise_seed_;

  // ── 过程噪声 ──
  std::mt19937 rng_;
  std::normal_distribution<double> accel_noise_dist_;

  // ── 可视化 ──
  visualization_msgs::msg::Marker position_marker_, armor_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
};

}  // namespace armor_sim

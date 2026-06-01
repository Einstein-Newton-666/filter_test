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
#include <sensor_msgs/msg/image.hpp>

#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>
#include <auto_aim_interfaces/msg/send_data.hpp>
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
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::SendData>::SharedPtr gimbal_pub_;
  bool publish_image_ = false;
  bool publish_gimbal_gt_ = false;
  int image_width_ = 1920;
  int image_height_ = 1440;
  double cx_ = 720.0;
  double cy_ = 640.0;

  // ── 图像显示选项 ──
  struct ImageDisplay {
      bool ground_truth = true;   // 绿色真值投影
      bool noisy        = true;   // 红色带噪检测
      bool labels       = true;   // 装甲板索引标签
      bool crosshair    = true;   // 画面中心十字线
      double resize_scale = 1.0;  // 发布前缩放比例 (0.5=一半)
  } img_disp_;

  // ── 定时器 ──
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_t;

  // ── 参数 ──
  double publish_rate;
  double linear_acc, angle_acc;
  double linear_limit, angle_limit;
  double linear_speed_limit, angle_speed_limit;
  double process_noise_xy_, process_noise_yaw_;
  double radial_min_ = 3.0, radial_max_ = 7.0;
  int noise_seed_;

  // ── 过程噪声 ──
  std::mt19937 rng_;
  std::normal_distribution<double> accel_noise_dist_;

  // ── 可视化 ──
  visualization_msgs::msg::Marker position_marker_, gt_marker_, obs_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;

  // ── 仿真图像 (持久化缓冲区, 避免高频重分配) ──
  cv::Mat sim_image_;
};

}  // namespace armor_sim

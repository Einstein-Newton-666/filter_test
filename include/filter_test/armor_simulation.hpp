#pragma once

#include <cmath>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <filter_test/msg/simulation.hpp>

using namespace std::chrono_literals;

class ArmorSimulation : public rclcpp::Node {
public:
  ArmorSimulation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publishSimulation();
  
  // 创建四元数（绕Z轴旋转yaw）
  geometry_msgs::msg::Quaternion createQuaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  double x, y, z1, z2;
  double yaw;
  double x_v, y_v, yaw_v;
  double r1, r2;
  double x_a, y_a, yaw_a;

  rclcpp::Publisher<filter_test::msg::Simulation>::SharedPtr sim_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double last_t;

  double publish_rate;
  double linear_acc, angle_acc;
  double linear_limit, angle_limit;
  double linear_speed_limit, angle_speed_limit;

};

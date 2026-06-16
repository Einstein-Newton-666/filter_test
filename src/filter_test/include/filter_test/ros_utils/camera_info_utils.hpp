#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <rclcpp/node.hpp>

#include <array>
#include <string>

namespace filter_test {

// ROS node 层使用的相机标定缓存。Eigen 版本给 typed GTSAM 图因子使用，
// OpenCV Mat 版本给 jlu_tracker adapter 使用；GraphOptimizer core 不直接依赖 ROS。
struct CameraCalibration {
    Eigen::Matrix3d camera_matrix = Eigen::Matrix3d::Identity();
    std::array<double, 5> distortion = {0.0, 0.0, 0.0, 0.0, 0.0};
    cv::Mat cv_camera_matrix;
    cv::Mat cv_distortion;
    std::string camera_name;
    std::string camera_info_url;
};

// 从 camera_name + camera_info_url 加载 CameraInfo，并转换为当前各节点需要的 K/D。
// 这是 node 层 helper，避免在 graph_optimizer/* 中引入 camera_info_manager 或 ROS 参数语义。
CameraCalibration loadCameraCalibrationFromInfo(
    rclcpp::Node& node,
    const std::string& default_camera_name = "narrow_stereo",
    const std::string& default_camera_info_url =
        "package://armor_simulation/config/camera_info.yaml");

}  // namespace filter_test

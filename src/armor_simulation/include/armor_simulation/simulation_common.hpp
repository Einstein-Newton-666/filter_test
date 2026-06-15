#pragma once

#include "armor_simulation/camera_model.hpp"
#include "armor_simulation/detection_noise.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <memory>
#include <stdexcept>
#include <string>

namespace armor_sim {

// 两个仿真器共享的开关。具体 topic 名仍由各节点决定：
// armor -> /simulation/image，rune -> /rune_simulation/image。
struct ImagePublishParams {
    bool publish_image = false;
    double image_resize_scale = 1.0;
    bool publish_gimbal_gt = false;
};

// DetectionNoiseParams 是噪声模型本体；pixel_noise_enabled 和 seed 是仿真节点的
// 外层控制项，所以放在同一个读取结果里，避免 armor/rune 重复声明这些参数。
struct DetectionNoiseConfig {
    DetectionNoiseParams params;
    bool pixel_noise_enabled = true;
    uint32_t seed = 42;
};

// CameraInfoManager 需要和节点同生命周期，否则后续相机信息 reload/URL 状态会丢失。
// 因此 helper 把 manager 一并返回，由调用节点保存 unique_ptr。
struct CameraInfoLoadResult {
    CameraIntrinsics intrinsics;
    std::string camera_name;
    std::string camera_info_url;
    std::unique_ptr<camera_info_manager::CameraInfoManager> manager;
};

// 把 ROS 标准 CameraInfo 压缩成仿真内部的 Brown-Conrady 相机模型参数。
// 只使用前四个畸变项 k1/k2/p1/p2；k3 当前 CameraModel 固定为 0。
CameraIntrinsics cameraIntrinsicsFromCameraInfo(
    const sensor_msgs::msg::CameraInfo& camera_info);

// autoaim 风格内参入口：camera_name + camera_info_url。
// 这里故意不提供 camera_fx/fy fallback；URL 无效时直接抛异常，让启动阶段暴露配置错误。
CameraInfoLoadResult loadCameraIntrinsicsFromCameraInfo(
    rclcpp::Node& node,
    const std::string& default_camera_name = "narrow_stereo",
    const std::string& default_camera_info_url =
        "package://armor_simulation/config/camera_info.yaml");

DetectionNoiseConfig loadDetectionNoiseConfig(rclcpp::Node& node);

ImagePublishParams loadImagePublishParams(rclcpp::Node& node);

}  // namespace armor_sim

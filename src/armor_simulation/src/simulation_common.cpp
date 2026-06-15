#include "armor_simulation/simulation_common.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <sstream>

namespace armor_sim {
namespace {

std::string declareOrGetString(
    rclcpp::Node& node, const std::string& name, const std::string& default_value) {
    // 节点可能已经在更高层声明过公共参数；重复 declare 会抛异常。
    // 这里兼容“先声明再读取”和“helper 内声明”两种调用顺序。
    if (node.has_parameter(name)) {
        return node.get_parameter(name).as_string();
    }
    return node.declare_parameter<std::string>(name, default_value);
}

}  // namespace

CameraIntrinsics cameraIntrinsicsFromCameraInfo(
    const sensor_msgs::msg::CameraInfo& camera_info) {
    CameraIntrinsics intr;
    // sensor_msgs/CameraInfo::k 是 row-major 3x3:
    // [fx, s, cx, 0, fy, cy, 0, 0, 1]。当前仿真模型不使用 skew。
    intr.fx = camera_info.k[0];
    intr.fy = camera_info.k[4];
    intr.cx = camera_info.k[2];
    intr.cy = camera_info.k[5];
    if (camera_info.d.size() >= 4) {
        intr.k1 = camera_info.d[0];
        intr.k2 = camera_info.d[1];
        intr.p1 = camera_info.d[2];
        intr.p2 = camera_info.d[3];
    } else {
        throw std::runtime_error("CameraInfo distortion vector must contain at least 4 values");
    }
    if (camera_info.width == 0 || camera_info.height == 0) {
        throw std::runtime_error("CameraInfo width/height must be non-zero");
    }
    intr.image_width = static_cast<int>(camera_info.width);
    intr.image_height = static_cast<int>(camera_info.height);
    return intr;
}

CameraInfoLoadResult loadCameraIntrinsicsFromCameraInfo(
    rclcpp::Node& node,
    const std::string& default_camera_name,
    const std::string& default_camera_info_url) {
    CameraInfoLoadResult result;
    result.camera_name =
        declareOrGetString(node, "camera_name", default_camera_name);
    result.camera_info_url =
        declareOrGetString(node, "camera_info_url", default_camera_info_url);
    if (result.camera_info_url.empty()) {
        throw std::runtime_error("camera_info_url must not be empty");
    }

    result.manager =
        std::make_unique<camera_info_manager::CameraInfoManager>(&node, result.camera_name);
    // validateURL 只检查 URL 语法/可解析性；loadCameraInfo 才真正读取 YAML。
    // 两者任一失败都视为配置错误，不回退到硬编码内参。
    if (!result.manager->validateURL(result.camera_info_url) ||
        !result.manager->loadCameraInfo(result.camera_info_url)) {
        std::ostringstream oss;
        oss << "Failed to load camera_info_url '" << result.camera_info_url << "'";
        throw std::runtime_error(oss.str());
    }

    result.intrinsics = cameraIntrinsicsFromCameraInfo(result.manager->getCameraInfo());
    return result;
}

DetectionNoiseConfig loadDetectionNoiseConfig(rclcpp::Node& node) {
    DetectionNoiseConfig config;
    // 这些参数在 simulation_config.yaml 的 /** 中给默认值，节点块只覆盖差异。
    // declare_parameter 会让 ROS 参数覆盖 YAML，同时保留头文件默认值作为最终兜底。
    config.params.pixel_noise_optimal =
        node.declare_parameter<double>("pixel_noise_optimal",
                                       config.params.pixel_noise_optimal);
    config.params.pixel_noise_optimal_distance =
        node.declare_parameter<double>("pixel_noise_optimal_distance",
                                       config.params.pixel_noise_optimal_distance);
    config.params.pixel_noise_curvature =
        node.declare_parameter<double>("pixel_noise_curvature",
                                       config.params.pixel_noise_curvature);
    config.params.pixel_noise_common_ratio =
        node.declare_parameter<double>("pixel_noise_common_ratio",
                                       config.params.pixel_noise_common_ratio);
    config.params.use_outliers =
        node.declare_parameter<bool>("use_outliers", config.params.use_outliers);
    config.params.outlier_probability =
        node.declare_parameter<double>("outlier_probability",
                                       config.params.outlier_probability);
    config.params.outlier_std =
        node.declare_parameter<double>("outlier_std", config.params.outlier_std);
    config.params.min_detectable_area =
        node.declare_parameter<double>("min_detectable_area",
                                       config.params.min_detectable_area);
    config.params.max_detectable_distance =
        node.declare_parameter<double>("max_detectable_distance",
                                       config.params.max_detectable_distance);
    config.params.detection_probability =
        node.declare_parameter<double>("detection_probability",
                                       config.params.detection_probability);
    config.params.miss_probability =
        node.declare_parameter<double>("miss_probability",
                                       config.params.miss_probability);
    config.pixel_noise_enabled =
        node.declare_parameter<bool>("pixel_noise_enabled",
                                     config.pixel_noise_enabled);
    config.seed =
        static_cast<uint32_t>(node.declare_parameter<int>("noise_seed",
                                                         static_cast<int>(config.seed)));
    return config;
}

ImagePublishParams loadImagePublishParams(rclcpp::Node& node) {
    ImagePublishParams params;
    // 只读取跨仿真器通用的图像开关；装甲板专用的 labels/crosshair
    // 保留在 armor_simulation_node 内单独声明。
    params.publish_image =
        node.declare_parameter<bool>("publish_image", params.publish_image);
    params.image_resize_scale =
        node.declare_parameter<double>("image_resize_scale", params.image_resize_scale);
    params.publish_gimbal_gt =
        node.declare_parameter<bool>("publish_gimbal_gt", params.publish_gimbal_gt);
    return params;
}

}  // namespace armor_sim

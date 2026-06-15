#include "filter_test/camera_info_utils.hpp"

#include <camera_info_manager/camera_info_manager.hpp>

#include <stdexcept>
#include <sstream>

namespace filter_test {
namespace {

std::string declareOrGetString(
    rclcpp::Node& node, const std::string& name, const std::string& default_value) {
    // graph_optimizer_test/rune_graph_optimizer 会先声明参数再 loadConfig；
    // jlu_tracker 则直接调用 helper。这里同时支持两种路径。
    if (node.has_parameter(name)) {
        return node.get_parameter(name).as_string();
    }
    return node.declare_parameter<std::string>(name, default_value);
}

}  // namespace

CameraCalibration loadCameraCalibrationFromInfo(
    rclcpp::Node& node,
    const std::string& default_camera_name,
    const std::string& default_camera_info_url) {
    CameraCalibration calibration;
    calibration.camera_name =
        declareOrGetString(node, "camera_name", default_camera_name);
    calibration.camera_info_url =
        declareOrGetString(node, "camera_info_url", default_camera_info_url);
    if (calibration.camera_info_url.empty()) {
        throw std::runtime_error("camera_info_url must not be empty");
    }

    camera_info_manager::CameraInfoManager manager(&node, calibration.camera_name);
    // 和 autoaim 一样，CameraInfo 是唯一内参来源；URL 配错时直接让节点初始化失败。
    if (!manager.validateURL(calibration.camera_info_url) ||
        !manager.loadCameraInfo(calibration.camera_info_url)) {
        std::ostringstream oss;
        oss << "Failed to load camera_info_url '" << calibration.camera_info_url << "'";
        throw std::runtime_error(oss.str());
    }

    const auto info = manager.getCameraInfo();
    if (info.d.size() < 4) {
        throw std::runtime_error("CameraInfo distortion vector must contain at least 4 values");
    }
    // 保留完整 K，包括可能存在的 skew 项；GTSAM/投影 helper 会按自己的模型使用。
    calibration.camera_matrix << info.k[0], info.k[1], info.k[2],
                                 info.k[3], info.k[4], info.k[5],
                                 info.k[6], info.k[7], info.k[8];
    calibration.distortion[0] = info.d[0];
    calibration.distortion[1] = info.d[1];
    calibration.distortion[2] = info.d[2];
    calibration.distortion[3] = info.d[3];
    calibration.distortion[4] = info.d.size() >= 5 ? info.d[4] : 0.0;

    calibration.cv_camera_matrix = (cv::Mat_<double>(3, 3) <<
        info.k[0], info.k[1], info.k[2],
        info.k[3], info.k[4], info.k[5],
        info.k[6], info.k[7], info.k[8]);
    calibration.cv_distortion = (cv::Mat_<double>(1, 5) <<
        calibration.distortion[0], calibration.distortion[1],
        calibration.distortion[2], calibration.distortion[3],
        calibration.distortion[4]);
    return calibration;
}

}  // namespace filter_test

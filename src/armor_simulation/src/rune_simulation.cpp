#include "armor_simulation/camera_model.hpp"
#include "armor_simulation/detection_noise.hpp"
#include "armor_simulation/rune_geometry.hpp"
#include "armor_simulation/simulation_common.hpp"

#include <armor_simulation/msg/rune_ground_truth.hpp>
#include <auto_aim_interfaces/msg/rune_info.hpp>
#include <auto_aim_interfaces/msg/rune_targets.hpp>
#include <auto_aim_interfaces/msg/send_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace armor_sim {
namespace {
constexpr double kRuneBladeMarkerThinAxis = 0.01;
constexpr double kRuneBladeMarkerWideAxis = 0.15;


double polygonArea(const std::array<Eigen::Vector2d, 5>& pixels) {
    // 检测概率模型用四个扇叶外轮廓点面积衡量目标在图像里的有效尺寸；
    // R 标中心不参与面积计算。
    static constexpr std::array<size_t, 4> polygon{1, 2, 3, 4};
    double area = 0.0;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const auto& a = pixels[polygon[i]];
        const auto& b = pixels[polygon[(i + 1) % polygon.size()]];
        area += a.x() * b.y() - b.x() * a.y();
    }
    return std::abs(area) * 0.5;
}

auto_aim_interfaces::msg::Point2d toMsg(const Eigen::Vector2d& pixel) {
    auto_aim_interfaces::msg::Point2d out;
    out.x = static_cast<float>(pixel.x());
    out.y = static_cast<float>(pixel.y());
    return out;
}

auto_aim_interfaces::msg::RuneTarget makeRuneTarget(
    const std_msgs::msg::Header& header,
    const std::array<Eigen::Vector2d, 5>& pixels) {
    auto target = auto_aim_interfaces::msg::RuneTarget();
    target.header = header;
    target.is_detected = true;
    target.r_center = toMsg(pixels[0]);
    target.near_point = toMsg(pixels[1]);
    target.left_point = toMsg(pixels[2]);
    target.far_point = toMsg(pixels[3]);
    target.right_point = toMsg(pixels[4]);
    return target;
}

void setPoint(geometry_msgs::msg::Point& msg, const Eigen::Vector3d& point) {
    msg.x = point.x();
    msg.y = point.y();
    msg.z = point.z();
}

geometry_msgs::msg::Quaternion planeOrientation(double normal_yaw, double normal_pitch) {
    tf2::Quaternion q;
    q.setRPY(0.0, normal_pitch, normal_yaw);
    return tf2::toMsg(q);
}

geometry_msgs::msg::Quaternion eigenQuaternionToMsg(const Eigen::Quaterniond& q) {
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}

}  // namespace

class RuneSimulationNode : public rclcpp::Node {
public:
    explicit RuneSimulationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("rune_simulation_node", options) {
        publish_rate_ = declare_parameter<int>("publish_rate", 100);
        const auto legacy_mode = declare_parameter<std::string>("mode", "big");
        mode_ = declare_parameter<std::string>("rune_mode", legacy_mode);
        center_.x() = declare_parameter<double>("center.x", 0.0);
        center_.y() = declare_parameter<double>("center.y", -5.0);
        center_.z() = declare_parameter<double>("center.z", 1.5);
        normal_yaw_ = declare_parameter<double>("normal_yaw", M_PI_2);
        normal_pitch_ = declare_parameter<double>("normal_pitch", 0.0);
        roll_ = declare_parameter<double>("initial_roll", 0.0);
        const double legacy_small_velocity =
            declare_parameter<double>("small_rune_velocity", M_PI / 3.0);
        small_rune_velocity_ =
            declare_parameter<double>("small.angular_velocity", legacy_small_velocity);
        direction_ = declare_parameter<int>("direction", 1);
        require_front_facing_ = declare_parameter<bool>("require_front_facing", true);
        blade_switch_interval_ = declare_parameter<double>("blade_switch_interval", 3.0);
        const int blade_selector_seed = declare_parameter<int>("blade_selector_seed", 42);
        active_blades_ = std::make_unique<RuneActiveBladeSelector>(
            5, blade_switch_interval_, blade_selector_seed);

        // 与装甲板仿真共用 CameraInfo YAML；这样 rune 五点投影和图优化 PnP
        // 使用同一组 K/D，避免仿真端和求解端产生系统误差。
        auto camera_info = loadCameraIntrinsicsFromCameraInfo(*this);
        CameraIntrinsics intr = camera_info.intrinsics;
        image_width_ = intr.image_width;
        image_height_ = intr.image_height;
        camera_info_manager_ = std::move(camera_info.manager);
        camera_model_ = CameraModel(
            intr, CameraModel::fromEulerAngles(M_PI_2, -M_PI_2, 0.0, -0.045, 0.08557, 0.0));

        // 共用检测噪声读取逻辑，但 rune 节点在 YAML 中覆盖更大的
        // pixel_noise_optimal/curvature 来模拟细关键点的不稳定性。
        const auto noise_config = loadDetectionNoiseConfig(*this);
        noise_model_ = std::make_unique<DetectionNoise>(
            noise_config.params, noise_config.seed);
        pixel_noise_enabled_ = noise_config.pixel_noise_enabled;

        BigRuneVelocityConfig big_cfg;
        // 参数同时兼容旧命名 big_rune.* 和新命名 big.*。新命名优先生效，
        // 便于和 simulation_config.yaml 的分组保持一致。
        const int legacy_random_seed = declare_parameter<int>("big_rune_random_seed", 42);
        const int random_seed = declare_parameter<int>("big.random_seed", legacy_random_seed);
        big_cfg.random_seed = random_seed >= 0
            ? random_seed
            : static_cast<int>(std::random_device{}());
        big_cfg.direction = direction_;
        big_cfg.velocity_bias_range = readRange("big.velocity_bias_range", readRange(
            "big_rune.velocity_bias_range", big_cfg.velocity_bias_range));
        big_cfg.velocity_amplitude_range = readRange("big.velocity_amplitude_range", readRange(
            "big_rune.velocity_amplitude_range", big_cfg.velocity_amplitude_range));
        big_cfg.velocity_frequency_range = readRange("big.velocity_frequency_range", readRange(
            "big_rune.velocity_frequency_range", big_cfg.velocity_frequency_range));
        const double legacy_big_noise = declare_parameter<double>(
            "big_rune.velocity_noise_std", big_cfg.velocity_noise_std);
        big_cfg.velocity_noise_std =
            declare_parameter<double>("big.velocity_noise_std", legacy_big_noise);
        const double legacy_resample_interval = declare_parameter<double>(
            "big_rune.resample_interval", big_cfg.resample_interval);
        big_cfg.resample_interval =
            declare_parameter<double>("big.resample_interval", legacy_resample_interval);
        const double legacy_min_abs_velocity = declare_parameter<double>(
            "big_rune.min_abs_velocity", big_cfg.min_abs_velocity);
        big_cfg.min_abs_velocity =
            declare_parameter<double>("big.min_abs_velocity", legacy_min_abs_velocity);
        const double legacy_max_abs_velocity = declare_parameter<double>(
            "big_rune.max_abs_velocity", big_cfg.max_abs_velocity);
        big_cfg.max_abs_velocity =
            declare_parameter<double>("big.max_abs_velocity", legacy_max_abs_velocity);
        big_motion_ = std::make_unique<BigRuneMotionModel>(big_cfg);

        const auto image_params = loadImagePublishParams(*this);
        publish_image_ = image_params.publish_image;
        image_resize_scale_ = image_params.image_resize_scale;
        publish_gimbal_gt_ = image_params.publish_gimbal_gt;
        image_show_ground_truth_ = declare_parameter<bool>("image_show_ground_truth", true);
        image_show_noisy_ = declare_parameter<bool>("image_show_noisy", true);
        image_show_labels_ = declare_parameter<bool>("image_show_labels", true);
        image_show_crosshair_ = declare_parameter<bool>("image_show_crosshair", true);
        image_show_radius_lines_ = declare_parameter<bool>("image_show_radius_lines", true);

        tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

        targets_pub_ = create_publisher<auto_aim_interfaces::msg::RuneTargets>(
            "/rune_detector/rune_targets", rclcpp::SensorDataQoS());
        legacy_targets_pub_ = create_publisher<auto_aim_interfaces::msg::RuneTargets>(
            "/detector/rune_targets", rclcpp::SensorDataQoS());
        ground_truth_pub_ = create_publisher<armor_simulation::msg::RuneGroundTruth>(
            "/rune_simulation/ground_truth", 10);
        info_pub_ = create_publisher<auto_aim_interfaces::msg::RuneInfo>(
            "/rune_simulation/info", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/rune_simulation/marker", 10);
        if (publish_image_) {
            image_pub_ = create_publisher<sensor_msgs::msg::Image>(
                "/rune_simulation/image", rclcpp::SensorDataQoS());
        }
        if (publish_gimbal_gt_) {
            gimbal_pub_ = create_publisher<auto_aim_interfaces::msg::SendData>(
                "/send_pack", rclcpp::SensorDataQoS());
        }

        last_t_ = now();
        timer_ = create_wall_timer(
            1000ms / publish_rate_, std::bind(&RuneSimulationNode::publishFrame, this));

        RCLCPP_INFO(get_logger(), "Rune simulation initialized: mode=%s center=(%.2f, %.2f, %.2f)",
                    mode_.c_str(), center_.x(), center_.y(), center_.z());
    }

private:
    std::array<double, 2> readRange(
        const std::string& name, const std::array<double, 2>& fallback) {
        const auto values = declare_parameter<std::vector<double>>(
            name, std::vector<double>{fallback[0], fallback[1]});
        if (values.size() != 2) {
            RCLCPP_WARN(get_logger(), "Parameter %s must contain 2 values; using defaults",
                        name.c_str());
            return fallback;
        }
        return {values[0], values[1]};
    }

    bool updateCameraPose(const rclcpp::Time& stamp, Eigen::Vector3d& camera_pos_odom) {
        try {
            auto tf = tf2_buffer_->lookupTransform(
                "odom", "camera_optical_frame", stamp, rclcpp::Duration::from_seconds(0.1));
            Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                                 tf.transform.rotation.y, tf.transform.rotation.z);
            Eigen::Matrix3d R_c2o = q.toRotationMatrix();
            Eigen::Vector3d t_c2o(tf.transform.translation.x,
                                  tf.transform.translation.y,
                                  tf.transform.translation.z);
            // CameraModel 内部使用 world/odom -> camera 的外参，因此这里把 TF
            // 给出的 camera -> odom 变换取逆：R_o2c = R_c2o^T, t_o2c = -R_o2c*t_c2o。
            camera_model_.setExtrinsics(R_c2o.transpose(), -(R_c2o.transpose() * t_c2o));
            camera_pos_odom = t_c2o;
            return true;
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_SKIPFIRST(get_logger(), "TF lookup odom->camera_optical_frame: %s", e.what());
            camera_pos_odom.setZero();
            return false;
        }
    }

    void publishFrame() {
        const rclcpp::Time stamp = now();
        const double dt = std::min((stamp - last_t_).seconds(), 1.0);
        last_t_ = stamp;

        if (publish_image_ && (image_.empty() ||
            image_.cols != image_width_ || image_.rows != image_height_)) {
            image_ = cv::Mat(image_height_, image_width_, CV_8UC3);
        }
        if (publish_image_) {
            image_ = cv::Scalar(30, 30, 30);
        }

        if (mode_ == "small") {
            // 小符按固定角速度积分；direction 只控制旋转方向。
            vroll_ = (direction_ >= 0 ? 1.0 : -1.0) * std::abs(small_rune_velocity_);
            roll_ += vroll_ * dt;
        } else {
            // 大符每帧从随机正弦速度模型采样，再积分得到真实 roll。
            // 发布给图优化的仍然只是关键点像素，不直接泄漏曲线参数。
            roll_ = big_motion_->step(dt);
            vroll_ = big_motion_->velocity();
        }
        elapsed_time_ += std::max(0.0, dt);
        const int active_count = mode_ == "small" ? 1 : 2;
        active_blades_->update(elapsed_time_, active_count);

        Eigen::Vector3d camera_pos_odom = Eigen::Vector3d::Zero();
        if (!updateCameraPose(stamp, camera_pos_odom)) {
            return;
        }

        auto targets = auto_aim_interfaces::msg::RuneTargets();
        targets.header.stamp = stamp;
        targets.header.frame_id = "odom";

        int observed_count = 0;
        std::vector<int32_t> active_blades;
        std::vector<int32_t> observed_blades;
        marker_array_.markers.clear();

        for (int i = 0; i < 5; ++i) {
            const bool blade_active = active_blades_->isActive(i);
            if (blade_active) active_blades.push_back(i);

            const double blade_roll = roll_ + static_cast<double>(i) * RUNE_SLOT_ANGLE;
            // 先在 odom 中生成五个 3D 关键点，再统一经过相机模型投影。
            // 这里的 normal_pitch 是固定符面倾斜；图优化不会从观测里估计它。
            const auto points = computeRuneBladeWorldPoints(
                center_, normal_yaw_, normal_pitch_, blade_roll);
            const Eigen::Matrix3d R =
                runeBaseRotation(normal_yaw_, normal_pitch_) *
                Eigen::AngleAxisd(blade_roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
            const Eigen::Vector3d normal = R * Eigen::Vector3d::UnitX();
            // 可见性规则：只发布朝向相机的一面，避免背面扇叶产生不真实观测。
            // 调试时可关闭 require_front_facing_，强制所有进入画面的扇叶可检测。
            const bool front_facing =
                normal.dot(camera_pos_odom - center_) > 0.0 || !require_front_facing_;

            std::array<Eigen::Vector2d, 5> pixels;
            bool all_in_image = front_facing;
            for (size_t j = 0; j < points.size(); ++j) {
                pixels[j] = camera_model_.project3Dto2D(points[j]);
                if (!camera_model_.isInImage(pixels[j], 10.0)) {
                    all_in_image = false;
                }
            }
            const Eigen::Vector2d hit_pixel = camera_model_.project3Dto2D(
                center_ + R * Eigen::Vector3d(0.0, 0.0, RUNE_BLADE_RADIUS));

            const double distance = (points[0] - camera_pos_odom).norm();
            const double area = all_in_image ? polygonArea(pixels) : 0.0;
            if (blade_active && all_in_image && noise_model_->shouldDetect(distance, area)) {
                // 噪声模型复用装甲板仿真的 U 形距离噪声 + 目标级相关噪声：
                // 五个关键点会同时带一个共同偏移，再叠加各自独立扰动。
                auto noisy = pixel_noise_enabled_
                    ? noise_model_->addCorrelatedPixelNoise(pixels, distance)
                    : pixels;
                // 消息点顺序和 runeBladeLocalPoints() 一一对应，图优化按同样顺序建重投影因子。
                targets.targets.push_back(makeRuneTarget(targets.header, noisy));
                observed_count++;
                observed_blades.push_back(i);
                if (image_show_noisy_) {
                    drawBlade(noisy, i, true, pixels[0], hit_pixel);
                }
            }

            if (image_show_ground_truth_) {
                drawBlade(pixels, i, false, pixels[0], hit_pixel);
            }
        }
        legacy_targets_pub_->publish(targets);
        targets_pub_->publish(targets);
        publishGroundTruth(stamp, active_blades);
        const int target_blade_index = selectRuneTargetBlade(observed_blades, active_blades);
        const Eigen::Vector3d target_position = targetPosition(target_blade_index);
        addGraphStyleMarkers(stamp, target_position, observed_blades);
        publishInfo(stamp, observed_count, target_position);
        marker_pub_->publish(marker_array_);
        marker_array_.markers.clear();
        publishImage(stamp, observed_count);
        publishGimbalTruth(stamp, target_position);
    }

    void drawBlade(
        const std::array<Eigen::Vector2d, 5>& pixels,
        int blade_index,
        bool noisy,
        const Eigen::Vector2d& center_pixel,
        const Eigen::Vector2d& hit_pixel) {
        if (!publish_image_) return;
        const cv::Scalar color = noisy ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 180, 0);
        std::array<cv::Point, 5> pts;
        for (size_t i = 0; i < pixels.size(); ++i) {
            pts[i] = cv::Point(static_cast<int>(std::round(pixels[i].x())),
                               static_cast<int>(std::round(pixels[i].y())));
        }
        static constexpr std::array<size_t, 4> polygon{1, 2, 3, 4};
        for (size_t i = 0; i < polygon.size(); ++i) {
            cv::line(image_, pts[polygon[i]], pts[polygon[(i + 1) % polygon.size()]], color, 1);
        }
        if (image_show_radius_lines_) {
            cv::line(
                image_,
                cv::Point(static_cast<int>(std::round(center_pixel.x())),
                          static_cast<int>(std::round(center_pixel.y()))),
                cv::Point(static_cast<int>(std::round(hit_pixel.x())),
                          static_cast<int>(std::round(hit_pixel.y()))),
                color, 1);
        }
        cv::circle(image_, pts[0], noisy ? 4 : 2, color, -1);
        if (image_show_labels_) {
            cv::putText(image_, std::to_string(blade_index), pts[0] + cv::Point(8, -8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
    }

    Eigen::Vector3d targetPosition(int blade_index) const {
        return computeRuneBladeTargetPosition(
            center_, normal_yaw_, normal_pitch_, roll_, blade_index);
    }

    void addGraphStyleMarkers(
        const rclcpp::Time& stamp,
        const Eigen::Vector3d& target_position,
        const std::vector<int32_t>& observed_blades) {
        visualization_msgs::msg::Marker center;
        center.header.stamp = stamp;
        center.header.frame_id = "odom";
        center.ns = "rune_center";
        center.id = 0;
        center.type = visualization_msgs::msg::Marker::SPHERE;
        center.action = visualization_msgs::msg::Marker::ADD;
        center.lifetime = rclcpp::Duration::from_seconds(0.1);
        center.scale.x = center.scale.y = center.scale.z = 0.06;
        center.color.a = 1.0;
        center.color.g = 1.0;
        setPoint(center.pose.position, center_);
        center.pose.orientation = planeOrientation(normal_yaw_, normal_pitch_);
        marker_array_.markers.push_back(center);

        for (int i = 0; i < 5; ++i) {
            const double blade_roll = roll_ + static_cast<double>(i) * RUNE_SLOT_ANGLE;
            const Eigen::Matrix3d R =
                runeBaseRotation(normal_yaw_, normal_pitch_) *
                Eigen::AngleAxisd(blade_roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
            addBladeMarkers(
                stamp, "rune", i,
                center_ + R * Eigen::Vector3d(0.0, 0.0, RUNE_BLADE_RADIUS),
                Eigen::Quaterniond(R),
                0.0f, 1.0f, 0.0f, 0.5f);
        }

        int obs_id = 0;
        for (const int32_t blade_index : observed_blades) {
            const double blade_roll = roll_ + static_cast<double>(blade_index) * RUNE_SLOT_ANGLE;
            const Eigen::Matrix3d R =
                runeBaseRotation(normal_yaw_, normal_pitch_) *
                Eigen::AngleAxisd(blade_roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
            visualization_msgs::msg::Marker obs;
            obs.header.stamp = stamp;
            obs.header.frame_id = "odom";
            obs.ns = "observed_position";
            obs.id = obs_id++;
            obs.type = visualization_msgs::msg::Marker::SPHERE;
            obs.action = visualization_msgs::msg::Marker::ADD;
            obs.lifetime = rclcpp::Duration::from_seconds(0.1);
            obs.scale.x = kRuneBladeMarkerThinAxis;
            obs.scale.y = kRuneBladeMarkerWideAxis;
            obs.scale.z = kRuneBladeMarkerWideAxis;
            obs.color.r = 0.5f;
            obs.color.g = 0.0f;
            obs.color.b = 0.5f;
            obs.color.a = 0.5f;
            setPoint(obs.pose.position,
                     center_ + R * Eigen::Vector3d(0.0, 0.0, RUNE_BLADE_RADIUS));
            obs.pose.orientation = eigenQuaternionToMsg(Eigen::Quaterniond(R));
            marker_array_.markers.push_back(obs);
        }

        visualization_msgs::msg::Marker text;
        text.header.stamp = stamp;
        text.header.frame_id = "odom";
        text.ns = "rune_status";
        text.id = 0;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.lifetime = rclcpp::Duration::from_seconds(0.1);
        setPoint(text.pose.position, target_position);
        text.pose.position.z += 0.12;
        text.scale.z = 0.08;
        text.color.a = 1.0;
        text.color.b = 1.0;
        text.text = "roll=" + std::to_string(roll_) +
            " obs=" + std::to_string(observed_blades.size());
        marker_array_.markers.push_back(text);
    }

    void addBladeMarkers(
        const rclcpp::Time& stamp,
        const std::string& ns,
        int id,
        const Eigen::Vector3d& blade_point,
        const Eigen::Quaterniond& blade_orientation,
        float r,
        float g,
        float b,
        float a) {
        visualization_msgs::msg::Marker blade;
        blade.header.stamp = stamp;
        blade.header.frame_id = "odom";
        blade.ns = ns;
        blade.id = id;
        blade.type = visualization_msgs::msg::Marker::SPHERE;
        blade.action = visualization_msgs::msg::Marker::ADD;
        blade.lifetime = rclcpp::Duration::from_seconds(0.1);
        blade.scale.x = kRuneBladeMarkerThinAxis;
        blade.scale.y = kRuneBladeMarkerWideAxis;
        blade.scale.z = kRuneBladeMarkerWideAxis;
        blade.color.r = r;
        blade.color.g = g;
        blade.color.b = b;
        blade.color.a = a;
        setPoint(blade.pose.position, blade_point);
        blade.pose.orientation = eigenQuaternionToMsg(blade_orientation);
        marker_array_.markers.push_back(blade);

        visualization_msgs::msg::Marker line;
        line.header.stamp = stamp;
        line.header.frame_id = "odom";
        line.ns = ns;
        line.id = id + 10;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.lifetime = rclcpp::Duration::from_seconds(0.1);
        line.scale.x = 0.005;
        line.color.r = r;
        line.color.g = g;
        line.color.b = b;
        line.color.a = a * 0.5f;
        geometry_msgs::msg::Point center_point;
        geometry_msgs::msg::Point blade_point_msg;
        setPoint(center_point, center_);
        setPoint(blade_point_msg, blade_point);
        line.points = {center_point, blade_point_msg};
        marker_array_.markers.push_back(line);
    }

    void publishGroundTruth(const rclcpp::Time& stamp, const std::vector<int32_t>& active_blades) {
        auto truth = armor_simulation::msg::RuneGroundTruth();
        truth.header.stamp = stamp;
        truth.header.frame_id = "odom";
        truth.x = center_.x();
        truth.y = center_.y();
        truth.z = center_.z();
        truth.normal_yaw = normal_yaw_;
        truth.normal_pitch = normal_pitch_;
        truth.roll = roll_;
        truth.vroll = vroll_;
        truth.mode = mode_;
        truth.active_blades = active_blades;
        ground_truth_pub_->publish(truth);
    }

    void publishInfo(
        const rclcpp::Time& stamp,
        int observed_count,
        const Eigen::Vector3d& target_position) {
        auto info = auto_aim_interfaces::msg::RuneInfo();
        info.header.stamp = stamp;
        info.header.frame_id = "odom";
        info.mode = mode_;
        info.tracking = observed_count > 0;
        info.observed_count = observed_count;
        info.normal_yaw = normal_yaw_;
        info.normal_pitch = normal_pitch_;
        info.optimized_roll = roll_;
        info.predicted_roll = roll_ + vroll_ * 0.15;
        info.angular_velocity = vroll_;
        // RuneInfo 同时发布真实曲线参数，方便调试大符拟合误差；
        // RuneTargets 观测本身不包含这些真值。
        const auto& params = big_motion_->params();
        info.curve_bias = params.bias;
        info.curve_amplitude = params.amplitude;
        info.curve_frequency = params.frequency;
        info.curve_phase = params.phase;
        setPoint(info.target_position, target_position);
        info_pub_->publish(info);
    }

    void publishImage(const rclcpp::Time& stamp, int observed_count) {
        if (!publish_image_ || !image_pub_) return;
        cv::putText(image_, "rune v:" + std::to_string(observed_count),
                    cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(200, 200, 200), 1);
        if (image_show_crosshair_) {
            const cv::Point center(image_.cols / 2, image_.rows / 2);
            cv::line(image_, center + cv::Point(-20, 0), center + cv::Point(20, 0),
                     cv::Scalar(120, 120, 120), 1);
            cv::line(image_, center + cv::Point(0, -20), center + cv::Point(0, 20),
                     cv::Scalar(120, 120, 120), 1);
        }
        const cv::Mat* out = &image_;
        cv::Mat resized;
        const double scale = std::clamp(image_resize_scale_, 0.1, 1.0);
        if (std::abs(scale - 1.0) > 1e-6) {
            cv::resize(image_, resized, cv::Size(), scale, scale, cv::INTER_NEAREST);
            out = &resized;
        }

        auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
        img_msg->header.stamp = stamp;
        img_msg->header.frame_id = "camera_optical_frame";
        img_msg->height = out->rows;
        img_msg->width = out->cols;
        img_msg->encoding = "bgr8";
        img_msg->is_bigendian = false;
        img_msg->step = static_cast<uint32_t>(out->step);
        const size_t size = out->rows * out->step;
        img_msg->data.assign(out->data, out->data + size);
        image_pub_->publish(std::move(img_msg));
    }

    void publishGimbalTruth(const rclcpp::Time& stamp, const Eigen::Vector3d& target) {
        if (!publish_gimbal_gt_ || !gimbal_pub_) return;
        auto msg = auto_aim_interfaces::msg::SendData();
        msg.header.stamp = stamp;
        // 能量机关直控真值瞄准当前帧自动选择扇叶的击打中心，
        // 而不是 R 标中心；这样可以只验证云台/投影链路。
        const double yaw = std::atan2(target.y(), target.x());
        const double pitch = -std::atan2(
            target.z(), std::hypot(target.x(), target.y()));
        msg.yaw = static_cast<float>(yaw * 180.0 / M_PI);
        msg.pitch = static_cast<float>(pitch * 180.0 / M_PI);
        gimbal_pub_->publish(msg);
    }

    int publish_rate_ = 100;
    std::string mode_;
    Eigen::Vector3d center_{0.0, -5.0, 1.5};
    double normal_yaw_ = M_PI_2;
    double normal_pitch_ = 0.0;
    double roll_ = 0.0;
    double vroll_ = 0.0;
    double elapsed_time_ = 0.0;
    double small_rune_velocity_ = 1.2;
    int direction_ = 1;
    double blade_switch_interval_ = 3.0;
    bool require_front_facing_ = true;

    CameraModel camera_model_;
    std::unique_ptr<DetectionNoise> noise_model_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    std::unique_ptr<BigRuneMotionModel> big_motion_;
    std::unique_ptr<RuneActiveBladeSelector> active_blades_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    bool pixel_noise_enabled_ = true;
    bool publish_image_ = false;
    bool image_show_ground_truth_ = true;
    bool image_show_noisy_ = true;
    bool image_show_labels_ = true;
    bool image_show_crosshair_ = true;
    bool image_show_radius_lines_ = true;
    bool publish_gimbal_gt_ = false;
    int image_width_ = 1920;
    int image_height_ = 1440;
    double image_resize_scale_ = 1.0;
    cv::Mat image_;

    rclcpp::Time last_t_;
    rclcpp::TimerBase::SharedPtr timer_;
    visualization_msgs::msg::MarkerArray marker_array_;
    rclcpp::Publisher<auto_aim_interfaces::msg::RuneTargets>::SharedPtr targets_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::RuneTargets>::SharedPtr legacy_targets_pub_;
    rclcpp::Publisher<armor_simulation::msg::RuneGroundTruth>::SharedPtr ground_truth_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::RuneInfo>::SharedPtr info_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::SendData>::SharedPtr gimbal_pub_;
};

}  // namespace armor_sim

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<armor_sim::RuneSimulationNode>());
    rclcpp::shutdown();
    return 0;
}

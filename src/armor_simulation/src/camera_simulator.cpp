#include <rclcpp/rclcpp.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "armor_simulation/camera_model.hpp"
#include "armor_simulation/armor_geometry.hpp"
#include "armor_simulation/detection_noise.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <random>

namespace armor_sim {

/**
 * 2D相机仿真器节点
 *
 * 订阅3D仿真器的输出，投影到2D图像平面，添加检测噪声，
 * 发布兼容autoaim格式的Armors消息。
 */
class CameraSimulator : public rclcpp::Node {
public:
    CameraSimulator(const rclcpp::NodeOptions& options)
        : Node("camera_simulator", options) {
        // 加载参数
        loadParameters();

        // 初始化相机模型
        initCameraModel();

        // 初始化噪声模型
        initNoiseModel();

        // 创建订阅者（订阅仿真器的全部装甲板）
        armors_sub_ = create_subscription<auto_aim_interfaces::msg::Armors>(
            "/armor_simulation/all_armors", rclcpp::SensorDataQoS(),
            [this](const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
                simulationCallback(msg);
            });

        // 创建发布者（发布兼容autoaim的Armors消息）
        armors_pub_ = create_publisher<auto_aim_interfaces::msg::Armors>(
            "/detector/armors", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(), "Camera simulator initialized");
    }

private:
    void loadParameters() {
        // 相机内参
        declare_parameter("fx", 1500.0);
        declare_parameter("fy", 1500.0);
        declare_parameter("cx", 640.0);
        declare_parameter("cy", 360.0);
        declare_parameter("k1", -0.1);
        declare_parameter("k2", 0.01);
        declare_parameter("p1", 0.0);
        declare_parameter("p2", 0.0);
        declare_parameter("image_width", 1280);
        declare_parameter("image_height", 720);

        // 相机外参
        declare_parameter("extrinsic_roll", 0.0);
        declare_parameter("extrinsic_pitch", -0.26);
        declare_parameter("extrinsic_yaw", 0.0);
        declare_parameter("extrinsic_tx", 0.0);
        declare_parameter("extrinsic_ty", 0.0);
        declare_parameter("extrinsic_tz", 0.0);

        // 检测噪声
        declare_parameter("pixel_noise_std", 2.0);
        declare_parameter("min_detectable_area", 100.0);
        declare_parameter("max_detectable_distance", 8.0);
        declare_parameter("detection_probability", 0.95);
        declare_parameter("miss_probability", 0.05);
        declare_parameter("use_noisy_pose", true);
        declare_parameter("pose_noise_std", 0.01);
        declare_parameter("seed", 42);

        // 敌方颜色
        declare_parameter("enemy_color", "blue");
    }

    void initCameraModel() {
        CameraIntrinsics intr;
        intr.fx = get_parameter("fx").as_double();
        intr.fy = get_parameter("fy").as_double();
        intr.cx = get_parameter("cx").as_double();
        intr.cy = get_parameter("cy").as_double();
        intr.k1 = get_parameter("k1").as_double();
        intr.k2 = get_parameter("k2").as_double();
        intr.p1 = get_parameter("p1").as_double();
        intr.p2 = get_parameter("p2").as_double();
        intr.image_width = get_parameter("image_width").as_int();
        intr.image_height = get_parameter("image_height").as_int();

        double roll = get_parameter("extrinsic_roll").as_double();
        double pitch = get_parameter("extrinsic_pitch").as_double();
        double yaw = get_parameter("extrinsic_yaw").as_double();
        double tx = get_parameter("extrinsic_tx").as_double();
        double ty = get_parameter("extrinsic_ty").as_double();
        double tz = get_parameter("extrinsic_tz").as_double();

        CameraExtrinsics extr = CameraModel::fromEulerAngles(roll, pitch, yaw, tx, ty, tz);
        camera_model_ = CameraModel(intr, extr);

        RCLCPP_INFO(get_logger(), "Camera model initialized: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
                    intr.fx, intr.fy, intr.cx, intr.cy);
    }

    void initNoiseModel() {
        DetectionNoiseParams params;
        params.pixel_noise_std = get_parameter("pixel_noise_std").as_double();
        params.min_detectable_area = get_parameter("min_detectable_area").as_double();
        params.max_detectable_distance = get_parameter("max_detectable_distance").as_double();
        params.detection_probability = get_parameter("detection_probability").as_double();
        params.miss_probability = get_parameter("miss_probability").as_double();
        params.use_noisy_pose = get_parameter("use_noisy_pose").as_bool();
        params.pose_noise_std = get_parameter("pose_noise_std").as_double();

        uint32_t seed = get_parameter("seed").as_int();
        noise_model_ = std::make_unique<DetectionNoise>(params, seed);

        enemy_color_ = get_parameter("enemy_color").as_string();

        RCLCPP_INFO(get_logger(), "Noise model initialized: pixel_std=%.2f, enemy_color=%s",
                    params.pixel_noise_std, enemy_color_.c_str());
    }

    void simulationCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
        auto_aim_interfaces::msg::Armors output;
        output.header = msg->header;
        output.header.frame_id = "camera_optical_frame";

        for (const auto& armor : msg->armors) {
            // 过滤敌方颜色
            if (armor.color != enemy_color_) {
                continue;
            }

            // 获取装甲板位姿
            Eigen::Vector3d position(
                armor.pose.position.x,
                armor.pose.position.y,
                armor.pose.position.z);

            Eigen::Quaterniond orientation(
                armor.pose.orientation.w,
                armor.pose.orientation.x,
                armor.pose.orientation.y,
                armor.pose.orientation.z);

            // 计算距离
            double distance = position.norm();

            // 计算4个角点
            double width = (armor.type == "large") ? LARGE_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;
            double height = (armor.type == "large") ? LARGE_ARMOR_HEIGHT : SMALL_ARMOR_HEIGHT;
            auto corners = computeArmorCorners(position, orientation, width, height);

            // 计算像素面积
            double area = camera_model_.computeArea(corners);

            // 判断是否应该检测到
            if (!noise_model_->shouldDetect(distance, area)) {
                continue;
            }

            // 投影4个角点到2D
            std::array<Eigen::Vector2d, 4> pixels;
            bool all_in_image = true;
            for (int i = 0; i < 4; i++) {
                pixels[i] = camera_model_.project3Dto2D(corners[i]);
                if (!camera_model_.isInImage(pixels[i], 10.0)) {
                    all_in_image = false;
                    break;
                }
            }

            // 检查是否所有角点都在图像内
            if (!all_in_image) {
                continue;
            }

            // 添加像素噪声
            for (int i = 0; i < 4; i++) {
                pixels[i] = noise_model_->addPixelNoise(pixels[i]);
            }

            // 添加位姿噪声
            Eigen::Vector3d noisy_position = noise_model_->addPoseNoise(position, distance);

            // 构建输出Armor消息
            auto_aim_interfaces::msg::Armor output_armor;
            output_armor.number = armor.number;
            output_armor.type = armor.type;
            output_armor.color = armor.color;
            output_armor.area = area;
            output_armor.distance_to_image_center = camera_model_.distanceToImageCenter(
                camera_model_.computeCenter(corners));

            // 设置位姿（带噪声）
            output_armor.pose.position.x = noisy_position.x();
            output_armor.pose.position.y = noisy_position.y();
            output_armor.pose.position.z = noisy_position.z();
            output_armor.pose.orientation = armor.pose.orientation;

            // 设置检测点（左下、左上、右上、右下）
            output_armor.detected_points[0].x = pixels[0].x();
            output_armor.detected_points[0].y = pixels[0].y();
            output_armor.detected_points[0].z = 0;
            output_armor.detected_points[1].x = pixels[1].x();
            output_armor.detected_points[1].y = pixels[1].y();
            output_armor.detected_points[1].z = 0;
            output_armor.detected_points[2].x = pixels[2].x();
            output_armor.detected_points[2].y = pixels[2].y();
            output_armor.detected_points[2].z = 0;
            output_armor.detected_points[3].x = pixels[3].x();
            output_armor.detected_points[3].y = pixels[3].y();
            output_armor.detected_points[3].z = 0;

            // 设置yaw
            tf2::Quaternion tf_q;
            tf2::fromMsg(armor.pose.orientation, tf_q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
            output_armor.yaw = yaw;
            output_armor.optimized_yaw = yaw;

            output.armors.push_back(output_armor);
        }

        // 发布
        armors_pub_->publish(output);
    }

    // 相机模型
    CameraModel camera_model_;

    // 噪声模型
    std::unique_ptr<DetectionNoise> noise_model_;

    // 敌方颜色
    std::string enemy_color_;

    // ROS2接口
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
};

}  // namespace armor_sim

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<armor_sim::CameraSimulator>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

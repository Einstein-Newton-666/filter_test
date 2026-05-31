#include "armor_simulation/armor_simulation.hpp"
#include <algorithm>

namespace armor_sim {

ArmorSimulation::ArmorSimulation(const rclcpp::NodeOptions & options)
: Node("armor_simulation_node", options) {
    // ═══════════════════════════════════════════════════════
    // 运动参数
    // ═══════════════════════════════════════════════════════
    publish_rate       = declare_parameter<int>("publish_rate", 15);
    linear_limit       = declare_parameter<double>("linear_limit", 3.5);
    angle_limit        = declare_parameter<double>("angle_limit", 10.0);
    linear_speed_limit = declare_parameter<double>("linear_speed_limit", 1.2);
    angle_speed_limit  = declare_parameter<double>("angle_speed_limit", 10.0);
    linear_acc         = declare_parameter<double>("linear_acceleration", 0.2);
    angle_acc          = declare_parameter<double>("angular_acceleration", 0.3924);

    x     = declare_parameter<double>("initial_state.x", 0.0);
    y     = declare_parameter<double>("initial_state.y", -5.0);
    yaw   = declare_parameter<double>("initial_state.yaw", 0.1);
    x_v   = declare_parameter<double>("initial_state.x_velocity", 0.0);
    y_v   = declare_parameter<double>("initial_state.y_velocity", -0.8);
    yaw_v = declare_parameter<double>("initial_state.yaw_velocity", 6.0);
    x_a   = declare_parameter<double>("initial_state.x_acceleration", 0.0);
    y_a   = declare_parameter<double>("initial_state.y_acceleration", 0.0);
    yaw_a = declare_parameter<double>("initial_state.yaw_acceleration", 0.1);

    z1 = declare_parameter<double>("geometry.z1", 0.0);
    z2 = declare_parameter<double>("geometry.z2", 0.15);
    r1 = declare_parameter<double>("geometry.r1", 0.28);
    r2 = declare_parameter<double>("geometry.r2", 0.38);

    process_noise_xy_  = declare_parameter<double>("process_noise_xy", 0.1);
    process_noise_yaw_ = declare_parameter<double>("process_noise_yaw", 0.5);
    noise_seed_        = declare_parameter<int>("process_noise_seed", 42);
    rng_ = std::mt19937(noise_seed_);
    accel_noise_dist_  = std::normal_distribution<double>(0.0, 1.0);

    // ═══════════════════════════════════════════════════════
    // 相机模型 (内参固定, 外参每帧从 TF 更新)
    // ═══════════════════════════════════════════════════════
    CameraIntrinsics intr;
    intr.fx           = declare_parameter<double>("camera_fx", 2411.0);
    intr.fy           = declare_parameter<double>("camera_fy", 2411.0);
    intr.cx           = declare_parameter<double>("camera_cx", 720.0);
    intr.cy           = declare_parameter<double>("camera_cy", 640.0);
    intr.k1           = declare_parameter<double>("camera_k1", -0.093);
    intr.k2           = declare_parameter<double>("camera_k2", 0.154);
    intr.p1           = declare_parameter<double>("camera_p1", 0.0001);
    intr.p2           = declare_parameter<double>("camera_p2", -0.0006);
    intr.image_width  = declare_parameter<int>("image_width", 1920);
    intr.image_height = declare_parameter<int>("image_height", 1440);
    camera_model_ = CameraModel(intr,
        CameraModel::fromEulerAngles(-1.5707963, 0.0, 0.0, 0.0, -0.045, 0.08557));

    // TF2 — 动态获取相机外参
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    // ═══════════════════════════════════════════════════════
    // 噪声模型
    // ═══════════════════════════════════════════════════════
    DetectionNoiseParams noise_params;
    noise_params.pixel_noise_optimal          = declare_parameter<double>("pixel_noise_optimal", 1.5);
    noise_params.pixel_noise_optimal_distance = declare_parameter<double>("pixel_noise_optimal_distance", 5.0);
    noise_params.pixel_noise_curvature        = declare_parameter<double>("pixel_noise_curvature", 0.125);
    noise_params.use_outliers                 = declare_parameter<bool>("use_outliers", true);
    noise_params.outlier_probability          = declare_parameter<double>("outlier_probability", 0.05);
    noise_params.outlier_std                  = declare_parameter<double>("outlier_std", 10.0);
    noise_params.min_detectable_area          = declare_parameter<double>("min_detectable_area", 100.0);
    noise_params.max_detectable_distance      = declare_parameter<double>("max_detectable_distance", 8.0);
    noise_params.detection_probability        = declare_parameter<double>("detection_probability", 0.95);
    noise_params.miss_probability             = declare_parameter<double>("miss_probability", 0.05);
    noise_model_ = std::make_unique<DetectionNoise>(noise_params,
        declare_parameter<int>("noise_seed", 42));
    enemy_color_ = declare_parameter<std::string>("enemy_color", "blue");

    // ═══════════════════════════════════════════════════════
    // ROS2 接口
    // ═══════════════════════════════════════════════════════
    detector_pub_ = create_publisher<auto_aim_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS());
    ground_truth_pub_ = create_publisher<armor_simulation::msg::GroundTruth>(
        "/armor_simulation/ground_truth", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/simulation/marker", 10);

    position_marker_.ns = "position_";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.r = 0.0; position_marker_.color.g = 1.0; position_marker_.color.b = 1.0;

    armor_marker_.ns = "armors_";
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.05; armor_marker_.scale.y = 0.135; armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.g = 0.5; armor_marker_.color.b = 1.0;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    last_t = now();
    timer_ = create_wall_timer(
        1000ms / publish_rate, std::bind(&ArmorSimulation::publishSimulation, this));

    RCLCPP_INFO(get_logger(), "Armor simulation initialized (camera pose from TF)");
    RCLCPP_INFO(get_logger(), "  Camera: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                intr.fx, intr.fy, intr.cx, intr.cy);
    RCLCPP_INFO(get_logger(), "  Noise: sigma(d)=%.1f+%.3f*(d-%.1f)^2, outliers=%s(%d%%@%.0fpx)",
                noise_params.pixel_noise_optimal, noise_params.pixel_noise_curvature,
                noise_params.pixel_noise_optimal_distance,
                noise_params.use_outliers ? "on" : "off",
                static_cast<int>(noise_params.outlier_probability * 100),
                noise_params.outlier_std);
}

void ArmorSimulation::publishSimulation() {
    rclcpp::Time now = this->now();
    double dt = std::min((now - last_t).seconds(), 1.0);
    last_t = now;

    // ── 过程噪声 ──
    double dt_safe = (dt > 1e-9) ? dt : 0.01;
    double ax_noise = std::sqrt(process_noise_xy_ / dt_safe) * accel_noise_dist_(rng_);
    double ay_noise = std::sqrt(process_noise_xy_ / dt_safe) * accel_noise_dist_(rng_);
    double ayaw_noise = std::sqrt(process_noise_yaw_ / dt_safe) * accel_noise_dist_(rng_);
    double ax_eff = x_a + ax_noise, ay_eff = y_a + ay_noise, ayaw_eff = yaw_a + ayaw_noise;

    // ── 运动更新 ──
    x   += x_v * dt + 0.5 * ax_eff * dt * dt;
    y   += y_v * dt + 0.5 * ay_eff * dt * dt;
    yaw += yaw_v * dt + 0.5 * ayaw_eff * dt * dt;
    x_v   += ax_eff * dt;
    y_v   += ay_eff * dt;
    yaw_v += ayaw_eff * dt;

    // ── 从 TF 获取相机外参 (由 gimbal_simulation 节点动态更新) ──
    try {
        auto tf = tf2_buffer_->lookupTransform(
            "odom", "camera_optical_frame", now, rclcpp::Duration::from_seconds(0.1));
        Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                             tf.transform.rotation.y, tf.transform.rotation.z);
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d t(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
        camera_model_.setExtrinsics(R, t);
    } catch (const tf2::TransformException& e) {
        // 降级: 保留上一帧外参
    }

    // ── 装甲板计算 + 相机投影 + 噪声 + PnP ──
    auto_aim_interfaces::msg::Armors detector_msg;
    detector_msg.header.stamp = now;
    detector_msg.header.frame_id = "camera_optical_frame";

    marker_array_.markers.clear();
    armor_marker_.header.stamp = now;
    armor_marker_.header.frame_id = "odom";
    armor_marker_.id = 0;

    static int frame_cnt = 0;
    int n_orient=0, n_detect=0, n_vis=0;
    frame_cnt++;

    for (int i = 0; i < 4; i++) {
        double armor_yaw = yaw + i * M_PI_2;
        double r  = (i % 2 == 0) ? r1 : r2;
        double dz = (i % 2 == 0) ? 0.0 : (z2 - z1);

        Eigen::Vector3d pos(x - cos(armor_yaw) * r,
                            y - sin(armor_yaw) * r,
                            z1 + dz);
        tf2::Quaternion tf_q;
        // 装甲板朝向: pitch=0.26 (15°, 向内倾斜, 与 autoaim 一致)
        tf_q.setRPY(0, 0.26, angles::normalize_angle(armor_yaw));
        Eigen::Quaterniond orient(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());
        double distance = pos.norm();

        double width  = SMALL_ARMOR_WIDTH;
        double height = SMALL_ARMOR_HEIGHT;
        auto corners = computeArmorCorners(pos, orient, width, height);
        double area = camera_model_.computeArea(corners);

        // 朝向过滤
        Eigen::Vector3d armor_normal_odom(cos(armor_yaw), sin(armor_yaw), 0);
        if (armor_normal_odom.dot(pos) >= 0) continue;
        n_orient++;

        if (!noise_model_->shouldDetect(distance, area)) continue;
        n_detect++;

        std::array<Eigen::Vector2d, 4> pixels;
        bool all_in_image = true;
        for (int j = 0; j < 4; j++) {
            pixels[j] = camera_model_.project3Dto2D(corners[j]);
            if (!camera_model_.isInImage(pixels[j], 10.0)) { all_in_image = false; break; }
        }
        if (!all_in_image) continue;
        n_vis++;

        for (int j = 0; j < 4; j++)
            pixels[j] = noise_model_->addPixelNoise(pixels[j], distance);

        auto pnp = camera_model_.estimatePose(pixels, width, height);

        auto_aim_interfaces::msg::Armor armor;
        armor.number = "4";
        armor.type = "small";
        armor.color = "blue";
        armor.area = area;
        armor.pose.position.x = pnp.position_odom.x();
        armor.pose.position.y = pnp.position_odom.y();
        armor.pose.position.z = pnp.position_odom.z();
        armor.pose.orientation.x = pnp.orientation_odom.x();
        armor.pose.orientation.y = pnp.orientation_odom.y();
        armor.pose.orientation.z = pnp.orientation_odom.z();
        armor.pose.orientation.w = pnp.orientation_odom.w();

        Eigen::Vector3d rpy = pnp.orientation_odom.toRotationMatrix().eulerAngles(2, 1, 0);
        armor.yaw = static_cast<float>(rpy[0]);
        armor.optimized_yaw = armor.yaw;
        for (int j = 0; j < 4; j++) {
            armor.detected_points[j].x = pixels[j].x();
            armor.detected_points[j].y = pixels[j].y();
        }
        detector_msg.armors.push_back(armor);

        armor_marker_.id++;
        armor_marker_.pose.position.x = pos.x();
        armor_marker_.pose.position.y = pos.y();
        armor_marker_.pose.position.z = pos.z();
        tf_q.setRPY(0, 0.26, angles::normalize_angle(armor_yaw));
        armor_marker_.pose.orientation = tf2::toMsg(tf_q);
        marker_array_.markers.push_back(armor_marker_);
    }

    if (frame_cnt % 30 == 1) {
        RCLCPP_INFO(get_logger(),
            "frame %d: orient=%d detect=%d vis=%d published=%zu pos=(%.1f,%.1f) yaw=%.1f",
            frame_cnt, n_orient, n_detect, n_vis, detector_msg.armors.size(), x, y, yaw);
    }

    if (!detector_msg.armors.empty())
        detector_pub_->publish(detector_msg);

    // ── 真值 ──
    armor_simulation::msg::GroundTruth truth;
    truth.header.stamp = now;
    truth.header.frame_id = "odom";
    truth.x    = x;     truth.y    = y;     truth.z    = z1;
    truth.yaw  = yaw;
    truth.vx   = x_v;   truth.vy   = y_v;   truth.vz   = 0.0;   truth.vyaw = yaw_v;
    truth.r1   = r1;    truth.r2   = r2;    truth.dz   = z2 - z1;
    ground_truth_pub_->publish(truth);

    // ── 可视化 ──
    position_marker_.header = detector_msg.header;
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = x;
    position_marker_.pose.position.y = y;
    position_marker_.pose.position.z = z1;
    marker_array_.markers.emplace_back(position_marker_);
    marker_pub_->publish(marker_array_);

    // ── 运动限幅 ──
    constexpr double eps = 1e-6;
    if (std::abs(x_v) > linear_speed_limit && linear_speed_limit != -1)
        x_v = x_v / (std::abs(x_v) + eps) * linear_speed_limit;
    if (std::abs(y_v) > linear_speed_limit && linear_speed_limit != -1)
        y_v = y_v / (std::abs(y_v) + eps) * linear_speed_limit;
    if (std::abs(yaw_v) > angle_speed_limit && angle_speed_limit != -1)
        yaw_v = yaw_v / (std::abs(yaw_v) + eps) * angle_speed_limit;
    if (std::abs(x) > linear_limit && linear_limit != -1)
        x_a = -x / (std::abs(x) + eps) * linear_acc;
    if (std::abs(y) > linear_limit && linear_limit != -1)
        y_a = -y / (std::abs(y) + eps) * linear_acc;
    if (std::abs(yaw) > angle_limit && angle_limit != -1)
        yaw_a = -yaw / (std::abs(yaw) + eps) * angle_acc;
}

}  // namespace armor_sim

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<armor_sim::ArmorSimulation>());
    rclcpp::shutdown();
    return 0;
}

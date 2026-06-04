#include "filter_test/filter_test.hpp"

ArmorTest::ArmorTest(const rclcpp::NodeOptions & options)
: Node("filter_test", options) {
    bool use_ekf = declare_parameter<bool>("use_ekf", true);
    bool use_cv_model = declare_parameter<bool>("use_cv_model", true);

    RCLCPP_INFO(get_logger(), "数据源: Detector, 滤波器: %s, 模型: %s",
        use_ekf ? "EKF" : "UKF", use_cv_model ? "CV" : "Singer");

    // 初始化滤波器
    armor_filter_ = std::make_unique<ArmorFilter>(use_ekf, use_cv_model,
        declare_parameter<double>("s2qxy_cv", 0.05), declare_parameter<double>("s2qz_cv", 0.05),
        declare_parameter<double>("s2qyaw_cv", 0.05), declare_parameter<double>("s2qr_cv", 10.0), declare_parameter<double>("s2qdz_cv", 0.01),
        declare_parameter<double>("s2qxy_singer", 0.05), declare_parameter<double>("s2qz_singer", 0.05),
        declare_parameter<double>("s2qyaw_singer", 0.05), declare_parameter<double>("s2qr_singer", 10.0), declare_parameter<double>("s2qdz_singer", 0.01),
        declare_parameter<double>("tau_singer", 1.0), declare_parameter<double>("ukf_alpha", 0.001),
        declare_parameter<double>("ukf_beta", 2.0), declare_parameter<double>("ukf_kappa", 0.0));

    // 发布者
    result_pub_ = create_publisher<filter_test::msg::Result>("/track_result", rclcpp::SensorDataQoS());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/track_result/marker", 10);

    // 订阅 /detector/armors
    detector_sub_ = create_subscription<auto_aim_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS(),
        [this](auto_aim_interfaces::msg::Armors::SharedPtr msg) { detectorCallback(msg); });

    position_marker_.ns = "position"; position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0; position_marker_.color.g = 1.0;

    armor_marker_.ns = "armors"; armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.005; armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0; armor_marker_.color.r = 1.0;

    // R参数
    r_pose_det_ = declare_parameter<double>("r_pose_det", 0.01);
    r_distance_det_ = declare_parameter<double>("r_distance_det", 0.01);
    r_yaw_det_ = declare_parameter<double>("r_yaw_det", 0.01);
}

void ArmorTest::detectorCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr) {
    if (armors_ptr->armors.empty()) return;

    // 检测器模式：R参数更大（考虑PnP误差）
    armor_filter_->set_r_for_detector(r_pose_det_, r_distance_det_, r_yaw_det_);

    if (!init) {
        auto copy = std::make_shared<auto_aim_interfaces::msg::Armors>(*armors_ptr);
        armor_filter_->init(copy);
        init = true;
        RCLCPP_INFO(get_logger(), "滤波器已初始化");
        return;
    }

    auto status = armor_filter_->update(armors_ptr);
    auto & h = armors_ptr->header;

    // 发布 Result
    filter_test::msg::Result result;
    result.header = h;
    result.position.x = status[0]; result.position.y = status[1]; result.position.z = status[2];
    result.velocity.x = status[3]; result.velocity.y = status[4]; result.velocity.z = status[5];
    result.yaw = status[6]; result.v_yaw = status[7];
    result.radius_1 = status[8]; result.radius_2 = status[9]; result.dz = status[10];
    result_pub_->publish(result);

    // Detector模式可视化
    visualization_msgs::msg::MarkerArray marker_array;
    double xc = result.position.x, yc = result.position.y, za = result.position.z;
    double dz = result.dz;

    position_marker_.header = h;
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;
    marker_array.markers.emplace_back(position_marker_);

    // DELETE 旧 marker (armors_pred + armors_obs)
    for (auto* ns : {"armors_pred", "armors_obs"}) {
        for (int i = 0; i < 4; i++) {
            auto del = armor_marker_;
            del.ns = ns;
            del.id = i;
            del.action = visualization_msgs::msg::Marker::DELETE;
            del.lifetime = rclcpp::Duration::from_seconds(0);
            marker_array.markers.emplace_back(del);
        }
    }

    armor_marker_.header = h;
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
    armor_marker_.scale.y = 0.135;

    // 追踪器预测的 4 块装甲板 (蓝色)
    double yaw = result.yaw;
    double r1 = result.radius_1;
    double r2 = result.radius_2;
    double dz_val = result.dz;
    armor_marker_.ns = "armors_pred";
    armor_marker_.color.r = 0.0;
    armor_marker_.color.g = 0.5;
    armor_marker_.color.b = 1.0;
    for (int i = 0; i < 4; i++) {
        double ay = yaw + i * M_PI_2;
        double r = (i % 2 == 0) ? r1 : r2;
        double dz_i = (i % 2 == 0) ? 0.0 : dz_val;
        armor_marker_.id = i;
        armor_marker_.pose.position.x = xc - r * std::cos(ay);
        armor_marker_.pose.position.y = yc - r * std::sin(ay);
        armor_marker_.pose.position.z = za + dz_i;
        tf2::Quaternion q; q.setRPY(0, 15.0 * M_PI / 180.0, ay);
        armor_marker_.pose.orientation = tf2::toMsg(q);
        marker_array.markers.emplace_back(armor_marker_);
    }

    // 检测器观测到的装甲板 (红色)
    armor_marker_.ns = "armors_obs";
    armor_marker_.color.r = 1.0;
    armor_marker_.color.g = 0.0;
    armor_marker_.color.b = 0.0;
    for (size_t i = 0; i < armors_ptr->armors.size(); i++) {
        const auto& observed = armors_ptr->armors[i];
        armor_marker_.id = i;
        armor_marker_.pose.position.x = observed.pose.position.x;
        armor_marker_.pose.position.y = observed.pose.position.y;
        armor_marker_.pose.position.z = observed.pose.position.z;
        armor_marker_.pose.orientation =
            tf2::toMsg(filter_test::observedArmorMarkerQuaternion(observed.yaw));
        marker_array.markers.emplace_back(armor_marker_);
    }

    marker_pub_->publish(marker_array);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorTest>());
    rclcpp::shutdown();
    return 0;
}

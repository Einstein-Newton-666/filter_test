#include "filter_test/filter_test.hpp"

ArmorTest::ArmorTest(const rclcpp::NodeOptions & options)
: Node("filter_test", options) {
    // 数据源参数
    bool use_sim = declare_parameter<bool>("use_simulation", true);
    bool use_ekf = declare_parameter<bool>("use_ekf", true);
    bool use_cv_model = declare_parameter<bool>("use_cv_model", true);

    RCLCPP_INFO(get_logger(), "数据源: %s, 滤波器: %s, 模型: %s",
        use_sim ? "模拟器" : "Detector", use_ekf ? "EKF" : "UKF", use_cv_model ? "CV" : "Singer");

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

    // 根据数据源只订阅一路，避免模拟器和检测器同时更新滤波器并发布同一个marker话题
    if (use_sim) {
        simulation_sub_ = create_subscription<filter_test::msg::Simulation>(
            "/armor_simulation", rclcpp::SensorDataQoS(),
            [this](filter_test::msg::Simulation::SharedPtr msg) { simCallback(msg); });
    } else {
        detector_sub_ = create_subscription<auto_aim_interfaces::msg::Armors>(
            "/detector/armors", rclcpp::SensorDataQoS(),
            [this](auto_aim_interfaces::msg::Armors::SharedPtr msg) { detectorCallback(msg); });
    }

    position_marker_.ns = "position"; position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0; position_marker_.color.g = 1.0;
    
    linear_v_marker_.ns = "linear_v"; linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.scale.x = 0.03; linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.color.a = 1.0; linear_v_marker_.color.r = 1.0; linear_v_marker_.color.g = 1.0;
    
    angular_v_marker_.ns = "angular_v"; angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    angular_v_marker_.scale.x = 0.03; angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.color.a = 1.0; angular_v_marker_.color.b = 1.0; angular_v_marker_.color.g = 1.0;
    
    armor_marker_.ns = "armors"; armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.03; armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0; armor_marker_.color.r = 1.0;

    // R参数（统一声明一次）
    r_pose_sim_ = declare_parameter<double>("r_pose_sim", 0.001);
    r_distance_sim_ = declare_parameter<double>("r_distance_sim", 0.001);
    r_yaw_sim_ = declare_parameter<double>("r_yaw_sim", 0.001);
    r_pose_det_ = declare_parameter<double>("r_pose_det", 0.01);
    r_distance_det_ = declare_parameter<double>("r_distance_det", 0.01);
    r_yaw_det_ = declare_parameter<double>("r_yaw_det", 0.01);
}

void ArmorTest::simCallback(const filter_test::msg::Simulation::SharedPtr sim_ptr) {
    // 仿真模式：R参数更小（信任仿真数据）
    armor_filter_->set_r_for_simulation(r_pose_sim_, r_distance_sim_, r_yaw_sim_);
    processArmors(std::make_shared<auto_aim_interfaces::msg::Armors>(sim_ptr->armors));

    // 差分计算（从 ArmorSimulation 读取真实值）
    if (!last_sim_pos_) {
        last_sim_pos_ = std::make_shared<geometry_msgs::msg::Point>(sim_ptr->position);
    }
    auto status = armor_filter_->get_last_result();
    result_diff_.position_x_diff = status[0] - sim_ptr->position.x;
    result_diff_.position_y_diff = status[1] - sim_ptr->position.y;
    result_diff_.position_z1_diff = status[2] - sim_ptr->position.z;
    result_diff_.position_z2_diff = status[10];
    result_diff_.position_yaw_diff = angles::shortest_angular_distance(sim_ptr->yaw, status[6]);
    result_diff_.velocity_x_diff = status[3] - sim_ptr->velocity.x;
    result_diff_.velocity_y_diff = status[4] - sim_ptr->velocity.y;
    result_diff_.velocity_yaw_diff = angles::shortest_angular_distance(sim_ptr->v_yaw, status[7]);
    result_diff_.r1_diff = status[8] - sim_ptr->radius_1;
    result_diff_.r2_diff = status[9] - sim_ptr->radius_2;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "pos_err: [%.4f, %.4f, %.4f] yaw_err: %.4f vel_err: [%.4f, %.4f, %.4f] vyaw_err: %.4f",
        result_diff_.position_x_diff, result_diff_.position_y_diff, result_diff_.position_z1_diff,
        result_diff_.position_yaw_diff,
        result_diff_.velocity_x_diff, result_diff_.velocity_y_diff, result_diff_.velocity_yaw_diff,
        result_diff_.velocity_yaw_diff);
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

    // Detector模式可视化：只显示观测到的装甲板
    visualization_msgs::msg::MarkerArray marker_array;
    double xc = result.position.x, yc = result.position.y, za = result.position.z;
    double dz = result.dz;

    position_marker_.header = h;
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;
    marker_array.markers.emplace_back(position_marker_);

    armor_marker_.header = h;
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.scale.y = 0.135;

    // 只显示检测到的装甲板
    for (size_t i = 0; i < armors_ptr->armors.size(); i++) {
        const auto & observed = armors_ptr->armors[i];
        armor_marker_.id = i;
        armor_marker_.pose.position.x = observed.pose.position.x;
        armor_marker_.pose.position.y = observed.pose.position.y;
        armor_marker_.pose.position.z = observed.pose.position.z;
        double t = atan2(observed.pose.position.y - yc, observed.pose.position.x - xc);
        tf2::Quaternion q; q.setRPY(0, 0.26, t);
        armor_marker_.pose.orientation = tf2::toMsg(q);
        marker_array.markers.emplace_back(armor_marker_);
    }

    marker_pub_->publish(marker_array);
}

void ArmorTest::processArmors(const auto_aim_interfaces::msg::Armors::SharedPtr & armors) {
    if (!init) {
        auto copy = std::make_shared<auto_aim_interfaces::msg::Armors>(*armors);
        armor_filter_->init(copy);
        init = true;
        RCLCPP_INFO(get_logger(), "滤波器已初始化");
        return;
    }

    auto status = armor_filter_->update(armors);
    auto & h = armors->header;

    // 发布 Result
    filter_test::msg::Result result;
    result.header = h;
    result.position.x = status[0]; result.position.y = status[1]; result.position.z = status[2];
    result.velocity.x = status[3]; result.velocity.y = status[4]; result.velocity.z = status[5];
    result.yaw = status[6]; result.v_yaw = status[7];
    result.radius_1 = status[8]; result.radius_2 = status[9]; result.dz = status[10];
    result_pub_->publish(result);

    // 可视化
    visualization_msgs::msg::MarkerArray marker_array;
    double yaw = result.yaw, r1 = result.radius_1, r2 = result.radius_2;
    double xc = result.position.x, yc = result.position.y, za = result.position.z;
    double dz = result.dz;

    position_marker_.header = h;
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;

    linear_v_marker_.header = angular_v_marker_.header = h;
    linear_v_marker_.action = angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    angular_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point p = position_marker_.pose.position;
    p.x += result.velocity.x; p.y += result.velocity.y; p.z += result.velocity.z;
    linear_v_marker_.points.emplace_back(p);
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    p = position_marker_.pose.position; p.z += result.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(p);

    armor_marker_.header = h;
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.scale.y = 0.135;
    bool is_pair = true;
    for (size_t j = 0; j < 4; j++) {
        double t = yaw + j * M_PI / 2;
        double r = is_pair ? r1 : r2;
        armor_marker_.id = j;
        armor_marker_.pose.position.x = xc - r * cos(t);
        armor_marker_.pose.position.y = yc - r * sin(t);
        armor_marker_.pose.position.z = za + (is_pair ? 0 : dz);
        tf2::Quaternion q; q.setRPY(0, 0.26, t);
        armor_marker_.pose.orientation = tf2::toMsg(q);
        marker_array.markers.emplace_back(armor_marker_);
        is_pair = !is_pair;
    }
    marker_array.markers.emplace_back(position_marker_);
    marker_array.markers.emplace_back(linear_v_marker_);
    marker_array.markers.emplace_back(angular_v_marker_);
    marker_pub_->publish(marker_array);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorTest>());
    rclcpp::shutdown();
    return 0;
}
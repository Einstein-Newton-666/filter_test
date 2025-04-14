#include "armor_tracker/tracker_node.hpp"

// STD
#include <memory>
#include <vector>

namespace rm_auto_aim {
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions &options)
        : Node("armor_tracker", options) {
    RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

    // Maximum allowable armor distance in the XOY plane
    max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

    debug_ = this->declare_parameter("debug", true);

    // Tracker
    tracker_ = std::make_unique<Tracker>();
    tracker_->armor_model.tracking_thres = this->declare_parameter("ArmorModel.tracking_thres", 5);
    tracker_->enemy_model.tracking_thres = this->declare_parameter("EnemyModel.tracking_thres", 5);

    s2qx_armor = declare_parameter("ArmorModel.s2qx_", 10.0);
    s2qy_armor = declare_parameter("ArmorModel.s2qy_", 10.0);
    s2qz_armor = declare_parameter("ArmorModel.s2qz_", 10.0);
    s2qyaw_armor = declare_parameter("ArmorModel.s2qyaw_", 10.0);

    r_x_armor = declare_parameter("ArmorModel.r_x_", 0.001);
    r_y_armor = declare_parameter("ArmorModel.r_y_", 0.001);
    r_z_armor = declare_parameter("ArmorModel.r_z_", 0.001);
    r_yaw_armor = declare_parameter("ArmorModel.r_yaw_", 0.001);

    position_diff_thres_ = declare_parameter("ArmorModel.position_diff_thres", 0.05);

    s2qxy_max_enemy = declare_parameter("EnemyModel.s2qxy_max_", 10.0);
    s2qxy_min_enemy = declare_parameter("EnemyModel.s2qxy_min_", 0.05);
    s2qz_enemy = declare_parameter("EnemyModel.s2qz_", 10.0);
    s2qyaw_max_enemy = declare_parameter("EnemyModel.s2qyaw_max_", 10.0);
    s2qyaw_min_enemy = declare_parameter("EnemyModel.s2qyaw_min_", 0.05);
    s2qr_enemy = declare_parameter("EnemyModel.s2qr_", 10.0);

    r_pose_enemy = declare_parameter("EnemyModel.r_pose_", 0.001);
    r_distance_enemy = declare_parameter("EnemyModel.r_distance_", 0.001);
    r_yaw_enemy = declare_parameter("EnemyModel.r_yaw_", 0.001);

    lost_time_thres_armor_ = declare_parameter("ArmorModel.lost_time_thres", 0.3);
    lost_time_thres_enemy_ = declare_parameter("EnemyModel.lost_time_thres", 0.3);

    // Subscriber with tf2 message_filter
    // tf2 relevant
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    // subscriber and filter
    armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
    target_frame_ = this->declare_parameter("target_frame", "odom");
    tf2_filter_ = std::make_shared<tf2_filter>(
            armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
            this->get_node_clock_interface(), std::chrono::duration<int>(1));
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);

    // Measurement publisher (for debug usage)
    info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

    // predict publisher (for debug usage)
    predict_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerPredict>("/tracker/predict", 10);

    // Publisher
    target_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerTarget>("/tracker/target", rclcpp::SensorDataQoS());

    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;//球体
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;
    linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;//箭头
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.color.a = 1.0;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;
    angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;//箭头
    angular_v_marker_.ns = "angular_v";
    angular_v_marker_.scale.x = 0.03;
    angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.color.a = 1.0;
    angular_v_marker_.color.b = 1.0;
    angular_v_marker_.color.g = 1.0;
    armor_marker_.ns = "armors";
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;//立方体
    armor_marker_.scale.x = 0.03;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.g = 1.0;
    enemy_marker_.ns = "enemy";
    enemy_marker_.type = visualization_msgs::msg::Marker::CUBE;//立方体
    armor_marker_.scale.x = 0.03;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.r = 1.0;
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);
}

void ArmorTrackerNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg) {
    update_params();
    // Tranform armor position from image frame to world coordinate
    for (auto &armor: armors_msg->armors) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = armors_msg->header;
        ps.pose = armor.pose;
        try {
            armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
        } catch (const tf2::ExtrapolationException &ex) {
            RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
            return;
        }
    }

    // Filter abnormal armors
    armors_msg->armors.erase(
            std::remove_if(
                    armors_msg->armors.begin(), armors_msg->armors.end(),
                    [this](const auto_aim_interfaces::msg::Armor &armor) {
                        return abs(armor.pose.position.z) > 1.2 ||
                        Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                        max_armor_distance_;
                    }),
            armors_msg->armors.end());

    // Init message
    auto_aim_interfaces::msg::TrackerInfo tracker_info;
    auto_aim_interfaces::msg::TrackerPredict tracker_predict;
    auto_aim_interfaces::msg::TrackerTarget tracker_target;

    rclcpp::Time time = armors_msg->header.stamp;
    tracker_info.header.stamp = tracker_predict.header.stamp = tracker_target.header.stamp= time;
    tracker_info.header.frame_id = tracker_predict.header.frame_id = tracker_target.header.frame_id =
            target_frame_;

    dt_ = (time - last_time_).seconds();
    last_time_ = time;
    tracker_->dt = dt_;
    tracker_->armor_model.lost_thres = static_cast<int>(lost_time_thres_armor_ / dt_);
    tracker_->enemy_model.lost_thres = static_cast<int>(lost_time_thres_enemy_ / dt_);
    if (tracker_->enemy_model.tracker_state == LOST) {// TODO:优化模型初始化条件
        tracker_->findTrackerArmor(armors_msg);
        tracker_->initArmorModel();
        tracker_->initEnemyModel();
    } else {
        tracker_->updateTrackerArmor(armors_msg);
        tracker_->updateArmorModel();
        tracker_->updateEnemyModel();
    }
    int index = 0;
    for(const auto& armor : tracker_->tracked_armors){
        if(index > 1){ break;} //armors是定长数组，避免越界访问
        auto_aim_interfaces::msg::ArmorInfo armor_info;
        armor_info.position.x = armor.x;
        armor_info.position.y = armor.y;
        armor_info.position.z = armor.z;
        armor_info.orientation_yaw = armor.orientation_yaw;
        armor_info.yaw = armor.yaw;
        armor_info.pitch = armor.pitch;
        armor_info.distance = armor.distance;
        tracker_info.armors[index++] = armor_info;
    }
    index = 0;
    for(const auto& filter : tracker_->armor_model.filters){
        if(index > 1){ break;} //armors是定长数组，避免越界访问
        auto_aim_interfaces::msg::ArmorInfo armor_info;
        armor_info.position.x = filter.second.pri_estimation[0];
        armor_info.position.y = filter.second.pri_estimation[2];
        armor_info.position.z = filter.second.pri_estimation[4];
        armor_info.orientation_yaw = filter.second.pri_estimation[6];
        armor_info.velocity.x = filter.second.pri_estimation[1];
        armor_info.velocity.y = filter.second.pri_estimation[3];
        armor_info.velocity.z = filter.second.pri_estimation[5];
        tracker_predict.armors[index++] = armor_info;
        armor_info.position.x = filter.second.post_estimation[0];
        armor_info.position.y = filter.second.post_estimation[2];
        armor_info.position.z = filter.second.post_estimation[4];
        armor_info.orientation_yaw = filter.second.post_estimation[6];
        armor_info.velocity.x = filter.second.post_estimation[1];
        armor_info.velocity.y = filter.second.post_estimation[3];
        armor_info.velocity.z = filter.second.post_estimation[5];
        armor_info.tracking = (filter.second.tracker_state != LOST) ? true : false;// 只要不是丢失状态，就认为tracking为true
        tracker_target.armors[index++] = armor_info;
    }

    tracker_predict.enemy.position.x = tracker_->enemy_model.pri_estimation[0];
    tracker_predict.enemy.position.y = tracker_->enemy_model.pri_estimation[2];
    tracker_predict.enemy.position.z = tracker_->enemy_model.pri_estimation[4];
    tracker_predict.enemy.orientation_yaw = tracker_->enemy_model.pri_estimation[6];
    tracker_predict.enemy.velocity.x = tracker_->enemy_model.pri_estimation[1];
    tracker_predict.enemy.velocity.y = tracker_->enemy_model.pri_estimation[3];
    tracker_predict.enemy.radius_1 = tracker_->enemy_model.pri_estimation[8];
    tracker_predict.enemy.radius_2 = tracker_->enemy_model.pri_estimation[9];
    tracker_predict.enemy.dz = tracker_->enemy_model.pri_estimation[5] - tracker_->enemy_model.pri_estimation[4];

    tracker_target.id = tracker_->trackered_enemy_id;
    tracker_target.armors_num = tracker_->enemy_armor_num;
    tracker_target.enemy.position.x = tracker_->enemy_model.post_estimation[0];
    tracker_target.enemy.position.y = tracker_->enemy_model.post_estimation[2];
    tracker_target.enemy.position.z = tracker_->enemy_model.post_estimation[4];
    tracker_target.enemy.orientation_yaw = tracker_->enemy_model.post_estimation[6];
    tracker_target.enemy.velocity.x = tracker_->enemy_model.post_estimation[1];
    tracker_target.enemy.velocity.y = tracker_->enemy_model.post_estimation[3];
    tracker_target.enemy.radius_1 = tracker_->enemy_model.post_estimation[8];
    tracker_target.enemy.radius_2 = tracker_->enemy_model.post_estimation[9];
    tracker_target.enemy.dz = tracker_->enemy_model.post_estimation[5] - tracker_->enemy_model.post_estimation[4];
    tracker_target.enemy.tracking = (tracker_->enemy_model.tracker_state != LOST) ? true : false;// 只要不是丢失状态，就认为tracking为true

    info_pub_->publish(tracker_info);
    predict_pub_->publish(tracker_predict);
    target_pub_->publish(tracker_target);
    if (debug_) {
        publishMarkers(tracker_target);
    }
}

// TODO:两个模型的可视化
void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::TrackerTarget &tracker_target) {
    position_marker_.header = tracker_target.header;
    linear_v_marker_.header = tracker_target.header;
    angular_v_marker_.header = tracker_target.header;
    armor_marker_.header = tracker_target.header;
    enemy_marker_.header = tracker_target.header;

    visualization_msgs::msg::MarkerArray marker_array;
    // 绘制整车观测模型
    if (tracker_target.enemy.tracking) {
        double orientation_yaw = tracker_target.enemy.orientation_yaw, r1 = tracker_target.enemy.radius_1, r2 = tracker_target.enemy.radius_2;
        double xc = tracker_target.enemy.position.x, yc = tracker_target.enemy.position.y, za = tracker_target.enemy.position.z;
        double vx = tracker_target.enemy.velocity.x, vy = tracker_target.enemy.velocity.y, vz = tracker_target.enemy.velocity.z;
        double dz = tracker_target.enemy.dz;

        position_marker_.action = visualization_msgs::msg::Marker::ADD;
        position_marker_.pose.position.x = xc;
        position_marker_.pose.position.y = yc;
        position_marker_.pose.position.z = za + dz / 2;

        linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        linear_v_marker_.points.clear();
        linear_v_marker_.points.emplace_back(position_marker_.pose.position);
        geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
        arrow_end.x += vx / 2;
        arrow_end.y += vy / 2;
        arrow_end.z += vz / 2;
        linear_v_marker_.points.emplace_back(arrow_end);

        angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        angular_v_marker_.points.clear();
        angular_v_marker_.points.emplace_back(position_marker_.pose.position);
        arrow_end = position_marker_.pose.position;
        arrow_end.z += tracker_target.enemy.v_yaw / M_PI;
        angular_v_marker_.points.emplace_back(arrow_end);

        enemy_marker_.action = visualization_msgs::msg::Marker::ADD;
        enemy_marker_.scale.y = tracker_target.id == "1" ? 0.23 : 0.135;
        bool is_current_pair = true;
        size_t a_n = tracker_target.armors_num;
        geometry_msgs::msg::Point p_a;
        double r = 0;
        for (size_t i = 0; i < a_n; i++) {
            double tmp_yaw = orientation_yaw + i * (2 * M_PI / a_n);
            // Only 4 armors has 2 radius and height
            if (a_n == 4) {
                r = is_current_pair ? r1 : r2;
                p_a.z = za + (is_current_pair ? 0 : dz);
                is_current_pair = !is_current_pair;

            } else {
                r = r1;
                p_a.z = za;
            }
            p_a.x = xc - r * cos(tmp_yaw);
            p_a.y = yc - r * sin(tmp_yaw);

            enemy_marker_.id = i;
            enemy_marker_.pose.position = p_a;
            tf2::Quaternion q;
            q.setRPY(0, tracker_target.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
            enemy_marker_.pose.orientation = tf2::toMsg(q);
            marker_array.markers.emplace_back(enemy_marker_);
        }
    } else {
        position_marker_.action = visualization_msgs::msg::Marker::DELETE;
        linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
        angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;

        enemy_marker_.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.emplace_back(enemy_marker_);
    }

    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.scale.y = tracker_target.id == "1" ? 0.23 : 0.135;
    geometry_msgs::msg::Point a_p;
    int index = 0;
    for(const auto& armor : tracker_target.armors){
        double orientation_yaw = armor.orientation_yaw;
        a_p.x = armor.position.x;
        a_p.y = armor.position.y;
        a_p.z = armor.position.z;
        armor_marker_.id = index++;
        armor_marker_.pose.position = a_p;
        tf2::Quaternion q;
        q.setRPY(0, tracker_target.id == "outpost" ? -0.26 : 0.26, orientation_yaw);
        armor_marker_.pose.orientation = tf2::toMsg(q);
        marker_array.markers.emplace_back(armor_marker_);
    }
    marker_array.markers.emplace_back(position_marker_);
    marker_array.markers.emplace_back(linear_v_marker_);
    marker_array.markers.emplace_back(angular_v_marker_);
    marker_pub_->publish(marker_array);
}

void ArmorTrackerNode::update_params() {
    // Update ArmorModel parameters
    s2qx_armor = this->get_parameter("ArmorModel.s2qx_").as_double();
    s2qy_armor = this->get_parameter("ArmorModel.s2qy_").as_double();
    s2qz_armor = this->get_parameter("ArmorModel.s2qz_").as_double();
    s2qyaw_armor = this->get_parameter("ArmorModel.s2qyaw_").as_double();
    r_x_armor = this->get_parameter("ArmorModel.r_x_").as_double();
    r_y_armor = this->get_parameter("ArmorModel.r_y_").as_double();
    r_z_armor = this->get_parameter("ArmorModel.r_z_").as_double();
    r_yaw_armor = this->get_parameter("ArmorModel.r_yaw_").as_double();
    lost_time_thres_armor_ = this->get_parameter("ArmorModel.lost_time_thres").as_double();
    position_diff_thres_ = this->get_parameter("ArmorModel.position_diff_thres").as_double();

    // Update EnemyModel parameters
    s2qxy_max_enemy = this->get_parameter("EnemyModel.s2qxy_max_").as_double();
    s2qxy_min_enemy = this->get_parameter("EnemyModel.s2qxy_min_").as_double();
    s2qz_enemy = this->get_parameter("EnemyModel.s2qz_").as_double();
    s2qyaw_max_enemy = this->get_parameter("EnemyModel.s2qyaw_max_").as_double();
    s2qyaw_min_enemy = this->get_parameter("EnemyModel.s2qyaw_min_").as_double();
    s2qr_enemy = this->get_parameter("EnemyModel.s2qr_").as_double();
    r_pose_enemy = this->get_parameter("EnemyModel.r_pose_").as_double();
    r_distance_enemy = this->get_parameter("EnemyModel.r_distance_").as_double();
    r_yaw_enemy = this->get_parameter("EnemyModel.r_yaw_").as_double();
    lost_time_thres_enemy_ = this->get_parameter("EnemyModel.lost_time_thres").as_double();
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)

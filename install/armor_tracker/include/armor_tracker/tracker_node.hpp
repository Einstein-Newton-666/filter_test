// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/armor_info.hpp"
#include "auto_aim_interfaces/msg/enemy_info.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "auto_aim_interfaces/msg/tracker_predict.hpp"
#include "auto_aim_interfaces/msg/tracker_target.hpp"


namespace rm_auto_aim {
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;

class ArmorTrackerNode : public rclcpp::Node {
public:
    explicit ArmorTrackerNode(const rclcpp::NodeOptions &options);

private:
    void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

    void publishMarkers(const auto_aim_interfaces::msg::TrackerTarget &tracker_target);

    void update_params();

    // Reset tracker service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

    // Subscriber with tf2 message_filter
    std::string target_frame_;
    std::shared_ptr <tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr <tf2_ros::TransformListener> tf2_listener_;
    std::shared_ptr <tf2_filter> tf2_filter_;
    message_filters::Subscriber <auto_aim_interfaces::msg::Armors> armors_sub_;

    // Tracker publisher
    rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::TrackerPredict>::SharedPtr predict_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::TrackerTarget>::SharedPtr target_pub_;

    // Visualization marker publisher
    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker linear_v_marker_;
    visualization_msgs::msg::Marker angular_v_marker_;
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker enemy_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Maximum allowable armor distance in the XOY plane
    double max_armor_distance_;

    // The time when the last message was received
    rclcpp::Time last_time_;
    double dt_;

    // Armor tracker
    // Armor model
    double s2qx_armor, s2qy_armor, s2qz_armor, s2qyaw_armor;
    double r_x_armor, r_y_armor, r_z_armor, r_yaw_armor;
    double position_diff_thres_, lost_time_thres_armor_;
    // Enemy model
    double s2qxy_max_enemy, s2qxy_min_enemy, s2qz_enemy, s2qyaw_max_enemy, s2qyaw_min_enemy, s2qr_enemy;
    double r_pose_enemy, r_distance_enemy, r_yaw_enemy;
    double lost_time_thres_enemy_;

    std::unique_ptr <Tracker> tracker_;
    bool debug_;


};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

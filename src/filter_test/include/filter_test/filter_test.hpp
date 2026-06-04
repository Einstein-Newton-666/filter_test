#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "filter_test/filter.hpp"
#include "filter_test/visualization_marker_utils.hpp"
#include <filter_test/msg/result.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>


class ArmorTest: public rclcpp::Node
{

public:
    ArmorTest(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void detectorCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

    // 订阅
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr detector_sub_;

    // 发布
    rclcpp::Publisher<filter_test::msg::Result>::SharedPtr result_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::unique_ptr<ArmorFilter> armor_filter_;
    bool init = false;

    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker armor_marker_;

    // R参数
    double r_pose_det_, r_distance_det_, r_yaw_det_;
};

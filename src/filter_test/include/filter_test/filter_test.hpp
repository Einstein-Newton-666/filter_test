#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "filter_test/filter.hpp"
#include <filter_test/msg/simulation.hpp>
#include <filter_test/msg/result.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>
#include <geometry_msgs/msg/point.hpp>


class ArmorTest: public rclcpp::Node
{

public:
    ArmorTest(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // 统一处理函数
    void processArmors(const auto_aim_interfaces::msg::Armors::SharedPtr & armors);

    // 两个数据源的回调
    void simCallback(const filter_test::msg::Simulation::SharedPtr sim_ptr);
    void detectorCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

    // 订阅
    rclcpp::Subscription<filter_test::msg::Simulation>::SharedPtr simulation_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr detector_sub_;
    
    // 发布
    rclcpp::Publisher<filter_test::msg::Result>::SharedPtr result_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::unique_ptr<ArmorFilter> armor_filter_;
    bool init = false; 

    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker linear_v_marker_;
    visualization_msgs::msg::Marker angular_v_marker_;
    visualization_msgs::msg::Marker armor_marker_;

    // R参数（统一在构造函数中声明一次）
    double r_pose_sim_, r_distance_sim_, r_yaw_sim_;
    double r_pose_det_, r_distance_det_, r_yaw_det_;

    // 用于误差计算的成员
    filter_test::msg::Result result_diff_;
    geometry_msgs::msg::Point::SharedPtr last_sim_pos_;
};



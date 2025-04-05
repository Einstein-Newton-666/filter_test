#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>

#include "filter_test/filter.hpp"
#include <filter_test/msg/simulation.hpp>
#include <filter_test/msg/result.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>


class ArmorTest: public rclcpp::Node
{

public:
    ArmorTest(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void armorsCallback(const filter_test::msg::Simulation::SharedPtr simulation_ptr);

    rclcpp::Subscription<filter_test::msg::Simulation>::SharedPtr armors_sub_;
    rclcpp::Publisher<filter_test::msg::Result>::SharedPtr target_pub_;

    ArmorFilter armor_filter;
    
    bool init; 

};



#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>

#include "filter_test/filter.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>

class ArmorTest
{

public:
    ArmorTest(/* args */);

private:
    void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;

    ArmorFilter armor_filter;

};



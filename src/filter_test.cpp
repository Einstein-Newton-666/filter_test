#include "filter_test/filter_test.hpp"

ArmorTest::ArmorTest(/* args */){
}

void ArmorTest::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr){

}

int main(){
    ArmorFilter af;
    auto_aim_interfaces::msg::Armors::SharedPtr armors_msg = std::make_shared<auto_aim_interfaces::msg::Armors>();;
    auto_aim_interfaces::msg::Armor armor;
    tf2::Quaternion q;
    q.setRPY(0, 0, 1.5708);  // 绕 Z 轴旋转 90 度（弧度）
    armor.pose.orientation.x = q.x();
    armor.pose.orientation.y = q.y();
    armor.pose.orientation.z = q.z();
    armor.pose.orientation.w = q.w();
    armor.pose.position.x = 1.0;
    armor.pose.position.y = 2.0;
    armor.pose.position.z = 0.5;
    armors_msg->armors.push_back(armor);

    armors_msg->header.stamp = rclcpp::Clock().now();
    af.init(armors_msg);
    armors_msg->header.stamp = rclcpp::Clock().now() + rclcpp::Duration::from_seconds(0.01);
    af.update(armors_msg);
}
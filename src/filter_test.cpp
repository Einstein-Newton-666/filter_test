#include "filter_test/filter_test.hpp"

ArmorTest::ArmorTest(const rclcpp::NodeOptions & options)
: Node("filter_test", options){
    init = false;
    target_pub_ = this->create_publisher<filter_test::msg::Result>(
        "/track_result", rclcpp::SensorDataQoS());
    armors_sub_ = this->create_subscription<filter_test::msg::Simulation>(
        "/armor_simulation", rclcpp::SensorDataQoS(),bind(&ArmorTest::armorsCallback,this,std::placeholders::_1));

}

void ArmorTest::armorsCallback(const filter_test::msg::Simulation::SharedPtr simulation_ptr){
    auto armors_ptr = std::make_shared<auto_aim_interfaces::msg::Armors>();
    *armors_ptr = simulation_ptr->armors;
    if(!init){
        armor_filter.init(armors_ptr);
    } else {
        armor_filter.update(armors_ptr);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorTest>());
    rclcpp::shutdown();
    return 0;
}
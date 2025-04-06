#include "filter_test/filter_test.hpp"

ArmorTest::ArmorTest(const rclcpp::NodeOptions & options)
: Node("filter_test", options){
    init = false;
    result_pub_ = this->create_publisher<filter_test::msg::Result>(
        "/track_result", rclcpp::SensorDataQoS());
    simulation_sub_ = this->create_subscription<filter_test::msg::Simulation>(
        "/armor_simulation", rclcpp::SensorDataQoS(), bind(&ArmorTest::armorsCallback,this,std::placeholders::_1));

}

//TODO:添加可视化
void ArmorTest::armorsCallback(const filter_test::msg::Simulation::SharedPtr simulation_ptr){
    auto armors_ptr = std::make_shared<auto_aim_interfaces::msg::Armors>();
    *armors_ptr = simulation_ptr->armors;
    if(!init){
        armor_filter.init(armors_ptr);
        init = true;
    } else {
        auto status = armor_filter.update(armors_ptr);
        filter_test::msg::Result result;
        result.header = simulation_ptr->armors.header;
        result.position.x = status[0]; 
        result.position.y = status[2]; 
        result.position.z = status[4]; 
        //没有z轴速度
        result.velocity.x = status[1];
        result.velocity.y = status[3];
        result.dz = status[5] - status[4];
        result.yaw = status[6];
        result.v_yaw = status[7];
        result.radius_1 = status[8];
        result.radius_2 = status[9];
        //计算观测偏差
        result.position_x_diff = (simulation_ptr->position.x - status[0])/simulation_ptr->position.x;
        result.position_y_diff = (simulation_ptr->position.y - status[2])/simulation_ptr->position.y;
        result.position_z1_diff = (simulation_ptr->position.z - status[4])/simulation_ptr->position.z;
        result.position_z2_diff = (simulation_ptr->dz - result.dz)/(simulation_ptr->position.z + simulation_ptr->dz);
        result.position_yaw_diff = (simulation_ptr->yaw - status[6])/simulation_ptr->yaw;
        result.r1_diff = (simulation_ptr->radius_1 - status[8])/simulation_ptr->radius_1;
        result.r2_diff = (simulation_ptr->radius_2 - status[9])/simulation_ptr->radius_2;
        result.velocity_x_diff = (simulation_ptr->velocity.x - status[1]) / simulation_ptr->velocity.x;
        result.velocity_y_diff = (simulation_ptr->velocity.y - status[3]) / simulation_ptr->velocity.y;
        result.velocity_yaw_diff = (simulation_ptr->v_yaw - status[7]) / simulation_ptr->v_yaw;
        result_pub_->publish(result);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorTest>());
    rclcpp::shutdown();
    return 0;
}
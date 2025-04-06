#include "filter_test/armor_simulation.hpp"

ArmorSimulation::ArmorSimulation(const rclcpp::NodeOptions & options) 
: Node("armor_simulation", options) {
    // 参数声明
    publish_rate = 100;
    linear_limit = 20, angle_limit = M_PI * 4;
    linear_speed_limit = 3, angle_speed_limit = M_PI;
    linear_acc = 0.5, angle_acc = M_PI_4;
    
    //设置初值
    x = y = yaw = 0;
    x_v = 0.5; 
    y_v = -0.5;
    yaw_v = M_PI_4;
    z1 = 0.3, z2 = 0.2;
    r1 = 0.2, r2 = 0.3;

    // 创建发布者
    sim_pub_ = this->create_publisher<filter_test::msg::Simulation>(
      "/armor_simulation", rclcpp::SensorDataQoS());

    // 初始化定时器
    timer_ = this->create_wall_timer(
      1000ms / publish_rate, std::bind(&ArmorSimulation::publishSimulation, this));
  }

void ArmorSimulation::publishSimulation() {
    auto sim_msg = filter_test::msg::Simulation();
    
    double t = this->now().seconds();
    double dt = std::min(t - last_t, 1.);
    last_t = t;

    x += x_v * dt + 0.5 * x_a * dt * dt;
    y += y_v * dt + 0.5 * y_a * dt * dt;
    yaw += yaw_v * dt + 0.5 * yaw_a * dt * dt;
    x_v += x_a * dt;
    y_v += y_a * dt;
    yaw_v += yaw_a * dt; 

    double angle[4];
    std::vector<int> indexs;
    for (size_t i = 0; i < 4; i++)
    {
        angle[i] = yaw + M_PI_2 * i;
        if(angles::normalize_angle(angle[i]) < M_PI_4 * 1.5 && angles::normalize_angle(angle[i]) > - M_PI_4 * 1.5){
            indexs.push_back(i);
        }
    }

    if(abs(x_v) > linear_speed_limit) x_a = 0;    
    if(abs(y_v) > linear_speed_limit) y_a = 0;    
    if(abs(yaw_v) > angle_speed_limit) yaw_a = 0;    
    if(abs(x) > linear_limit) x_a = - x / abs(x) * linear_acc;    
    if(abs(y) > linear_limit) y_a = - y / abs(y) * linear_acc;    
    if(abs(yaw) > angle_limit) yaw_a = - yaw / abs(yaw) * angle_acc;    

    for (size_t i = 0; i < indexs.size(); i++)
    {
        auto_aim_interfaces::msg::Armor armor;
        armor.pose.position.x = x + cos(yaw + indexs[i] * M_PI_2) * (indexs[i] % 2 == 0 ? r1 : r2);
        armor.pose.position.y = y + sin(yaw + indexs[i] * M_PI_2) * (indexs[i] % 2 == 0 ? r1 : r2);
        armor.pose.position.z = indexs[i] % 2 == 0 ? z1 : z2;
        tf2::Quaternion q;
        q.setRPY(0, 0, angles::normalize_angle(yaw + indexs[i] * M_PI_2));
        armor.pose.orientation = tf2::toMsg(q);
        sim_msg.armors.armors.push_back(armor);
    }
    
    sim_msg.armors.header.stamp = this->now();
    sim_msg.position.x = x;
    sim_msg.position.y = y;
    sim_msg.position.z = z1;
    sim_msg.velocity.x = x_v;
    sim_msg.velocity.y = y_v;
    sim_msg.yaw = angles::normalize_angle(yaw);
    sim_msg.v_yaw = yaw_v;
    sim_msg.dz = z2 - z1;
    sim_msg.radius_1 = r1;
    sim_msg.radius_2 = r2;
    
    sim_pub_->publish(sim_msg);
  }

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorSimulation>());
    rclcpp::shutdown();
    return 0;
}
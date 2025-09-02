#include "filter_test/armor_simulation.hpp"

ArmorSimulation::ArmorSimulation(const rclcpp::NodeOptions & options) 
: Node("armor_simulation", options) {
    // // 参数声明
    // publish_rate = 10;
    // linear_limit = 3, angle_limit = M_PI * 2; // 位姿和角度限制
    // linear_speed_limit = 1.0, angle_speed_limit = 8.0; // 线速度和角速度限制
    // linear_acc = 0.1, angle_acc = M_PI_4 /2; // 线角速度和角加速度
    
    // //设置初值
    // x = 2;
    // y = -2;
    // yaw = 0;
    // x_v = 0.; 
    // y_v = -0.;
    // yaw_v = 5.0;
    // x_a = 0.0, y_a = 0.0, yaw_a = 0.0;
    // z1 = 0.3, z2 = 0.2;
    // r1 = 0.25, r2 = 0.35;

    publish_rate = this->declare_parameter<int>("publish_rate", 10);
    linear_limit = this->declare_parameter<double>("linear_limit", 3.0);
    angle_limit = this->declare_parameter<double>("angle_limit", 2 * M_PI);

    // 运动限制
    linear_speed_limit = this->declare_parameter<double>("linear_speed_limit", 1.0);
    angle_speed_limit = this->declare_parameter<double>("angle_speed_limit", 8.0);

    // 加速度参数
    linear_acc = this->declare_parameter<double>("linear_acceleration", 0.1);
    angle_acc = this->declare_parameter<double>("angular_acceleration", M_PI_4 / 2.0);
    
    // 初始状态
    x = this->declare_parameter<double>("initial_state.x", 2.0);
    y = this->declare_parameter<double>("initial_state.y", -2.0);
    yaw = this->declare_parameter<double>("initial_state.yaw", 0.0);
    x_v = this->declare_parameter<double>("initial_state.x_velocity", 0.0);
    y_v = this->declare_parameter<double>("initial_state.y_velocity", 0.0);
    yaw_v = this->declare_parameter<double>("initial_state.yaw_velocity", 5.0);
    x_a = this->declare_parameter<double>("initial_state.x_acceleration", 0.0);
    y_a = this->declare_parameter<double>("initial_state.y_acceleration", 0.0);
    yaw_a = this->declare_parameter<double>("initial_state.yaw_acceleration", 0.0);
    
    // 几何参数
    z1 = this->declare_parameter<double>("geometry.z1", 0.3);
    z2 = this->declare_parameter<double>("geometry.z2", 0.2);
    r1 = this->declare_parameter<double>("geometry.r1", 0.25);
    r2 = this->declare_parameter<double>("geometry.r2", 0.35);

    // 初始化last_t
    last_t = this->now();
    position_marker_.ns = "position_";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.r = 0.0;
    position_marker_.color.g = 1.0;
    position_marker_.color.b = 1.0;
    armor_marker_.ns = "armors_";
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.05;
    armor_marker_.scale.y = 0.135;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.g = 0.5;
    armor_marker_.color.b = 1.0;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
  
    marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/simulation/marker", 10);

    // 创建发布者
    sim_pub_ = this->create_publisher<filter_test::msg::Simulation>(
      "/armor_simulation", rclcpp::SensorDataQoS());

    armor_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
      "/detector/armors", rclcpp::SensorDataQoS());
    // 初始化定时器
    timer_ = this->create_wall_timer(
      1000ms / publish_rate, std::bind(&ArmorSimulation::publishSimulation, this));
  }

void ArmorSimulation::publishSimulation() {
    filter_test::msg::Simulation sim_msg;
    
    // rclcpp::Time now = this->now();
    rclcpp::Time now = this->now();
    rclcpp::Duration dt_duration = now - last_t; // 计算时间差
    double dt = std::min(dt_duration.seconds(), 1.); // 将时间差转换为秒并取最小值
    last_t = now;

    x += x_v * dt + 0.5 * x_a * dt * dt;
    y += y_v * dt + 0.5 * y_a * dt * dt;
    yaw += yaw_v * dt + 0.5 * yaw_a * dt * dt;
    x_v += x_a * dt;
    y_v += y_a * dt;
    yaw_v += yaw_a * dt; 

    auto_aim_interfaces::msg::Armors armors_msg;
    armor_marker_.header.stamp = sim_msg.armors.header.stamp = now;
    armor_marker_.header.frame_id = sim_msg.armors.header.frame_id = "odom";
    armors_msg.header = sim_msg.armors.header;
    marker_array_.markers.clear();
    armor_marker_.id = 0;

    //发布可见装甲板位姿
    for (size_t i = 0; i < 4; i++)
    {
        //计算4个装甲板位姿
        auto_aim_interfaces::msg::Armor armor;
        double armor_yaw = yaw + i * M_PI_2;
        double r = i % 2 == 0 ? r1 : r2;
        armor.pose.position.x = x - cos(armor_yaw) * r;
        armor.pose.position.y = y - sin(armor_yaw) * r;
        armor.pose.position.z = i % 2 == 0 ? z1 : z2;
        tf2::Quaternion q;
        q.setRPY(0, 0, angles::normalize_angle(armor_yaw));
        armor.pose.orientation = tf2::toMsg(q);
        //判断装甲板是否在可见角度范围内
        // if(abs(armor_yaw-(atan2(y,x)+M_PI))<2*M_PI/3){
        if(angles::normalize_angle(armor_yaw) < M_PI_4 * 1.5 && angles::normalize_angle(armor_yaw) > - M_PI_4 * 1.5){
            armor.type = "small";
            armor.priority = 1;
            armor.number = "4";
            armors_msg.armors.push_back(armor);
            sim_msg.armors.armors.push_back(armor);
            // RCLCPP_INFO(this->get_logger(), "第%ld个装甲板,x: %f,y: %f,z: %f,armor_yaw: %f", i, armor.pose.position.x,
            //                                                                                   armor.pose.position.y,
            //                                                                                   armor.pose.position.z,
            //                                                                                   armor_yaw);
        }

        armor_marker_.id++;
        armor_marker_.pose = armor.pose;        
        q.setRPY(0, 0.26, angles::normalize_angle(armor_yaw));
        armor_marker_.pose.orientation = tf2::toMsg(q);
        marker_array_.markers.emplace_back(armor_marker_);
    }
    
    sim_msg.position.x = x;
    sim_msg.position.y = y;
    sim_msg.position.z = z1;
    sim_msg.velocity.x = x_v;
    sim_msg.velocity.y = y_v;
    // sim_msg.yaw = angles::normalize_angle(yaw);
    sim_msg.yaw = yaw;
    sim_msg.v_yaw = yaw_v;
    sim_msg.dz = z2 - z1;
    sim_msg.radius_1 = r1;
    sim_msg.radius_2 = r2;

    position_marker_.header = sim_msg.armors.header;
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = x;
    position_marker_.pose.position.y = y;
    position_marker_.pose.position.z = (z1+z2)/2;
    
    sim_pub_->publish(sim_msg);
    armor_pub_->publish(armors_msg);
    marker_array_.markers.emplace_back(position_marker_);
    marker_array_.markers.emplace_back(armor_marker_);
    marker_pub_->publish(marker_array_);

    
    //整车模型运动控制
    // if(abs(x) > linear_limit){
    //     x_a += (x_v > 0 ? -1 : 1) * 0.1;
    // }
    // if(abs(y) > linear_limit){
    //     y_a += (y_v > 0 ? -1 : 1) * 0.1;
    // }
    // if(abs(yaw_v) > angle_speed_limit){
    //     yaw_a += (yaw_v > 0 ? -1 : 1) * 0.5;
    // }
    
    //角速度限制
    constexpr double eps = 1e-6;  // 防止除零
    if((abs(x_v) > linear_speed_limit) && linear_speed_limit != -1) { x_v = x_v / (abs(x_v) + eps) * linear_speed_limit;}
    if((abs(y_v) > linear_speed_limit) && linear_speed_limit != -1) { y_v = y_v / (abs(y_v) + eps) * linear_speed_limit;}   
    if((abs(yaw_v) > angle_speed_limit) && angle_speed_limit != -1) { yaw_v = yaw_v / (abs(yaw_v) + eps) * angle_speed_limit;}    
    //位置过大时开始刹车并反向运动
    if((abs(x) > linear_limit) && linear_limit != -1) { x_a = - x / (abs(x) + eps) * linear_acc;}
    if((abs(y) > linear_limit) && linear_limit != -1) { y_a = - y / (abs(y) + eps) * linear_acc;}   
    if((abs(yaw) > angle_limit) && angle_limit != -1) { yaw_a = - yaw / (abs(yaw) + eps) * angle_acc;}    
    // if(abs(last_v_yaw - yaw_v) > 1){
    //   RCLCPP_ERROR_STREAM(this->get_logger(), "error" + std::to_string(dt));
    // }
    // if(abs(yaw) > angle_limit) yaw = 0;    
    last_v_yaw = yaw_v;

    // dt异常监控
    if (dt > 0.2) {
        RCLCPP_WARN(this->get_logger(), "dt过大: %f", dt);
    }
  }

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorSimulation>());
    rclcpp::shutdown();
    return 0;
}
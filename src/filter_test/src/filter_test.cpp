#include "filter_test/filter_test.hpp"

ArmorTest::ArmorTest(const rclcpp::NodeOptions & options)
: Node("filter_test", options){
    init = false;
    result_pub_ = this->create_publisher<filter_test::msg::Result>(
        "/track_result", rclcpp::SensorDataQoS());
    simulation_sub_ = this->create_subscription<filter_test::msg::Simulation>(
        "/armor_simulation", rclcpp::SensorDataQoS(), bind(&ArmorTest::armorsCallback,this,std::placeholders::_1));

    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;
    linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.color.a = 1.0;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;
    angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    angular_v_marker_.ns = "angular_v";
    angular_v_marker_.scale.x = 0.03;
    angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.color.a = 1.0;
    angular_v_marker_.color.b = 1.0;
    angular_v_marker_.color.g = 1.0;
    armor_marker_.ns = "armors";
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.03;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.r = 1.0;
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/track_result/marker", 10);

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

        position_marker_.header = simulation_ptr->armors.header;
        linear_v_marker_.header = simulation_ptr->armors.header;
        angular_v_marker_.header = simulation_ptr->armors.header;
        armor_marker_.header = simulation_ptr->armors.header;
      
        visualization_msgs::msg::MarkerArray marker_array;
        double yaw = result.yaw, r1 = result.radius_1, r2 = result.radius_2;
        double xc = result.position.x, yc = result.position.y, za = result.position.z;
        double vx = result.velocity.x, vy = result.velocity.y, vz = result.velocity.z;
        double dz = result.dz;
    
        position_marker_.action = visualization_msgs::msg::Marker::ADD;
        position_marker_.pose.position.x = xc;
        position_marker_.pose.position.y = yc;
        position_marker_.pose.position.z = za + dz / 2;
    
        linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        linear_v_marker_.points.clear();
        linear_v_marker_.points.emplace_back(position_marker_.pose.position);
        geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
        arrow_end.x += vx;
        arrow_end.y += vy;
        arrow_end.z += vz;
        linear_v_marker_.points.emplace_back(arrow_end);
    
        angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        angular_v_marker_.points.clear();
        angular_v_marker_.points.emplace_back(position_marker_.pose.position);
        arrow_end = position_marker_.pose.position;
        arrow_end.z += result.v_yaw / M_PI;
        angular_v_marker_.points.emplace_back(arrow_end);
    
        armor_marker_.action = visualization_msgs::msg::Marker::ADD;
        armor_marker_.scale.y =  0.135;
        bool is_current_pair = true;
        size_t a_n = 4;
        geometry_msgs::msg::Point p_a;
        double r = 0;
        for (size_t j = 0; j < a_n; j++) {
            double tmp_yaw = yaw + j * (2 * M_PI / a_n);
            // Only 4 armors has 2 radius and height
            r = is_current_pair ? r1 : r2;
            p_a.z = za + (is_current_pair ? 0 : dz);
            is_current_pair = !is_current_pair;

            p_a.x = xc - r * cos(tmp_yaw);
            p_a.y = yc - r * sin(tmp_yaw);
        
            armor_marker_.id = j;
            armor_marker_.pose.position = p_a;
            tf2::Quaternion q;
            q.setRPY(0, -0.26, tmp_yaw);
            armor_marker_.pose.orientation = tf2::toMsg(q);
            marker_array.markers.emplace_back(armor_marker_);
        }
        marker_array.markers.emplace_back(position_marker_);
        marker_array.markers.emplace_back(linear_v_marker_);
        marker_array.markers.emplace_back(angular_v_marker_);
        marker_pub_->publish(marker_array);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorTest>());
    rclcpp::shutdown();
    return 0;
}
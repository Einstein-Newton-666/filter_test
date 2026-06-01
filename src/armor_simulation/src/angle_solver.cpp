/**
 * 角度解算器 placeholder (对齐 autoaim)
 *
 * 接收 tracker 输出，计算目标云台角度。
 * 当前为简单几何解算，未来可扩展: 弹道补偿 + TinyMPC 规划器。
 */
#include <rclcpp/rclcpp.hpp>
#include <auto_aim_interfaces/msg/tracker_target.hpp>
#include <auto_aim_interfaces/msg/send_data.hpp>
#include <auto_aim_interfaces/msg/recieve_data.hpp>

#include <cmath>

class AngleSolver : public rclcpp::Node {
public:
    AngleSolver() : Node("angle_solver") {
        // 订阅 tracker 输出
        tracker_sub_ = create_subscription<auto_aim_interfaces::msg::TrackerTarget>(
            "/tracker/target", rclcpp::SensorDataQoS(),
            [this](const auto_aim_interfaces::msg::TrackerTarget::SharedPtr msg) {
                targetCallback(msg);
            });

        // 订阅云台反馈 (用于闭环)
        recieve_sub_ = create_subscription<auto_aim_interfaces::msg::RecieveData>(
            "/recieve_pack", rclcpp::SensorDataQoS(),
            [this](const auto_aim_interfaces::msg::RecieveData::SharedPtr msg) {
                current_yaw_   = msg->yaw   * M_PI / 180.0;
                current_pitch_ = msg->pitch * M_PI / 180.0;
            });

        // 发布云台指令
        send_pub_ = create_publisher<auto_aim_interfaces::msg::SendData>(
            "/send_pack", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(), "Angle solver initialized (placeholder, simple geometry)");
    }

private:
    void targetCallback(const auto_aim_interfaces::msg::TrackerTarget::SharedPtr msg) {
        if (!msg->enemy.tracking) return;

        // 简单几何解算: 直接从 tracker 位置计算云台目标角度
        double x = msg->enemy.position.x;
        double y = msg->enemy.position.y;
        double z = msg->enemy.position.z;

        double target_yaw   = std::atan2(y, x);             // R_o2c 光轴 (cosθ,sinθ) ∥ (x,y)
        double target_pitch = -std::atan2(z, std::sqrt(x*x + y*y));

        auto_aim_interfaces::msg::SendData send;
        send.header.stamp = msg->header.stamp;
        send.yaw   = static_cast<float>(target_yaw   * 180.0 / M_PI);
        send.pitch = static_cast<float>(target_pitch * 180.0 / M_PI);
        send.enable_shoot = 0;
        send_pub_->publish(send);
    }

    double current_yaw_ = 0.0, current_pitch_ = 0.0;

    rclcpp::Subscription<auto_aim_interfaces::msg::TrackerTarget>::SharedPtr tracker_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::RecieveData>::SharedPtr recieve_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::SendData>::SharedPtr send_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleSolver>());
    rclcpp::shutdown();
    return 0;
}

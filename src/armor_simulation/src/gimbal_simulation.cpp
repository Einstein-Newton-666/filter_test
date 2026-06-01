/**
 * 云台仿真节点 (对齐 autoaim)
 *
 * 接收角度解算器的目标角度，模拟云台动力学 (S-curve + 限速)，
 * 发布 TF odom→gimbal_link 和 RecieveData 反馈。
 *
 * 闭环: tracker → angle_solver → gimbal_sim → TF → tracker
 */
#include <rclcpp/rclcpp.hpp>
#include <auto_aim_interfaces/msg/send_data.hpp>
#include <auto_aim_interfaces/msg/recieve_data.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>

using namespace std::chrono_literals;

class GimbalSimulation : public rclcpp::Node {
public:
    GimbalSimulation() : Node("gimbal_simulation") {
        // 云台参数
        declare_parameter("publish_rate", 500);
        declare_parameter("max_yaw_vel", M_PI / 0.01);       // rad/s
        declare_parameter("max_pitch_vel", M_PI / 0.01);
        declare_parameter("max_yaw_acc", M_PI / 0.001);      // rad/s²
        declare_parameter("max_pitch_acc", M_PI / 0.01);
        declare_parameter("s_curve_duration", 0.1);          // S-curve 总时间 (s)
        declare_parameter("shoot_speed", 23.0);               // 弹速 m/s
        declare_parameter("camera_gimbal_tx", 0.0);
        declare_parameter("camera_gimbal_ty", -0.045);
        declare_parameter("camera_gimbal_tz", 0.08557);

        publish_rate_        = get_parameter("publish_rate").as_int();
        max_yaw_vel_         = get_parameter("max_yaw_vel").as_double();
        max_pitch_vel_       = get_parameter("max_pitch_vel").as_double();
        max_yaw_acc_         = get_parameter("max_yaw_acc").as_double();
        max_pitch_acc_       = get_parameter("max_pitch_acc").as_double();
        s_curve_duration_    = get_parameter("s_curve_duration").as_double();
        shoot_speed_         = get_parameter("shoot_speed").as_double();
        camera_gimbal_tx_    = get_parameter("camera_gimbal_tx").as_double();
        camera_gimbal_ty_    = get_parameter("camera_gimbal_ty").as_double();
        camera_gimbal_tz_    = get_parameter("camera_gimbal_tz").as_double();

        // 订阅角度解算器输出
        send_sub_ = create_subscription<auto_aim_interfaces::msg::SendData>(
            "/send_pack", rclcpp::SensorDataQoS(),
            [this](const auto_aim_interfaces::msg::SendData::SharedPtr msg) {
                target_yaw_   = msg->yaw   * M_PI / 180.0;
                target_pitch_ = msg->pitch * M_PI / 180.0;
                start_yaw_   = current_yaw_;
                start_pitch_ = current_pitch_;
                elapsed_yaw_   = 0.0;
                elapsed_pitch_ = 0.0;
            });

        // 发布云台反馈
        recieve_pub_ = create_publisher<auto_aim_interfaces::msg::RecieveData>(
            "/recieve_pack", rclcpp::SensorDataQoS());

        // TF 广播
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 初始化 last_time_
        last_time_ = get_clock()->now();

        // 定时器 — 高频率更新云台状态
        auto dt = std::chrono::microseconds(1000000 / publish_rate_);
        timer_ = create_wall_timer(dt, std::bind(&GimbalSimulation::update, this));

        RCLCPP_INFO(get_logger(), "Gimbal simulation initialized (rate=%dHz, S-curve=%.0fms)",
                    publish_rate_, s_curve_duration_ * 1000);
    }

private:
    void update() {
        auto now = get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0 || dt > 0.1) dt = 0.002;
        last_time_ = now;

        // ── S-curve 加速度插值 ──
        // 目标变化时重置 S-curve
        double err_yaw   = std::abs(target_yaw_ - current_yaw_);
        double err_pitch = std::abs(target_pitch_ - current_pitch_);
        if (err_yaw > 1e-6) {
            elapsed_yaw_ += dt;
        } else {
            elapsed_yaw_ = 0;
            start_yaw_ = current_yaw_;
        }
        if (err_pitch > 1e-6) {
            elapsed_pitch_ += dt;
        } else {
            elapsed_pitch_ = 0;
            start_pitch_ = current_pitch_;
        }

        // S-curve: s(t) = 3t² - 2t³  (t ∈ [0,1])
        double delta_yaw   = target_yaw_ - start_yaw_;
        double delta_pitch = target_pitch_ - start_pitch_;
        double s_yaw   = sCurve(elapsed_yaw_   / s_curve_duration_);
        double s_pitch = sCurve(elapsed_pitch_ / s_curve_duration_);

        double step_yaw   = delta_yaw   * s_yaw   - current_yaw_ + start_yaw_;
        double step_pitch = delta_pitch * s_pitch - current_pitch_ + start_pitch_;

        // 角速度限幅
        double max_step_yaw   = max_yaw_vel_   * dt;
        double max_step_pitch = max_pitch_vel_ * dt;
        step_yaw   = std::clamp(step_yaw,   -max_step_yaw,   max_step_yaw);
        step_pitch = std::clamp(step_pitch, -max_step_pitch, max_step_pitch);

        current_yaw_   += step_yaw;
        current_pitch_ += step_pitch;

        // ── 发布 RecieveData ──
        auto_aim_interfaces::msg::RecieveData recv;
        recv.header.stamp = now;
        recv.yaw   = static_cast<float>(current_yaw_   * 180.0 / M_PI);
        recv.pitch = static_cast<float>(current_pitch_ * 180.0 / M_PI);
        recv.shoot_speed = static_cast<float>(shoot_speed_);
        recieve_pub_->publish(recv);

        // ── TF 树 (移植自 autoaim rm_gimbal_description URDF) ──
        // odom → gimbal_link → camera_link → camera_optical_frame
        // camera_optical_joint: RPY ${-π/2} 0 ${-π/2}
        // 推导: 相机光轴在 gimbal +X, yaw=atan2(rx,-ry) 时光轴对准 (rx,ry)

        // 1) odom → gimbal_link (云台 yaw/pitch 动态控制)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = now;
            tf.header.frame_id = "odom";
            tf.child_frame_id = "gimbal_link";
            tf.transform.translation.x = 0.0;
            tf.transform.translation.y = 0.0;
            tf.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, current_pitch_, current_yaw_);
            tf.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(tf);
        }

        // 2) gimbal_link → camera_link (相机物理安装偏移, autoaim 默认 xyz="0.08 0 0.07")
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = now;
            tf.header.frame_id = "gimbal_link";
            tf.child_frame_id = "camera_link";
            tf.transform.translation.x = camera_gimbal_tx_;
            tf.transform.translation.y = camera_gimbal_ty_;
            tf.transform.translation.z = camera_gimbal_tz_;
            tf2::Quaternion q;  // identity
            q.setRPY(0, 0, 0);
            tf.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(tf);
        }

        // 3) camera_link → camera_optical_frame (REP-103 光学坐标系)
        //    autoaim URDF camera_optical_joint: RPY ${-π/2} 0 ${-π/2}
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = now;
            tf.header.frame_id = "camera_link";
            tf.child_frame_id = "camera_optical_frame";
            tf.transform.translation.x = 0.0;
            tf.transform.translation.y = 0.0;
            tf.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(-M_PI_2, 0.0, -M_PI_2);
            tf.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(tf);
        }
    }

    static double sCurve(double t) {
        if (t <= 0.0) return 0.0;
        if (t >= 1.0) return 1.0;
        return 3.0 * t * t - 2.0 * t * t * t;
    }

    // 订阅
    rclcpp::Subscription<auto_aim_interfaces::msg::SendData>::SharedPtr send_sub_;
    // 发布
    rclcpp::Publisher<auto_aim_interfaces::msg::RecieveData>::SharedPtr recieve_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    // 云台状态
    double current_yaw_ = 0.0, current_pitch_ = 0.0;
    double target_yaw_ = 0.0, target_pitch_ = 0.0;
    double start_yaw_ = 0.0, start_pitch_ = 0.0;
    double elapsed_yaw_ = 0.0, elapsed_pitch_ = 0.0;

    // 参数
    int publish_rate_;
    double max_yaw_vel_, max_pitch_vel_, max_yaw_acc_, max_pitch_acc_;
    double s_curve_duration_;
    double camera_gimbal_tx_, camera_gimbal_ty_, camera_gimbal_tz_;
    double shoot_speed_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalSimulation>());
    rclcpp::shutdown();
    return 0;
}

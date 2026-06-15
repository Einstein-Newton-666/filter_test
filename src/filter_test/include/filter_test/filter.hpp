#pragma once
#include "rclcpp/rclcpp.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "filter_test/filters/extended_kalman.hpp"
#include "filter_test/filters/unscented_kalman.hpp"
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>

class ArmorFilter
{
public:
    ArmorFilter(bool use_ekf = true, bool use_cv_model = true,
        double s2qxy_cv = 0.5, double s2qz_cv = 1.0, double s2qyaw_cv = 0.5, double s2qr_cv = 10.0, double s2qdz_cv = 1.0,
        double s2qxy_singer = 0.5, double s2qz_singer = 1.0, double s2qyaw_singer = 0.5, double s2qr_singer = 10.0, double s2qdz_singer = 1.0,
        double tau_singer = 1.0,
        double ukf_alpha = 0.001, double ukf_beta = 2.0, double ukf_kappa = 0.0,
        double outpost_radius = 0.2765,
        double init_r = 0.25,
        double outpost_s2qxy_cv = 0.5, double outpost_s2qz_cv = 1.0, double outpost_s2qyaw_cv = 0.5, double outpost_s2qr_cv = 10.0, double outpost_s2qdz_cv = 1.0,
        double outpost_s2qxy_singer = 0.5, double outpost_s2qz_singer = 1.0, double outpost_s2qyaw_singer = 0.5, double outpost_s2qr_singer = 10.0, double outpost_s2qdz_singer = 1.0,
        double outpost_r_pose_det = 0.01, double outpost_r_distance_det = 0.01, double outpost_r_yaw_det = 0.05);

    void init(auto_aim_interfaces::msg::Armors::SharedPtr & armors_msg);

    Eigen::VectorXd update(const auto_aim_interfaces::msg::Armors::SharedPtr & armors_msg);

    Eigen::VectorXd get_last_result() const { return last_result_; }  // 获取最近一次滤波结果用于误差计算

    void set_r_for_simulation(double r_pose, double r_distance, double r_yaw);   // 仿真模式R参数（固定值，无PnP误差）
    void set_r_for_detector(double r_pose, double r_distance, double r_yaw);  // 检测器模式R参数（距离相关噪声）

private:
    double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

    Eigen::MatrixXd get_q(double dt_);

    Eigen::MatrixXd get_r(Eigen::VectorXd & z);

    Eigen::MatrixXd get_r_with_abs_yaw(Eigen::VectorXd & z, std::vector<double> & abs_yaws);

    ExtendedKalmanFilter ekf;
    UnscentedKalmanFilter ukf;

    double init_r;
    int last_armor_number;

    rclcpp::Time last_time_;

    // CV模型过程噪声参数
    double s2qxy_cv_, s2qz_cv_, s2qyaw_cv_, s2qr_cv_, s2qdz_cv_;
    // Singer模型过程噪声参数
    double s2qxy_singer_, s2qz_singer_, s2qyaw_singer_, s2qr_singer_, s2qdz_singer_;
    double outpost_s2qxy_cv_, outpost_s2qz_cv_, outpost_s2qyaw_cv_, outpost_s2qr_cv_, outpost_s2qdz_cv_;
    double outpost_s2qxy_singer_, outpost_s2qz_singer_, outpost_s2qyaw_singer_, outpost_s2qr_singer_, outpost_s2qdz_singer_;
    // Singer模型机动时间常数
    double tau_singer_;
    
    double r_pose_, r_distance_, r_yaw_;
    double outpost_r_pose_, outpost_r_distance_, outpost_r_yaw_;
    bool use_fixed_r_;  // true=固定R（仿真），false=距离相关R（检测器）

    double last_yaw;
    Eigen::VectorXd last_result_;  // 最近一次滤波结果，用于误差计算
    bool tracking_outpost_ = false;
    double outpost_radius_ = 0.2765;

    bool use_ekf_;      // true: EKF, false: UKF
    bool use_cv_model_; // true: CV模型, false: Singer模型
    
    // UKF参数
    double ukf_alpha_, ukf_beta_, ukf_kappa_;
};

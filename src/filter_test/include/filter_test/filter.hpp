#pragma once
#include "rclcpp/rclcpp.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "filter_test/extended_kalman.hpp"
#include "filter_test/unscented_kalman.hpp"
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>


class ArmorFilter
{
public:
    ArmorFilter();

    void init(auto_aim_interfaces::msg::Armors::SharedPtr & armors_msg);

    Eigen::VectorXd update(const auto_aim_interfaces::msg::Armors::SharedPtr & armors_msg);

private:
    double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

    Eigen::MatrixXd get_q(double dt_);

    Eigen::MatrixXd get_r(Eigen::VectorXd & z);

    ExtendedKalmanFilter ekf;
    UnscentedKalmanFilter ukf;

    double init_r;
    int last_armor_number;

    rclcpp::Time last_time_;

    double s2qxyz_, s2qyaw_, s2qr_;
    double r_pose, r_distance, r_yaw;

    double last_yaw;
};



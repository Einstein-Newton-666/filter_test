#pragma once

#include "filter_test/auto_graph_optimizer/graph_optimizer.hpp"

#include <auto_aim_interfaces/msg/armors.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/time.hpp>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace filter_test::graph_optimizer {

struct GeoNoise {
    double tangential = 0.01;
    double radial = 0.03;
    double height = 0.01;
    double yaw = 0.005;
};

struct PriorNoise {
    double radius = 1.0;
    double dz = 1.0;
};

struct TrackerConfig {
    bool use_2d_observation = false;

    double s2qxy = 0.1;
    double s2qz = 0.1;
    double s2qyaw = 0.1;
    double s2qr = 10.0;
    double s2qdz = 0.1;
    double s2qvel = 0.1;
    double s2qvyaw = 0.1;

    double vel_sigma = 0.01;
    double vyaw_sigma = 0.05;

    double r_pose = 0.01;
    double r_distance = 0.01;
    double r_yaw = 0.01;
    double pixel_sigma = 1.0;

    GeoNoise geo_noise;
    PriorNoise prior_noise;

    Eigen::Matrix3d camera_matrix = Eigen::Matrix3d::Identity();
    std::array<double, 5> distortion = {0.0, 0.0, 0.0, 0.0, 0.0};

    auto_graph::GraphOptimizerConfig optimizer;
};

struct TrackerState {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double vyaw = 0.0;
    double radius_1 = 0.25;
    double radius_2 = 0.25;
    double dz = 0.0;
};

struct PredictedArmor {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double radius = 0.0;
    double dz = 0.0;
    int index = 0;
};

struct TrackerFrameInput {
    const auto_aim_interfaces::msg::Armors& armors_msg;
    double dt = 0.01;
    Eigen::Isometry3d T_camera_to_odom = Eigen::Isometry3d::Identity();
};

struct TrackerUpdateResult {
    bool accepted_frame = false;
    bool initialized = false;
    bool solved = false;
    bool cold_start = false;
    bool solve_failed = false;
    std::string solve_error;
    uint64_t frame_id = 0;
    TrackerState state;
    std::vector<PredictedArmor> predicted_armors;
    std::vector<int> matched_indices;
};

class FrameTimeTracker {
public:
    double computeDt(const builtin_interfaces::msg::Time& stamp) const {
        rclcpp::Time current_time(stamp);
        if (!have_last_time_) return 0.01;

        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) {
            dt = 0.01;
        }
        return dt;
    }

    void commit(const builtin_interfaces::msg::Time& stamp) {
        last_time_ = rclcpp::Time(stamp);
        have_last_time_ = true;
    }

private:
    bool have_last_time_ = false;
    rclcpp::Time last_time_;
};

}  // namespace filter_test::graph_optimizer

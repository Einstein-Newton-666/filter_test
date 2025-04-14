#ifndef ARMOR_PROCESSOR__ENEMY_MODEL_HPP_
#define ARMOR_PROCESSOR__ENEMY_MODEL_HPP_

#include <ceres/ceres.h>
#include "armor_tracker/filter/extended_kalman_filter.hpp"
#include "armor_tracker/common.hpp"

namespace rm_auto_aim
{
// enemy model
// xa = x_armor, xc = x_robot_center
// state:[ xc, v_xc, yc, v_yc, za1, za2, yaw, v_yaw, r1, r2]
// index:[  0,    1,  2,    3,   4,   5,   6,     7,  8,  9]
// measurement: [xa, ya, za, yaw] to [ yaw, pitch, distance, orientation_yaw]

template<typename T>
void ceres_xyz_to_ypd(const T& xyz, T& ypd) {
    ypd[0] = ceres::atan2(xyz[1], xyz[0]); // yaw
    ypd[1] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1])); // pitch
    ypd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]); // distance
};

struct ekfPredict{
public:
    explicit ekfPredict(const double& dt_): dt(dt_) {}
    template<typename T>
    void operator()(const T& x_pre, T& x_cur) const {
        x_cur[0] = x_pre[0] + x_pre[1] * dt; // xc
        x_cur[1] = x_pre[1];                 // v_xc
        x_cur[2] = x_pre[2] + x_pre[3] * dt; // yc
        x_cur[3] = x_pre[3];                 // v_yc
        x_cur[4] = x_pre[4];                 // za1
        x_cur[5] = x_pre[5];                 // za2
        x_cur[6] = x_pre[6] + x_pre[7] * dt; // yaw
        x_cur[7] = x_pre[7];                 // v_yaw
        x_cur[8] = x_pre[8];                 // r1
        x_cur[9] = x_pre[9];                 // r2
    }

    int size = 10; // 状态向量大小

private:
    double dt;
};

struct ekfMeasureSingle{
public:
    explicit ekfMeasureSingle(const int i): I(i) {}
    template<typename T>
    void operator()(const T& x, T& z) const {
        const T xyz_armor = {
            x[0] - ceres::cos(x[6] + M_PI_2 * I) * x[8 + I % 2],
            x[2] - ceres::sin(x[6] + M_PI_2 * I) * x[8 + I % 2],
            x[4 + I % 2] 
        };

        z[0] = ceres::atan2(xyz_armor[1], xyz_armor[0]); // yaw
        z[1] = ceres::atan2(xyz_armor[2], ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1])); // pitch
        z[2] = ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1] + xyz_armor[2] * xyz_armor[2]); // distance
        z[3] = x[6] + M_PI_2 * I; // orientation_yaw
    }

    int input_size = 10; // 状态向量大小
    int output_size = 4; // 观测向量大小
    int I;

private:
};

struct ekfMeasureDouble{
public:
    explicit ekfMeasureDouble(int i, int j): I(i), J(j) {}
    template<typename T>
    void operator()(const T& x, T& z) const {
        int idx = 0;
        for(const auto& i: {I, J}){
            T xyz_armor = {
                x[0] - ceres::cos(x[6] + M_PI_2 * i) * x[8 + i % 2],
                x[2] - ceres::sin(x[6] + M_PI_2 * i) * x[8 + i % 2],
                x[4 + i % 2]
            };

            z[idx + 0] = ceres::atan2(xyz_armor[1], xyz_armor[0]); // yaw
            z[idx + 1] = ceres::atan2(xyz_armor[2], ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1])); // pitch
            z[idx + 2] = ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1] + xyz_armor[2] * xyz_armor[2]); // distance
            z[idx + 3] = x[6] + M_PI_2 * i; // orientation_yaw
            idx += 4;
        }
    }

    int input_size = 10; // 状态向量大小
    int output_size = 8; // 观测向量大小
    int I, J; // I J 匹配到的装甲板的索引

private:
};

class EnemyModel {
public:
    Eigen::MatrixXd update_Q(const double& dt);
    Eigen::MatrixXd update_R(const Eigen::VectorXd &z);

    ExtendedKalmanFilter ekf;

    TrackerState tracker_state;

    Eigen::VectorXd pri_estimation;
    Eigen::VectorXd post_estimation;

    int detect_count;
    int lost_count;
    int tracking_thres;
    int lost_thres;

    double s2qxy_max_, s2qxy_min_, s2qz_, s2qyaw_max_, s2qyaw_min_, s2qr_; // 过程噪声方差
    double r_pose, r_distance, r_yaw; // 观测噪声方差
};

}


#endif // ARMOR_PROCESSOR__ENEMY_MODEL_HPP_
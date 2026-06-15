#pragma once

#include <Eigen/Dense>
#include <initializer_list>
#include <vector>

namespace cv_model {
    // EKF
// xa = x_armor, xc = x_robot_center
// state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r1, r2, dz
// index: 0,  1,    2,  3,    4,   5,   6,      7,     8,   9,
// - yaw 需要在切换时 set?
// - 对于只有单独半径的装甲板， 不更新多余的 r 和 z
// measurement: xa, ya, za, yaw
// measurement2: yaw（相机）, pitch, distance, yaw(orient)
// f - Process function

struct Predict {
public:
    explicit Predict(const double& delta_t): delta_t(delta_t) {}
    template<typename T>
    void operator()(const T& x_pre, T& x_cur) const {
        x_cur[0] = x_pre[0] + this->delta_t * x_pre[1];
        x_cur[1] = x_pre[1];
        x_cur[2] = x_pre[2] + this->delta_t * x_pre[3];
        x_cur[3] = x_pre[3];
        x_cur[4] = x_pre[4] + this->delta_t * x_pre[5];
        x_cur[5] = x_pre[5];
        x_cur[6] = x_pre[6] + this->delta_t * x_pre[7];
        x_cur[7] = x_pre[7];
        x_cur[8] = x_pre[8];
        x_cur[9] = x_pre[9];
        x_cur[10] = x_pre[10];
    }
    
    //状态向量的大小: [xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r1, r2, dz]
    int size = 11;

private:
    double delta_t = 0.;
};

struct MeasureSingle {
public:
    // z 用于接收. 由 x 推导到 z
    // z: 当前需求比较的装甲板，rotate: super_yaw 到需求装甲的 rotate
    // ekf 之与测量转观测量
    // 用装甲板进行更新J
    // 状态: [xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r1, r2, dz]
    explicit MeasureSingle(const int i): I(i) {}
    template<typename T>
    void operator()(const T& x, T& z) const {
        // x[6] 是 super 板的 yaw, x[10] 是 dz（两块装甲板的高度差）
        // 装甲板位置：z = x[4] + I % 2 * x[10] (装甲板0用x[4]，装甲板1用x[4] + dz)
        const T xyz_armor = { 
            x[0] - ceres::cos(x[6] + M_PI_2 * I) * x[8 + I % 2],
            x[2] - ceres::sin(x[6] + M_PI_2 * I) * x[8 + I % 2],
            ((I % 2 == 0) ? x[4] : (x[4] + x[10]))
        };
        // T xyz_armor;
        // if(I % 2 == 0){
        //     xyz_armor = {
        //         x[0] - ceres::cos(x[6] + M_PI_2 * I) * x[8 + I % 2],
        //         x[2] - ceres::sin(x[6] + M_PI_2 * I) * x[8 + I % 2],
        //         x[4]
        //     };
        // }else{
        //     xyz_armor = {
        //         x[0] - ceres::cos(x[6] + M_PI_2 * I) * x[8 + I % 2],
        //         x[2] - ceres::sin(x[6] + M_PI_2 * I) * x[8 + I % 2],
        //         x[4] + x[10]
        //     };
        // }
        z[0] = ceres::atan2(xyz_armor[1], xyz_armor[0]); // yaw
        z[1] = ceres::atan2(xyz_armor[2], ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1])); // pitch
        z[2] = ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1] + xyz_armor[2] * xyz_armor[2]); // distance

        // orien_yaw = orien_yaw
        z[3] = x[6] + M_PI_2 * I;
    }

    //状态向量的大小: [xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r1, r2, dz]
    int input_size = 11;
    //观测向量的大小
    int output_size = 4;
    //I 匹配到的装甲板的索引
    int I;

private:
};


struct MeasureDouble {
public:
    // z 用于接收. 由 x 推导到 z
    // z: 当前需求比较的装甲板，rotate: super_yaw 到需求装甲的 rotate
    // ekf 之与测量转观测量
    // 用装甲板进行更新J
    // 状态: [xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r1, r2, dz]
    explicit MeasureDouble(int i, int j): I(i), J(j) {}
    template<typename T>
    void operator()(const T& x, T& z) const {
        int idx = 0;
        for (const auto &k : {I, J})
        {
            // x[6] 是 super 板的 yaw, x[10] 是 dz（两块装甲板的高度差）
            // 装甲板位置：z = x[4] + k % 2 * x[10] (装甲板0用x[4]，装甲板1用x[4] + dz)
            T xyz_armor = { 
                x[0] - ceres::cos(x[6] + M_PI_2 * k) * x[8 + k % 2],
                x[2] - ceres::sin(x[6] + M_PI_2 * k) * x[8 + k % 2],
                ((k % 2 == 0) ? x[4] : (x[4] + x[10]))
            };
            // T xyz_armor;
            // if(k % 2 == 0){
            //     xyz_armor = {
            //         x[0] - ceres::cos(x[6] + M_PI_2 * k) * x[8 + k % 2],
            //         x[2] - ceres::sin(x[6] + M_PI_2 * k) * x[8 + k % 2],
            //         x[4]
            //     };
            //  }else{
            //     xyz_armor = {
            //         x[0] - ceres::cos(x[6] + M_PI_2 * k) * x[8 + k % 2],
            //         x[2] - ceres::sin(x[6] + M_PI_2 * k) * x[8 + k % 2],
            //         x[4] + x[10]
            //     };
            // }
            z[idx + 0] = ceres::atan2(xyz_armor[1], xyz_armor[0]); // yaw
            z[idx + 1] = ceres::atan2(xyz_armor[2], ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1])); // pitch
            z[idx + 2] = ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1] + xyz_armor[2] * xyz_armor[2]); // distance
            // orien_yaw = orien_yaw
            z[idx + 3] = x[6] + M_PI_2 * k;
            idx += 4;
        }
    }
    
    //状态向量的大小: [xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r1, r2, dz]
    int input_size = 11;
    //观测向量的大小
    int output_size = 8;
    // I J 匹配到的装甲板的索引
    int I, J;

private:
};

struct MeasureOutpost {
public:
    explicit MeasureOutpost(std::initializer_list<int> indices)
        : output_size(static_cast<int>(indices.size()) * 4), indices_(indices) {}

    explicit MeasureOutpost(const std::vector<int>& indices)
        : output_size(static_cast<int>(indices.size()) * 4), indices_(indices) {}

    template<typename T>
    void operator()(const T& x, T& z) const {
        int out = 0;
        for (const int i : indices_) {
            const auto armor_yaw = x[6] + static_cast<double>(i) * 2.0 * M_PI / 3.0;
            const auto zero = x[0] * 0.0;
            const auto dz = i == 0 ? zero : (i == 1 ? x[9] : x[10]);
            const auto armor_x = x[0] - ceres::cos(armor_yaw) * x[8];
            const auto armor_y = x[2] - ceres::sin(armor_yaw) * x[8];
            const auto armor_z = x[4] + dz;
            z[out + 0] = ceres::atan2(armor_y, armor_x);
            z[out + 1] = ceres::atan2(
                armor_z, ceres::sqrt(armor_x * armor_x + armor_y * armor_y));
            z[out + 2] = ceres::sqrt(
                armor_x * armor_x + armor_y * armor_y + armor_z * armor_z);
            z[out + 3] = armor_yaw;
            out += 4;
        }
    }

    int input_size = 11;
    int output_size = 0;

private:
    std::vector<int> indices_;
};

inline Eigen::MatrixXd predict_q(double dt_, double s2qxy_, double s2qz_, double s2qyaw_, double s2qr_, double s2qdz_){
    Eigen::MatrixXd q(11, 11);
    double t = dt_, x = s2qxy_, z = s2qz_, y = s2qyaw_, r = s2qr_, dz = s2qdz_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    double q_z_z = pow(t, 4) / 4 * z, q_z_vz = pow(t, 3) / 2 * z, q_vz_vz = pow(t, 2) * z;
    double q_r = pow(t, 4) / 4 * r;
    double q_dz = pow(t, 4) / 4 * dz;
    // clang-format off
    //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r1      r2      dz
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,      0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,      0,      0,
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
          0,      0,      0,      0,      q_z_z,  q_z_vz, 0,      0,      0,      0,      0,
          0,      0,      0,      0,      q_z_vz, q_vz_vz,0,      0,      0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,      0,      0,
          0,      0,      0,      0,      0,      0,      0,      0,      q_r,    0,      0,
          0,      0,      0,      0,      0,      0,      0,      0,      0,      q_r,    0,
          0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      q_dz;
    // clang-format on      
    return q;
};

inline Eigen::MatrixXd predict_outpost_q(
    double dt_, double s2qxy_, double s2qz_, double s2qyaw_,
    double s2qr_, double s2qdz_) {
    auto q = predict_q(dt_, s2qxy_, s2qz_, s2qyaw_, s2qr_, s2qdz_);
    const double dz_q = std::pow(dt_, 4) / 4.0 * s2qdz_;
    q(9, 9) = dz_q;
    q(10, 10) = dz_q;
    return q;
}

inline Eigen::MatrixXd measure_r(Eigen::VectorXd & z, double r_pose, double r_distance, double r_yaw,
                          const std::vector<double> & abs_yaws = std::vector<double>{0.0},
                          bool use_fixed_r = false){
    Eigen::VectorXd r(z.size());
    for(int i = 0; i < (z.size() / 4); i++){
        const auto obs_index = static_cast<std::size_t>(i);
        double abs_yaw = (abs_yaws.size() > obs_index) ? abs_yaws[obs_index] : 0.0;
        if (use_fixed_r) {
            // 固定R模式：仿真数据无PnP误差
            r.segment(i * 4, 4) << r_pose, r_pose, r_distance, r_yaw;
        } else {
            // 距离相关R模式：检测器数据有PnP误差
            r.segment(i * 4, 4) << r_pose, 
                                    r_pose, 
                                    r_distance * pow(z[i * 4 + 2], 2) * (pow(abs(abs_yaw * M_PI / 180.0), 2) + 1), 
                                    log(abs(abs_yaw * M_PI / 180.0) + 1.0) * 0.01 + r_yaw;
        }
    }
    return r.asDiagonal();
};

} // namespace cv_model

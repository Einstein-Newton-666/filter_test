#pragma once

#include <Eigen/Dense>

namespace cv_model {
    // EKF
// xa = x_armor, xc = x_robot_center
// state: xc, v_xc, yc, v_yc, za1, za2, orient, v_yaw, r1, r2
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
        x_cur[4] = x_pre[4];
        x_cur[5] = x_pre[5];
        x_cur[6] = x_pre[6] + this->delta_t * x_pre[7];
        x_cur[7] = x_pre[7];
        x_cur[8] = x_pre[8];
        x_cur[9] = x_pre[9];
    }
    
    //状态向量的大小
    int size = 10;

private:
    double delta_t = 0.;
};

struct MeasureSingle {
public:
    // z 用于接收. 由 x 推导到 z
    // z: 当前需求比较的装甲板，rotate: super_yaw 到需求装甲的 rotate
    // ekf 之与测量转观测量
    // 用装甲板进行更新J
    explicit MeasureSingle(const int i): I(i) {}
    template<typename T>
    void operator()(const T& x, T& z) const {
        // x[6] 是 super 板的 yaw
        const T xyz_armor = { 
            x[0] + ceres::cos(x[6] + M_PI_2 * I) * x[8 + I % 2],
            x[2] + ceres::sin(x[6] + M_PI_2 * I) * x[8 + I % 2],
            x[4 + I % 2] 
        };
        // T ypd(3);  //这里T的类型是自定义类型的容器
        // ceres_xyz_to_ypd(xyz_armor, ypd);
        // for (int i = 0; i < 3; i++) {
        //     z[i] = ypd[i];
        // }
        z[0] = ceres::atan2(xyz_armor[1], xyz_armor[0]); // yaw
        z[1] = ceres::atan2(xyz_armor[2], ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1])); // pitch
        z[2] = ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1] + xyz_armor[2] * xyz_armor[2]); // distance
        // orien_yaw = orien_yaw
        z[3] = x[6] + M_PI_2 * I;
    }

    //状态向量的大小
    int input_size = 10;
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
    explicit MeasureDouble(int i, int j): I(i), J(j) {}
    template<typename T>
    void operator()(const T& x, T& z) const {
        int idx = 0;
        for (const auto &i : {I, J})
        {
            // x[6] 是 super 板的 yaw
            T xyz_armor = { 
                x[0] + ceres::cos(x[6] + M_PI_2 * i) * x[8 + i % 2],
                x[2] + ceres::sin(x[6] + M_PI_2 * i) * x[8 + i % 2],
                x[4 + i % 2] 
            };
            // T ypd(3);
            // ceres_xyz_to_ypd(xyz_armor, ypd);
            // for (int j = 0; j < 3; j++) {
            //     z[idx + j] = ypd[j];
            // }
            z[idx + 0] = ceres::atan2(xyz_armor[1], xyz_armor[0]); // yaw
            z[idx + 1] = ceres::atan2(xyz_armor[2], ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1])); // pitch
            z[idx + 2] = ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1] + xyz_armor[2] * xyz_armor[2]); // distance
            // orien_yaw = orien_yaw
            z[idx + 3] = x[6] + M_PI_2 * i;
            idx += 4;
        }
    }
    
    //状态向量的大小
    int input_size = 10;
    //观测向量的大小
    int output_size = 8;
    // I J 匹配到的装甲板的索引
    int I, J;

private:
};

Eigen::MatrixXd predict_q(double dt_, double s2qxyz_, double s2qyaw_, double s2qr_){
    Eigen::MatrixXd q(10, 10);
    double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
    double q_r = pow(t, 4) / 4 * r;
    // clang-format off
    //    xc      v_xc    yc      v_yc    za1     za2     yaw     v_yaw   r1      r2
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,      0,
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,
          0,      0,      0,      0,      s2qxyz_,  0,      0,      0,      0,      0,
          0,      0,      0,      0,      0,      s2qxyz_,  0,      0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,      0,
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,      0,
          0,      0,      0,      0,      0,      0,      0,      0,      q_r,    0,
          0,      0,      0,      0,      0,      0,      0,      0,      0,      q_r;
    // clang-format on
    return q;
};

Eigen::MatrixXd measure_r(Eigen::VectorXd & z, double r_pose, double r_distance, double r_yaw){
    Eigen::VectorXd r(z.size());
    for(int i = 0; i < (z.size() / 4); i++){
        r.segment(i *4 , 4) << r_pose, r_pose, abs(r_distance * z[i * 4 + 2]), r_yaw;
    }
    return r.asDiagonal();
};

} // namespace cv_model
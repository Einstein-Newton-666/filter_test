#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace singer_model {

/**
 * Singer模型的实现
 * 参考文献：Singer, R. A. (1970). Estimating optimal tracking filter performance 
 * for manned maneuvering targets. IEEE Transactions on Aerospace and Electronic Systems
 * 
 * 状态向量定义：x = [x, vx, ax,   // 位置、速度、加速度（X轴）
 *                    y, vy, ay,   // 位置、速度、加速度（Y轴）
 *                    z, vz, az,   // 位置、速度、加速度（Z轴）
 *                    yaw, vyaw, ayaw,  // 偏航角、角速度、角加速度
 *                    r1, r2]      // 装甲板半径（常值）
 */
struct Predict {
public:
    // delta_t为时间间隔，tau为机动时间常数
    explicit Predict(const double& delta_t, const double& tau = 1.0)
        : dt(delta_t), tau(tau) {}
    template<typename T>
    void operator()(const T& x_pre, T& x_cur) const {
        const double a = dt / tau;  // 时间比率
        const double exp_term = std::exp(-a);  // 指数衰减项
        const double tau_sq = tau * tau;
        
        // x, vx, ax
        x_cur[0] = x_pre[0] + x_pre[1] * dt + x_pre[2] * tau_sq * (a - 1 + exp_term);
        x_cur[1] = x_pre[1] + x_pre[2] * tau * (1 - exp_term);
        x_cur[2] = x_pre[2] * exp_term;
        
        // y, vy, ay
        x_cur[3] = x_pre[3] + x_pre[4] * dt + x_pre[5] * tau_sq * (a - 1 + exp_term);
        x_cur[4] = x_pre[4] + x_pre[5] * tau * (1 - exp_term);
        x_cur[5] = x_pre[5] * exp_term;
        
        // z, vz, az
        x_cur[6] = x_pre[6] + x_pre[7] * dt + x_pre[8] * tau_sq * (a - 1 + exp_term);
        x_cur[7] = x_pre[7] + x_pre[8] * tau * (1 - exp_term);
        x_cur[8] = x_pre[8] * exp_term;
        
        // yaw, vyaw, ayaw
        x_cur[9]  = x_pre[9]  + x_pre[10] * dt + x_pre[11] * tau_sq * (a - 1 + exp_term);
        x_cur[10] = x_pre[10] + x_pre[11] * tau * (1 - exp_term);
        x_cur[11] = x_pre[11] * exp_term;
        
        // r1, r2 常值
        x_cur[12] = x_pre[12];
        x_cur[13] = x_pre[13];
    }
    int size = 14; // 状态维度
private:
    double dt = 0.;   // 时间间隔
    double tau = 1.;  // 机动时间常数
};


struct MeasureSingle {
public:
    explicit MeasureSingle(const int i): I(i) {}
    template<typename T>
    void operator()(const T& x, T& z) const {
        // 计算装甲板在世界坐标系下的位置
        // x[9] 是 super 板的 yaw
        const T xyz_armor = { 
            x[0] + ceres::cos(x[9] + M_PI_2 * I) * x[12 + I % 2],
            x[3] + ceres::sin(x[9] + M_PI_2 * I) * x[12 + I % 2],
            x[6 + I % 2] 
        };

        // 观测量：yaw, pitch, distance, yaw(orient)
        z[0] = ceres::atan2(xyz_armor[1], xyz_armor[0]); // yaw
        z[1] = ceres::atan2(xyz_armor[2], ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1])); // pitch
        z[2] = ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1] + xyz_armor[2] * xyz_armor[2]); // distance
        z[3] = x[9] + M_PI_2 * I;
    }
    int input_size = 14;   // 状态维度
    int output_size = 4;   // 观测维度
    int I; // 匹配到的装甲板索引
private:
};


struct MeasureDouble {
public:
    explicit MeasureDouble(int i, int j): I(i), J(j) {}
    template<typename T>
    void operator()(const T& x, T& z) const {
        int idx = 0;
        for (const auto &k : {I, J}) {
            T xyz_armor = { 
                x[0] + ceres::cos(x[9] + M_PI_2 * k) * x[12 + k % 2],
                x[3] + ceres::sin(x[9] + M_PI_2 * k) * x[12 + k % 2],
                x[6 + k % 2] 
            };
            
            z[idx + 0] = ceres::atan2(xyz_armor[1], xyz_armor[0]); // yaw
            z[idx + 1] = ceres::atan2(xyz_armor[2], ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1])); // pitch
            z[idx + 2] = ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1] + xyz_armor[2] * xyz_armor[2]); // distance
            z[idx + 3] = x[9] + M_PI_2 * k;
            idx += 4;
        }
    }
    int input_size = 14;   // 状态维度
    int output_size = 8;   // 观测维度
    int I, J; // 匹配到的装甲板索引
private:
};

inline Eigen::MatrixXd predict_q(double dt, double s2qxyz, double s2qyaw, double s2qr) {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(14, 14);

    auto fill_block = [&](int idx, double sigma_a2, double tau) {
        const double alpha = 1.0 / tau;  // 机动频率
        const double exp_neg_alpha_dt = std::exp(-alpha * dt);
        const double exp_neg_2alpha_dt = std::exp(-2.0 * alpha * dt);
        
        // Singer论文中的标准公式
        const double q11 = sigma_a2 * (1.0 / (2.0 * alpha * alpha * alpha * alpha * alpha)) * 
                          (1.0 - exp_neg_2alpha_dt + 2.0 * alpha * dt + 
                           (2.0 * alpha * alpha * alpha * dt * dt * dt) / 3.0 - 
                           2.0 * alpha * alpha * dt * dt - 4.0 * alpha * dt * exp_neg_alpha_dt);
        
        const double q12 = sigma_a2 * (1.0 / (2.0 * alpha * alpha * alpha * alpha)) * 
                          (exp_neg_2alpha_dt + 1.0 - 2.0 * exp_neg_alpha_dt + 
                           2.0 * alpha * dt * exp_neg_alpha_dt - 2.0 * alpha * dt + 
                           alpha * alpha * dt * dt);
        
        const double q13 = sigma_a2 * (1.0 / (2.0 * alpha * alpha * alpha)) * 
                          (1.0 - exp_neg_2alpha_dt - 2.0 * alpha * dt * exp_neg_alpha_dt);
        
        const double q22 = sigma_a2 * (1.0 / (2.0 * alpha * alpha * alpha)) * 
                          (4.0 * exp_neg_alpha_dt - 3.0 - exp_neg_2alpha_dt + 2.0 * alpha * dt);
        
        const double q23 = sigma_a2 * (1.0 / (2.0 * alpha * alpha)) * 
                          (exp_neg_2alpha_dt + 1.0 - 2.0 * exp_neg_alpha_dt);
        
        const double q33 = sigma_a2 * (1.0 / (2.0 * alpha)) * 
                          (1.0 - exp_neg_2alpha_dt);
        
        Q(idx, idx)     = q11;
        Q(idx, idx+1)   = q12;
        Q(idx, idx+2)   = q13;
        Q(idx+1, idx)   = q12;
        Q(idx+1, idx+1) = q22;
        Q(idx+1, idx+2) = q23;
        Q(idx+2, idx)   = q13;
        Q(idx+2, idx+1) = q23;
        Q(idx+2, idx+2) = q33;
    };

    // x, y, z
    fill_block(0, s2qxyz, 1.0);
    fill_block(3, s2qxyz, 1.0);
    fill_block(6, s2qxyz, 1.0);
    // yaw
    fill_block(9, s2qyaw, 1.0);

    // r1, r2 常值噪声
    Q(12,12) = s2qr * dt;
    Q(13,13) = s2qr * dt;

    return Q;
}

inline Eigen::MatrixXd measure_r(Eigen::VectorXd & z, double r_pose, double r_distance, double r_yaw) {
    Eigen::VectorXd r(z.size());
    for(int i = 0; i < (z.size() / 4); i++){
        r.segment(i * 4, 4) << r_pose, r_pose, r_distance * std::abs(z[i * 4 + 2]), r_yaw;
    }
    return r.asDiagonal();
}

} // namespace singer_model
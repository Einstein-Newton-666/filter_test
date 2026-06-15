#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <initializer_list>
#include <vector>

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
 *                    r1, r2, dz]   // 装甲板半径和高度差（常值）
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
        
        // r1, r2, dz 常值
        x_cur[12] = x_pre[12];
        x_cur[13] = x_pre[13];
        x_cur[14] = x_pre[14];
    }
    int size = 15; // 状态维度
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
        // x[9] 是 super 板的 yaw, x[14] 是 dz（两块装甲板的高度差）
        // 装甲板位置：z = x[6] + I % 2 * dz (装甲板0用x[6]，装甲板1用x[6] + dz)
        const T xyz_armor = { 
            x[0] - ceres::cos(x[9] + M_PI_2 * I) * x[12 + I % 2],
            x[3] - ceres::sin(x[9] + M_PI_2 * I) * x[12 + I % 2],
            I % 2 == 0 ? x[6] : x[6] + x[14]
        };

        // 观测量：yaw, pitch, distance, yaw(orient)
        z[0] = ceres::atan2(xyz_armor[1], xyz_armor[0]); // yaw
        z[1] = ceres::atan2(xyz_armor[2], ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1])); // pitch
        z[2] = ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1] + xyz_armor[2] * xyz_armor[2]); // distance
        z[3] = x[9] + M_PI_2 * I;
    }
    int input_size = 15;   // 状态维度
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
            // x[9] 是 super 板的 yaw, x[14] 是 dz（两块装甲板的高度差）
            // 装甲板位置：z = x[6] + k % 2 * dz (装甲板0用x[6]，装甲板1用x[6] + dz)
            T xyz_armor = { 
                x[0] - ceres::cos(x[9] + M_PI_2 * k) * x[12 + k % 2],
                x[3] - ceres::sin(x[9] + M_PI_2 * k) * x[12 + k % 2],
                k % 2 == 0 ? x[6] : x[6] + x[14]
            };
            
            z[idx + 0] = ceres::atan2(xyz_armor[1], xyz_armor[0]); // yaw
            z[idx + 1] = ceres::atan2(xyz_armor[2], ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1])); // pitch
            z[idx + 2] = ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1] + xyz_armor[2] * xyz_armor[2]); // distance
            z[idx + 3] = x[9] + M_PI_2 * k;
            idx += 4;
        }
    }
    int input_size = 15;   // 状态维度
    int output_size = 8;   // 观测维度
    int I, J; // 匹配到的装甲板索引
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
            const auto armor_yaw = x[9] + static_cast<double>(i) * 2.0 * M_PI / 3.0;
            const auto zero = x[0] * 0.0;
            const auto dz = i == 0 ? zero : (i == 1 ? x[13] : x[14]);
            const auto armor_x = x[0] - ceres::cos(armor_yaw) * x[12];
            const auto armor_y = x[3] - ceres::sin(armor_yaw) * x[12];
            const auto armor_z = x[6] + dz;
            z[out + 0] = ceres::atan2(armor_y, armor_x);
            z[out + 1] = ceres::atan2(
                armor_z, ceres::sqrt(armor_x * armor_x + armor_y * armor_y));
            z[out + 2] = ceres::sqrt(
                armor_x * armor_x + armor_y * armor_y + armor_z * armor_z);
            z[out + 3] = armor_yaw;
            out += 4;
        }
    }

    int input_size = 15;
    int output_size = 0;

private:
    std::vector<int> indices_;
};

inline Eigen::MatrixXd predict_q(double dt, double s2qxy, double s2qz, double s2qyaw, double s2qr, double s2qdz, double tau) {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(15, 15);

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
    fill_block(0, s2qxy, tau);
    fill_block(3, s2qxy, tau);
    fill_block(6, s2qz, tau);
    // yaw
    fill_block(9, s2qyaw, tau);

    // r1, r2, dz 常值噪声
    Q(12,12) = s2qr * pow(dt, 4) / 4;
    Q(13,13) = s2qr * pow(dt, 4) / 4;
    Q(14,14) = s2qdz * pow(dt, 4) / 4;

    return Q;
}

inline Eigen::MatrixXd predict_outpost_q(
    double dt, double s2qxy, double s2qz, double s2qyaw,
    double s2qr, double s2qdz, double tau) {
    auto Q = predict_q(dt, s2qxy, s2qz, s2qyaw, s2qr, s2qdz, tau);
    const double dz_q = s2qdz * std::pow(dt, 4) / 4.0;
    Q(13, 13) = dz_q;
    Q(14, 14) = dz_q;
    return Q;
}

inline Eigen::MatrixXd measure_r(
    Eigen::VectorXd& z,
    double r_pose,
    double r_distance,
    double r_yaw,
    const std::vector<double>& abs_yaws = std::vector<double>{0.0},
    bool use_fixed_r = false) noexcept
{
    Eigen::VectorXd r(z.size());
    for(int i = 0; i < (z.size() / 4); i++){
        const auto obs_index = static_cast<std::size_t>(i);
        double abs_yaw = (abs_yaws.size() > obs_index) ? abs_yaws[obs_index] : 0.0;
        double d2r = abs_yaw * M_PI / 180.0;
        if (use_fixed_r) {
            // 固定R模式：仿真数据无PnP误差
            r.segment(i * 4, 4) << r_pose, r_pose, r_distance, r_yaw;
        } else {
            // 距离相关R模式：检测器数据有PnP误差
            r.segment(i * 4, 4) << r_pose, 
                                    r_pose, 
                                    r_distance * pow(z[i * 4 + 2], 2) * (pow(abs(d2r), 2) + 1), 
                                    log(abs(d2r) + 1) * 0.01 + r_yaw;
        }
    }
    return r.asDiagonal();
}

} // namespace singer_model

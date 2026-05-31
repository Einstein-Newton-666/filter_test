#pragma once

#include <Eigen/Dense>
#include <random>
#include <cmath>

namespace armor_sim {

struct DetectionNoiseParams {
    // 像素噪声 (U 形模型: σ(d) = σ_min + k·(d - d_opt)²)
    double pixel_noise_optimal = 1.5;             // 最优距离的最小噪声 (px)
    double pixel_noise_optimal_distance = 5.0;    // 最优距离 (m)，边缘 3-5px 最锐
    double pixel_noise_curvature = 0.125;         // 曲率 k
    //  物理: 近处梯度分散→σ↑, 最优处(3-5px边)σ最小, 远处锯齿→σ↑
    //  2m→2.6px  3m→2.0px  5m→1.5px(最优)  8m→2.6px  12m→7.6px

    // 离群点
    bool use_outliers = true;                // 是否模拟离群角点
    double outlier_probability = 0.05;       // 每个角点的离群概率
    double outlier_std = 10.0;               // 离群点噪声标准差 (px)

    // 检测概率
    double min_detectable_area = 100.0;
    double max_detectable_distance = 8.0;
    double detection_probability = 0.95;
    double miss_probability = 0.05;
};

/**
 * 检测噪声模型
 *
 * 唯一噪声源：像素角点噪声。
 * 3D 位姿通过 PnP 从带噪像素求解，噪声自然传播，保证一致性。
 */
class DetectionNoise {
public:
    DetectionNoise(const DetectionNoiseParams& params, uint32_t seed = 42)
        : params_(params),
          rng_(seed),
          uniform_dist_(0.0, 1.0) {}

    /**
     * 添加像素噪声 (U 形距离模型)
     *
     * 物理依据：角点检测精度由边缘梯度质量决定。
     *
     *   近距离 (d < d_opt): 装甲板图像大，边缘跨度多像素 → 梯度分散 → σ↑
     *   最优距离 (d ≈ d_opt): 边缘 3-5px 宽 → 梯度最集中 → σ 最小
     *   远距离 (d > d_opt): 边缘 <2px → 锯齿 (aliasing) → σ↑
     *
     * 模型: σ(d) = σ_min + k · (d - d_opt)²
     *
     *   示例 (σ_min=1.5, d_opt=5m, k=0.125):
     *     2m→2.6px  3m→2.0px  5m→1.5px  8m→2.6px  12m→7.6px
     *
     * 注意：3D 位姿误差的距离缩放由 PnP 传播自动完成，
     *       无需在像素层面重复建模。
     */
    Eigen::Vector2d addPixelNoise(const Eigen::Vector2d& pixel, double distance) {
        double dd = distance - params_.pixel_noise_optimal_distance;
        double sigma = params_.pixel_noise_optimal + params_.pixel_noise_curvature * dd * dd;
        // 下限保护，防止数值问题
        if (sigma < 0.5) sigma = 0.5;

        // 离群点：偶尔角点严重偏离 (模拟反光、遮挡、误匹配)
        if (params_.use_outliers && uniform_dist_(rng_) < params_.outlier_probability) {
            sigma = params_.outlier_std;
        }

        double noise_u = std::normal_distribution<double>(0.0, sigma)(rng_);
        double noise_v = std::normal_distribution<double>(0.0, sigma)(rng_);
        return Eigen::Vector2d(pixel.x() + noise_u, pixel.y() + noise_v);
    }

    /**
     * 判断是否检测到
     */
    bool shouldDetect(double distance, double area) {
        if (uniform_dist_(rng_) < params_.miss_probability) {
            return false;
        }

        double k_area = 0.01;
        double k_dist = 1.0;
        double area_factor = sigmoid(k_area * (area - params_.min_detectable_area));
        double dist_factor = sigmoid(k_dist * (params_.max_detectable_distance - distance));
        double prob = params_.detection_probability * area_factor * dist_factor;

        return uniform_dist_(rng_) < prob;
    }

private:
    DetectionNoiseParams params_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_dist_;

    static double sigmoid(double x) {
        return 1.0 / (1.0 + std::exp(-x));
    }
};

}  // namespace armor_sim

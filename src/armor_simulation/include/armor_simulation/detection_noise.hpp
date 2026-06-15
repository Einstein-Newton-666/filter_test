#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <random>
#include <cmath>

namespace armor_sim {

struct DetectionNoiseParams {
    // 像素噪声 (U 形模型: σ(d) = σ_min + k·(d - d_opt)²)
    double pixel_noise_optimal = 1.5;             // 最优距离的最小噪声 (px)
    double pixel_noise_optimal_distance = 5.0;    // 最优距离 (m)，边缘 3-5px 最锐
    double pixel_noise_curvature = 0.125;         // 曲率 k
    double pixel_noise_common_ratio = 0.7;        // 装甲板整体平移噪声占比 ρ, 降低非刚体角点抖动
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
        double sigma = pixelNoiseSigma(distance);
        double noise_u = std::normal_distribution<double>(0.0, sigma)(rng_);
        double noise_v = std::normal_distribution<double>(0.0, sigma)(rng_);
        return Eigen::Vector2d(pixel.x() + noise_u, pixel.y() + noise_v);
    }

    template<size_t N>
    std::array<Eigen::Vector2d, N> addCorrelatedPixelNoise(
        const std::array<Eigen::Vector2d, N>& pixels, double distance) {
        double sigma = pixelNoiseSigma(distance);
        double common_ratio = std::clamp(params_.pixel_noise_common_ratio, 0.0, 1.0);
        // 目标级相关噪声分解:
        //   p_i' = p_i + n_c + n_i
        //   n_c ~ N(0, (ρσ)^2 I),  n_i ~ N(0, ((sqrt(1-ρ²))σ)^2 I)
        // 非离群点满足 Var(n_c+n_i)=σ²I, 单点总方差不变；ρ 越大,
        // 点集共同平移成分越多, 越少破坏目标局部几何约束。
        double common_sigma = sigma * common_ratio;
        double corner_sigma = sigma * std::sqrt(std::max(0.0, 1.0 - common_ratio * common_ratio));

        Eigen::Vector2d common_noise(
            std::normal_distribution<double>(0.0, common_sigma)(rng_),
            std::normal_distribution<double>(0.0, common_sigma)(rng_));

        std::array<Eigen::Vector2d, N> noisy = pixels;
        for (auto& pixel : noisy) {
            double sigma_i = corner_sigma;
            if (params_.use_outliers && uniform_dist_(rng_) < params_.outlier_probability) {
                sigma_i = params_.outlier_std;
            }
            pixel += common_noise + Eigen::Vector2d(
                std::normal_distribution<double>(0.0, sigma_i)(rng_),
                std::normal_distribution<double>(0.0, sigma_i)(rng_));
        }
        return noisy;
    }

    std::array<Eigen::Vector2d, 4> addArmorPixelNoise(
        const std::array<Eigen::Vector2d, 4>& pixels, double distance) {
        return addCorrelatedPixelNoise(pixels, distance);
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

    double pixelNoiseSigma(double distance) const {
        double dd = distance - params_.pixel_noise_optimal_distance;
        double sigma = params_.pixel_noise_optimal + params_.pixel_noise_curvature * dd * dd;
        return std::max(0.5, sigma);
    }

    static double sigmoid(double x) {
        return 1.0 / (1.0 + std::exp(-x));
    }
};

}  // namespace armor_sim

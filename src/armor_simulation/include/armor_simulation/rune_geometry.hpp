#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

namespace armor_sim {

// 能量机关几何约定：
// - 局部坐标原点是 R 标中心。
// - 扇叶点全部位于局部 X=0 平面；局部 +X 是符面法线/旋转轴方向。
// - roll=0 时，击打点在局部 +Z 方向，距离 R 标中心 0.7 m。
// - 五片扇叶间隔 72 deg，因此第 i 片扇叶 roll = base_roll + i * RUNE_SLOT_ANGLE。
inline constexpr double RUNE_BLADE_RADIUS = 0.700;
inline constexpr double RUNE_SLOT_ANGLE = 2.0 * M_PI / 5.0;

enum class RunePointIndex {
    RCenter = 0,
    Near = 1,
    Left = 2,
    Far = 3,
    Right = 4,
};

inline std::array<Eigen::Vector3d, 5> runeBladeLocalPoints() {
    // 点顺序必须和 RuneTarget.msg 保持一致：
    // r_center, near_point, left_point, far_point, right_point。
    // 这里沿用参考工程的五点语义。near/far 是目标框在半径方向的近端/远端，
    // left/right 是击打中心两侧的横向点；击打中心单独取半径 RUNE_BLADE_RADIUS。
    return {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 0.5415),
        Eigen::Vector3d(0.0, 0.160, RUNE_BLADE_RADIUS),
        Eigen::Vector3d(0.0, 0.0, 0.8585),
        Eigen::Vector3d(0.0, -0.160, RUNE_BLADE_RADIUS),
    };
}

inline Eigen::Matrix3d runeBaseRotation(double normal_yaw, double normal_pitch) {
    // 基础姿态只描述符面朝向，不包含扇叶自旋 roll：
    // 1. normal_pitch 绕局部/中间 Y 轴俯仰符面；
    // 2. normal_yaw 绕 odom Z 轴改变符面水平朝向；
    // 3. 扇叶 roll 在 computeRuneBladeWorldPoints() 中继续绕局部 X 轴叠加。
    //
    // 当前图优化沿用参考项目的工程假设：normal_pitch 是固定参数，不作为状态估计。
    return (Eigen::AngleAxisd(normal_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(normal_pitch, Eigen::Vector3d::UnitY()))
        .toRotationMatrix();
}

inline std::array<Eigen::Vector3d, 5> computeRuneBladeWorldPoints(
    const Eigen::Vector3d& center,
    double normal_yaw,
    double normal_pitch,
    double roll) {
    // 世界点 = R 标中心 + R_base(normal_yaw, normal_pitch) * Rx(roll) * 局部点。
    // Rx(roll) 表示扇叶绕符面法线旋转；normal_yaw/pitch 表示整个符面朝向。
    const Eigen::Matrix3d R =
        runeBaseRotation(normal_yaw, normal_pitch) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const auto local_points = runeBladeLocalPoints();
    std::array<Eigen::Vector3d, 5> world_points;
    for (size_t i = 0; i < local_points.size(); ++i) {
        world_points[i] = center + R * local_points[i];
    }
    return world_points;
}

inline Eigen::Vector3d computeRuneBladeTargetPosition(
    const Eigen::Vector3d& center,
    double normal_yaw,
    double normal_pitch,
    double roll,
    int blade_index) {
    // 击打点使用同一个定义：选中物理扇叶的击打中心。
    // clamp 让自动选择逻辑的兜底值始终落在合法 slot 内。
    const int blade = std::clamp(blade_index, 0, 4);
    const double blade_roll = roll + static_cast<double>(blade) * RUNE_SLOT_ANGLE;
    const Eigen::Matrix3d R =
        runeBaseRotation(normal_yaw, normal_pitch) *
        Eigen::AngleAxisd(blade_roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    return center + R * Eigen::Vector3d(0.0, 0.0, RUNE_BLADE_RADIUS);
}

inline int selectRuneTargetBlade(
    const std::vector<int32_t>& observed_blades,
    const std::vector<int32_t>& active_blades) {
    // 真值云台和输出 target 优先跟随 detector 实际观测到的叶片；
    // 没观测时退到当前 active blade，仍没有时用 slot 0 保持输出稳定。
    const auto pick = [](const std::vector<int32_t>& blades) {
        return blades.empty() ? 0 : std::clamp(static_cast<int>(blades.front()), 0, 4);
    };
    return observed_blades.empty() ? pick(active_blades) : pick(observed_blades);
}

struct BigRuneVelocityConfig {
    // 大符角速度曲线：
    // v(t) = direction * abs(bias + amplitude * sin(frequency * t + phase) + noise)
    // step(dt) 对 v(t) 做欧拉积分得到连续 roll。范围参数按 RoboMaster
    // 常见大符速度区间设置，random_seed 用于复现实验。
    int random_seed = 42;
    int direction = 1;
    std::array<double, 2> velocity_bias_range{1.045, 1.310};
    std::array<double, 2> velocity_amplitude_range{0.780, 1.045};
    std::array<double, 2> velocity_frequency_range{1.884, 2.000};
    double velocity_noise_std = 0.03;
    double resample_interval = 0.0;
    double min_abs_velocity = 0.1;
    double max_abs_velocity = 4.0;
};

struct BigRuneVelocityParams {
    double bias = 1.178;
    double amplitude = 0.9125;
    double frequency = 1.942;
    double phase = 0.0;
};

class BigRuneMotionModel {
public:
    explicit BigRuneMotionModel(BigRuneVelocityConfig config)
        : config_(config), rng_(static_cast<uint32_t>(config.random_seed)) {
        sampleParams();
    }

    double sampleVelocity(double dt) {
        // dt 只推进内部时间，不直接改变 roll；这样测试可以单独验证速度曲线。
        elapsed_ += std::max(0.0, dt);
        if (config_.resample_interval > 0.0 &&
            elapsed_ - last_resample_time_ >= config_.resample_interval) {
            sampleParams();
            last_resample_time_ = elapsed_;
        }

        const double sign = config_.direction >= 0 ? 1.0 : -1.0;
        const double noise = std::normal_distribution<double>(
            0.0, std::max(0.0, config_.velocity_noise_std))(rng_);
        double abs_velocity = params_.bias +
            params_.amplitude * std::sin(params_.frequency * elapsed_ + params_.phase) +
            noise;
        abs_velocity = std::clamp(std::abs(abs_velocity),
                                  config_.min_abs_velocity,
                                  config_.max_abs_velocity);
        last_velocity_ = sign * abs_velocity;
        return last_velocity_;
    }

    double step(double dt) {
        // 仿真器使用一阶积分，和真实 detector 输出的逐帧角度观测形态一致：
        // 每帧只有当前 roll，而不是解析曲线参数。
        roll_ += sampleVelocity(dt) * dt;
        return roll_;
    }

    double roll() const { return roll_; }
    double velocity() const { return last_velocity_; }
    const BigRuneVelocityParams& params() const { return params_; }

private:
    static double sampleUniform(std::mt19937& rng, std::array<double, 2> range) {
        if (range[0] > range[1]) std::swap(range[0], range[1]);
        return std::uniform_real_distribution<double>(range[0], range[1])(rng);
    }

    void sampleParams() {
        params_.bias = sampleUniform(rng_, config_.velocity_bias_range);
        params_.amplitude = sampleUniform(rng_, config_.velocity_amplitude_range);
        params_.frequency = sampleUniform(rng_, config_.velocity_frequency_range);
        params_.phase = std::uniform_real_distribution<double>(-M_PI, M_PI)(rng_);
    }

    BigRuneVelocityConfig config_;
    std::mt19937 rng_;
    BigRuneVelocityParams params_;
    double elapsed_ = 0.0;
    double last_resample_time_ = 0.0;
    double roll_ = 0.0;
    double last_velocity_ = 0.0;
};

class RuneActiveBladeSelector {
public:
    RuneActiveBladeSelector(int blade_count, double switch_interval, int random_seed)
        : blade_count_(std::max(1, blade_count)),
          switch_interval_(std::max(0.0, switch_interval)),
          rng_(static_cast<uint32_t>(random_seed)) {
        active_.fill(false);
    }

    void update(double time_seconds, int active_count) {
        const int count = std::clamp(active_count, 1, blade_count_);
        if (!initialized_ || (switch_interval_ > 0.0 &&
            time_seconds - last_switch_time_ >= switch_interval_)) {
            resample(count);
            last_switch_time_ = time_seconds;
            initialized_ = true;
        }
    }

    bool isActive(int blade_index) const {
        if (blade_index < 0 || blade_index >= blade_count_) return false;
        return active_[static_cast<size_t>(blade_index)];
    }

    int activeCount() const {
        return static_cast<int>(std::count(active_.begin(), active_.end(), true));
    }

    std::array<bool, 5> activeMask() const { return active_; }

private:
    void resample(int active_count) {
        active_.fill(false);
        std::array<int, 5> indices{0, 1, 2, 3, 4};
        std::shuffle(indices.begin(), indices.begin() + blade_count_, rng_);
        for (int i = 0; i < active_count; ++i) {
            active_[static_cast<size_t>(indices[static_cast<size_t>(i)])] = true;
        }
    }

    int blade_count_ = 5;
    double switch_interval_ = 3.0;
    double last_switch_time_ = 0.0;
    bool initialized_ = false;
    std::mt19937 rng_;
    std::array<bool, 5> active_{};
};

struct OutpostGeometryConfig {
    // 前哨站三装甲板几何。dz_1/dz_2 是相对中心高度偏移；
    // armor_pitch 是装甲板自身俯仰，和车身 body_roll/body_pitch 分开处理。
    double radius = 0.2765;
    double dz_1 = 0.2;
    double dz_2 = -0.1;
    double armor_pitch = -0.26;
};

struct OutpostMotionConfig {
    // 前哨站不走普通机器人平移/加速度模型：
    // 中心位置固定，yaw 使用规则固定角速度方程 yaw(t)=yaw0+omega*t。
    Eigen::Vector3d center = Eigen::Vector3d(0.0, -4.0, 0.0);
    double initial_yaw = 0.0;
    double angular_velocity = 0.8 * M_PI;
};

struct OutpostMotionState {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double angular_velocity = 0.0;
};

struct OutpostModelConfig {
    Eigen::Vector3d center = Eigen::Vector3d(0.0, -4.0, 0.0);
    double body_roll = 0.0;
    double body_pitch = 0.0;
    OutpostGeometryConfig geometry;
    double initial_yaw = 0.0;
    double angular_velocity = 0.8 * M_PI;
};

inline OutpostMotionConfig outpostMotionConfig(const OutpostModelConfig& config) {
    OutpostMotionConfig motion;
    motion.center = config.center;
    motion.initial_yaw = config.initial_yaw;
    motion.angular_velocity = config.angular_velocity;
    return motion;
}

inline Eigen::Matrix3d outpostBodyRotation(const OutpostModelConfig& config) {
    return (Eigen::AngleAxisd(config.body_roll, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(config.body_pitch, Eigen::Vector3d::UnitY()))
        .toRotationMatrix();
}

inline OutpostMotionState computeOutpostMotionState(
    const OutpostMotionConfig& config,
    double elapsed_time) {
    const double t = std::max(0.0, elapsed_time);
    OutpostMotionState state;
    state.center = config.center;
    state.yaw = config.initial_yaw + config.angular_velocity * t;
    state.angular_velocity = config.angular_velocity;
    return state;
}

struct OutpostPlate {
    int index = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    double yaw = 0.0;
};

inline std::array<OutpostPlate, 3> computeOutpostPlates(
    const Eigen::Vector3d& center,
    double yaw,
    const Eigen::Matrix3d& body_rotation,
    const OutpostGeometryConfig& config) {
    // 三块装甲板均匀分布在圆周上。先在车体局部系中计算圆周位置，
    // 再乘 body_rotation，把整体车身 roll/pitch 倾斜统一作用到位置和姿态。
    std::array<OutpostPlate, 3> plates;
    const std::array<double, 3> dz{0.0, config.dz_1, config.dz_2};
    for (int i = 0; i < 3; ++i) {
        const double armor_yaw = yaw + static_cast<double>(i) * 2.0 * M_PI / 3.0;
        const Eigen::Vector3d local(
            -config.radius * std::cos(armor_yaw),
            -config.radius * std::sin(armor_yaw),
            dz[static_cast<size_t>(i)]);
        const Eigen::Matrix3d armor_rotation =
            body_rotation *
            (Eigen::AngleAxisd(armor_yaw, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(config.armor_pitch, Eigen::Vector3d::UnitY()))
                .toRotationMatrix();
        plates[static_cast<size_t>(i)].index = i;
        plates[static_cast<size_t>(i)].position = center + body_rotation * local;
        plates[static_cast<size_t>(i)].orientation = Eigen::Quaterniond(armor_rotation);
        plates[static_cast<size_t>(i)].yaw = armor_yaw;
    }
    return plates;
}

}  // namespace armor_sim

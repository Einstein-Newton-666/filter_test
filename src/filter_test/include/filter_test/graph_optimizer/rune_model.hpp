#pragma once

#include "filter_test/graph_optimizer/graph_core.hpp"
#include "filter_test/graph_optimizer/graph_math.hpp"

#include <auto_aim_interfaces/msg/rune_target.hpp>
#include <auto_aim_interfaces/msg/rune_targets.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <array>
#include <cstdint>
#include <vector>

namespace filter_test::graph_optimizer {

inline constexpr double kRuneBladeRadius = 0.700;
inline constexpr double kRuneSlotAngle = 2.0 * M_PI / 5.0;

inline std::array<Eigen::Vector3d, 5> runeBladeLocalPoints() {
    // 与 armor_simulation::runeBladeLocalPoints() 保持完全一致。
    // 图优化的重投影因子直接用这些局部点，把 RuneTarget 的五个像素观测
    // 约束到同一个辅助 Pose3 上。
    return {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 0.5415),
        Eigen::Vector3d(0.0, 0.160, kRuneBladeRadius),
        Eigen::Vector3d(0.0, 0.0, 0.8585),
        Eigen::Vector3d(0.0, -0.160, kRuneBladeRadius),
    };
}

struct RuneTrackerConfig {
    auto_graph::GraphOptimizerConfig optimizer;
    // initial_center/normal_yaw/normal_pitch 是冷启动先验。
    // 当前模型只优化 center、roll、vroll 和 normal_yaw；normal_pitch 仍是固定参数。
    Eigen::Vector3d initial_center = Eigen::Vector3d(0.0, -5.0, 1.5);
    double normal_yaw = M_PI_2;
    double normal_pitch = 0.0;
    double pixel_sigma = 2.0;
    double center_sigma = 0.05;
    double roll_sigma = 0.05;
    double vroll_sigma = 0.10;
    double pose_prior_sigma = 0.20;
    double pnp_pose_prior_sigma = 0.08;
    bool use_direct_reproj_factor = true;
    double direct_reproj_sigma = 0.02;
    double normal_yaw_sigma = 0.05;
    double match_max_roll_diff = 0.523599;
    double match_max_center_distance = 1.0;
    bool use_direct_reproj_match_fallback = true;
    double match_max_direct_reproj_error = 0.20;
    double predict_dt = 0.15;
    Eigen::Matrix3d camera_matrix = Eigen::Matrix3d::Identity();
    std::array<double, 5> distortion = {0.0, 0.0, 0.0, 0.0, 0.0};
};

Eigen::Matrix3d runeBladeRotation(double normal_yaw, double normal_pitch, double roll);

Eigen::Vector3d runeTargetPosition(const Eigen::Vector3d& center,
                                   double normal_yaw,
                                   double normal_pitch,
                                   double roll);

std::array<Eigen::Vector2d, 5> projectRuneBladePixels(
    const Eigen::Vector3d& center,
    double normal_yaw,
    double normal_pitch,
    double roll,
    const Eigen::Isometry3d& T_camera_to_odom,
    const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist);

struct RunePnPResult {
    bool success = false;
    gtsam::Pose3 pose_camera;
};

RunePnPResult estimateRuneBladePosePnP(
    const std::array<Eigen::Vector2d, 5>& pixels,
    const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist);

double runeBigCurveAngle(double time, const std::array<double, 5>& params);
double runeBigCurveVelocity(double time, const std::array<double, 5>& params);

struct RuneBigCurveFitterConfig {
    int min_data_size = 100;
    int max_data_size = 600;
    int max_iterations = 30;
};

class RuneBigCurveFitter {
public:
    explicit RuneBigCurveFitter(RuneBigCurveFitterConfig config = {});

    void clear();
    void setC(double c);
    void addSample(double time, double roll);
    bool fitOnce();
    void fit();

    bool hasFit() const { return has_fit_; }
    size_t sampleCount() const { return samples_.size(); }
    std::array<double, 5> params() const { return params_; }
    double angleAt(double time) const;
    double velocityAt(double time) const;

private:
    struct Sample {
        double time = 0.0;
        double roll = 0.0;
    };

    RuneBigCurveFitterConfig config_;
    std::vector<Sample> samples_;
    std::array<double, 5> params_{0.470, 1.942, 0.0, 1.178, 0.0};
    bool has_fit_ = false;
};

struct RuneTrackerState {
    // center: R 标中心在 odom 下的位置。
    // normal_yaw: 符面法线在水平面的朝向。
    // roll/vroll: 第 0 片扇叶的连续角度和角速度；第 i 片扇叶相差 i*72 deg。
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double normal_yaw = 0.0;
    double roll = 0.0;
    double vroll = 0.0;
};

struct RuneGraphOutput {
    auto_graph::SolveResult solve_result;
    RuneTrackerState state;
    Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
    int observed_count = 0;
    std::vector<int> matched_blade_indices;
    std::vector<Eigen::Isometry3d> observed_pnp_poses;
    std::vector<Eigen::Vector3d> observed_pnp_target_positions;
};

class RuneRollFactor
    : public gtsam::NoiseModelFactorN<gtsam::Rot2, double, gtsam::Rot2> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Rot2, double, gtsam::Rot2>;

    RuneRollFactor(const gtsam::SharedNoiseModel& noise, gtsam::Key roll_prev,
                   gtsam::Key vroll_prev, gtsam::Key roll_cur, double dt);

    gtsam::Vector evaluateError(
        const gtsam::Rot2& roll_prev, const double& vroll_prev,
        const gtsam::Rot2& roll_cur, gtsam::OptionalMatrixType H1,
        gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const override;

private:
    double dt_ = 0.0;
};

class RuneVrollFactor : public gtsam::NoiseModelFactorN<double, double> {
public:
    using Base = gtsam::NoiseModelFactorN<double, double>;

    RuneVrollFactor(const gtsam::SharedNoiseModel& noise, gtsam::Key prev, gtsam::Key cur);

    gtsam::Vector evaluateError(
        const double& prev, const double& cur,
        gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const override;
};

class RuneBladeReprojFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

    RuneBladeReprojFactor(const gtsam::SharedNoiseModel& noise,
                          gtsam::Key blade_pose_key,
                          const Eigen::Vector3d& point_local,
                          const Eigen::Matrix3d& K,
                          const std::array<double, 5>& dist,
                          const Eigen::Vector2d& observed_pixel);

    gtsam::Vector evaluateError(
        const gtsam::Pose3& blade_pose_camera,
        gtsam::OptionalMatrixType H) const override;

private:
    // 单个 3D 局部点投影到图像后，与 detector/仿真器提供的 2D 像素做残差。
    // 这个因子只连接辅助 blade_pose，不直接连接 center/roll。
    gtsam::Point3 point_local_;
    gtsam::Cal3DS2 calib_;
    gtsam::Point2 observed_;
};

class RuneBladeDirectReprojFactor
    : public gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Rot2, gtsam::Rot2> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Rot2, gtsam::Rot2>;

    RuneBladeDirectReprojFactor(const gtsam::SharedNoiseModel& noise,
                                gtsam::Key center_key,
                                gtsam::Key roll_key,
                                gtsam::Key normal_yaw_key,
                                const Eigen::Isometry3d& T_camera_to_odom,
                                int blade_index,
                                double normal_pitch,
                                const Eigen::Matrix3d& K,
                                const std::array<double, 5>& dist,
                                const std::array<Eigen::Vector2d, 5>& observed_pixels);

    gtsam::Vector evaluateError(
        const gtsam::Point3& center,
        const gtsam::Rot2& roll,
        const gtsam::Rot2& normal_yaw,
        gtsam::OptionalMatrixType H1,
        gtsam::OptionalMatrixType H2,
        gtsam::OptionalMatrixType H3) const override;

private:
    gtsam::Vector computeError(
        const gtsam::Point3& center,
        const gtsam::Rot2& roll,
        const gtsam::Rot2& normal_yaw) const;

    Eigen::Isometry3d T_camera_to_odom_;
    int blade_index_ = 0;
    double normal_pitch_ = 0.0;
    Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();
    std::array<double, 5> dist_{};
    std::array<Eigen::Vector2d, 5> observed_pixels_{};
};

class RuneBladeRollDirectReprojFactor : public gtsam::NoiseModelFactorN<gtsam::Rot2> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Rot2>;

    RuneBladeRollDirectReprojFactor(const gtsam::SharedNoiseModel& noise,
                                    gtsam::Key roll_key,
                                    const Eigen::Vector3d& center,
                                    double normal_yaw,
                                    const Eigen::Isometry3d& T_camera_to_odom,
                                    int blade_index,
                                    double normal_pitch,
                                    const Eigen::Matrix3d& K,
                                    const std::array<double, 5>& dist,
                                    const std::array<Eigen::Vector2d, 5>& observed_pixels);

    gtsam::Vector evaluateError(
        const gtsam::Rot2& roll,
        gtsam::OptionalMatrixType H) const override;

private:
    gtsam::Vector computeError(const gtsam::Rot2& roll) const;

    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    double normal_yaw_ = 0.0;
    Eigen::Isometry3d T_camera_to_odom_;
    int blade_index_ = 0;
    double normal_pitch_ = 0.0;
    Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();
    std::array<double, 5> dist_{};
    std::array<Eigen::Vector2d, 5> observed_pixels_{};
};

class RuneBladeGeometryFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3,
                                      gtsam::Rot2, gtsam::Rot2> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3,
                                          gtsam::Rot2, gtsam::Rot2>;

    RuneBladeGeometryFactor(const gtsam::SharedNoiseModel& noise,
                            gtsam::Key blade_pose_key,
                            gtsam::Key center_key,
                            gtsam::Key roll_key,
                            gtsam::Key normal_yaw_key,
                            const Eigen::Isometry3d& T_camera_to_odom,
                            int blade_index,
                            double normal_pitch);

    gtsam::Vector evaluateError(
        const gtsam::Pose3& blade_pose_camera,
        const gtsam::Point3& center,
        const gtsam::Rot2& roll,
        const gtsam::Rot2& normal_yaw,
        gtsam::OptionalMatrixType H1,
        gtsam::OptionalMatrixType H2,
        gtsam::OptionalMatrixType H3,
        gtsam::OptionalMatrixType H4) const override;

private:
    gtsam::Vector computeError(const gtsam::Pose3& blade_pose_camera,
                               const gtsam::Point3& center,
                               const gtsam::Rot2& roll,
                               const gtsam::Rot2& normal_yaw) const;

    // 把辅助 blade_pose(camera) 转到 odom 后，用 {x,y,z,roll,yaw} 残差
    // 连接到主状态 center/roll/normal_yaw。normal_pitch 固定来自配置。
    Eigen::Isometry3d T_camera_to_odom_;
    int blade_index_ = 0;
    double normal_pitch_ = 0.0;
};

class RuneCvGraph {
public:
    explicit RuneCvGraph(RuneTrackerConfig config);

    void initialize();
    RuneGraphOutput update(const auto_aim_interfaces::msg::RuneTargets& msg,
                           double dt,
                           const Eigen::Isometry3d& T_camera_to_odom);
    void reset();

    bool initialized() const { return initialized_; }
    uint64_t frameId() const;

private:
    struct Variables {
        // 动态变量每帧一个 key；normal_yaw 是静态变量，在整个滑窗中共享。
        // normal_pitch 不在图里声明，预测和输出时从 config 固定读取。
        auto_graph::Var<gtsam::Point3> center;
        auto_graph::Var<gtsam::Rot2> roll;
        auto_graph::Var<double> vroll;
        auto_graph::Var<gtsam::Rot2> normal_yaw;
    };

    static Variables declareVariables(auto_graph::GraphOptimizer& optimizer);

    void addMotionFactors(double dt);
    std::vector<int> addObservationFactors(
        const auto_aim_interfaces::msg::RuneTargets& msg,
        const Eigen::Isometry3d& T_camera_to_odom,
        bool had_previous_frame);
    RuneGraphOutput makeOutput(const auto_graph::SolveResult& solve_result,
                               std::vector<int> matched_blade_indices);
    gtsam::Pose3 predictedBladePoseCamera(int blade_index,
                                          const Eigen::Isometry3d& T_camera_to_odom) const;

    RuneTrackerConfig config_;
    RuneTrackerState state_;
    auto_graph::GraphOptimizer optimizer_;
    Variables vars_;
    bool initialized_ = false;
    std::vector<Eigen::Isometry3d> last_observed_pnp_poses_;
    std::vector<Eigen::Vector3d> last_observed_pnp_target_positions_;
};

}  // namespace filter_test::graph_optimizer

#pragma once

#include "filter_test/graph_optimizer/graph_core.hpp"
#include "filter_test/graph_optimizer/graph_math.hpp"

#include <auto_aim_interfaces/msg/armor.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace filter_test::graph_optimizer {

// 几何因子残差标准差，顺序对应 tangential/radial/height/yaw。
struct GeoNoise {
    double tangential = 0.01;
    double radial = 0.03;
    double height = 0.01;
    double yaw = 0.005;
};

// 静态几何参数的先验标准差：半径使用 logistic 无界状态，dz 使用米。
struct PriorNoise {
    double radius = 1.0;
    double dz = 1.0;
};

inline constexpr double kDefaultArmorMatchMaxCost = 1.5;
inline constexpr double kDefaultOutpostAmbiguousMatchMargin = 0.15;
inline constexpr double kRadiusMin = 0.10;
inline constexpr double kRadiusMax = 0.60;

// ROS node 解析后的 graph tracker 配置。这里保留模型噪声、像素噪声和相机参数，
// 求解器相关参数则嵌套在 optimizer 中。
struct TrackerConfig {
    double s2qxy = 0.1;
    double s2qz = 0.1;
    double s2qyaw = 0.1;

    double vel_sigma = 0.01;
    double vyaw_sigma = 0.05;
    double outpost_s2qxy = 0.1;
    double outpost_s2qz = 0.0004;
    double outpost_s2qyaw = 0.1;
    double outpost_vel_sigma = 0.01;
    double outpost_vyaw_sigma = 0.05;

    double match_max_cost = kDefaultArmorMatchMaxCost;
    double outpost_ambiguous_match_margin = kDefaultOutpostAmbiguousMatchMargin;
    int match_quality_window_size = 10;
    double match_quality_failure_threshold = 1.0;
    double match_quality_failure_ratio = 0.60;
    double pixel_sigma = 1.0;
    double outpost_pixel_sigma = 2.0;
    double observation_noise_reference_distance = 5.0;
    double pixel_sigma_distance_quadratic = 0.025;
    double pose_prior_sigma = 0.1;
    double pose_prior_distance_scale = 0.05;
    bool use_edge_reproj_factor = true;
    double edge_reproj_sigma = 0.05;
    double edge_loss_slope_k = 2.0;
    double standard_armor_pitch = 15.0 * M_PI / 180.0;
    double outpost_armor_pitch = -0.26;
    double outpost_radius = 0.2765;
    double outpost_initial_vyaw = 0.0;

    GeoNoise geo_noise;
    GeoNoise outpost_geo_noise{0.01, 0.03, 0.05, 0.10};
    double geo_tangential_distance_scale = 0.20;
    double geo_radial_distance_scale = 0.20;
    double geo_yaw_distance_scale = 0.10;
    PriorNoise prior_noise;
    PriorNoise outpost_prior_noise;

    Eigen::Matrix3d camera_matrix = Eigen::Matrix3d::Identity();
    std::array<double, 5> distortion = {0.0, 0.0, 0.0, 0.0, 0.0};

    auto_graph::GraphOptimizerConfig optimizer;
};

struct ObservationNoise {
    double pixel_sigma = 1.0;
    double pose_prior_sigma = 0.1;
    GeoNoise geo_noise;
};

// 当前拆分运动因子的标准差。center/yaw 对应积分残差，
// velocity/vyaw 对应随机游走残差。
struct MotionNoiseSigmas {
    double center_xy = 1e-6;
    double center_z = 1e-6;
    double yaw = 1e-6;
    double velocity_xy = 1e-6;
    double velocity_z = 1e-6;
    double vyaw = 1e-6;
};

ObservationNoise observationNoiseForArmor(
    const TrackerConfig& config,
    const auto_aim_interfaces::msg::Armor& armor);

ObservationNoise observationNoiseForArmor(
    const TrackerConfig& config,
    const auto_aim_interfaces::msg::Armor& armor,
    double distance);

MotionNoiseSigmas motionNoiseSigmasForConfig(
    const TrackerConfig& config,
    bool outpost_motion,
    double dt);

// typed armor graph 的主状态：
// center/velocity/yaw/vyaw 是动态变量，radius_1/radius_2/dz 是静态几何参数。
struct TrackerState {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double vyaw = 0.0;
    double radius_1 = 0.25;
    double radius_2 = 0.25;
    double dz = 0.0;
    double outpost_dz_2 = 0.0;
    Eigen::Vector2d outpost_base_xy = Eigen::Vector2d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    double outpost_base_z = std::numeric_limits<double>::quiet_NaN();
    int armor_count = 4;
};

// 根据主状态推导出的单块装甲板状态，只用于匹配和输出，不直接作为主优化状态。
struct PredictedArmor {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double radius = 0.0;
    double dz = 0.0;
    int index = 0;
};

// 半径物理值和无界优化状态之间的转换。
inline double radiusFromState(double radius_u) {
    return auto_graph::logisticFunction(radius_u, kRadiusMin, kRadiusMax);
}

inline double radiusToState(double radius) {
    return auto_graph::logisticInverse(radius, kRadiusMin, kRadiusMax);
}

Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R);
double armorYawOffset(int armor_index, int armor_count = 4);

// 从中心状态推导四块装甲板的预期位置/yaw。
std::vector<PredictedArmor> predictedArmorsFromState(const TrackerState& state);

// 按主状态直接预测单块装甲板四角点像素。局部坐标约定为
// X=法线, Y=宽度, Z=高度，角点顺序 [左下, 左上, 右上, 右下]。
std::array<Eigen::Vector2d, 4> projectArmorPixels(
    const Eigen::Vector3d& center,
    double center_yaw,
    double radius,
    double dz,
    int armor_index,
    double armor_pitch,
    const Eigen::Isometry3d& T_camera_to_odom,
    const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist,
    int armor_count = 4);

// 单观测匹配到最可能的物理装甲板 index。保留给测试和简单调用。
int matchArmorIndex(const TrackerState& state,
                    const auto_aim_interfaces::msg::Armor& armor,
                    int last_armor_index,
                    double max_match_cost = kDefaultArmorMatchMaxCost);

// 多观测匹配保证同一帧内 index 唯一，避免多块观测共享同一个 Pose3 key。
std::vector<int> matchArmorIndicesUnique(
    const TrackerState& state,
    const auto_aim_interfaces::msg::Armors& msg,
    int last_armor_index,
    double max_match_cost = kDefaultArmorMatchMaxCost,
    double outpost_ambiguous_match_margin = kDefaultOutpostAmbiguousMatchMargin);

std::pair<std::vector<int>, std::vector<double>> matchArmorIndicesUniqueWithCosts(
    const TrackerState& state,
    const auto_aim_interfaces::msg::Armors& msg,
    int last_armor_index,
    double max_match_cost = kDefaultArmorMatchMaxCost,
    double outpost_ambiguous_match_margin = kDefaultOutpostAmbiguousMatchMargin);

// 位置积分因子:
//   residual = X(k) - X(k-1) - V(k-1) * dt
// 将位置和速度拆成两个 typed 变量，图结构比旧 6D pos_vel Vector 更稀疏。
class TranslationFactor
    : public gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Vector3, gtsam::Point3> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Vector3, gtsam::Point3>;

    TranslationFactor(const gtsam::SharedNoiseModel& noise, gtsam::Key x_prev,
                      gtsam::Key v_prev, gtsam::Key x_cur, double dt);

    gtsam::Vector evaluateError(
        const gtsam::Point3& x_prev, const gtsam::Vector3& v_prev,
        const gtsam::Point3& x_cur, gtsam::OptionalMatrixType H1,
        gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const override;

private:
    double dt_ = 0.0;
};

// yaw 使用 Rot2 流形变量，残差通过 localCoordinates 计算，天然处理 +-pi 跨界。
class YawFactor : public gtsam::NoiseModelFactorN<gtsam::Rot2, double, gtsam::Rot2> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Rot2, double, gtsam::Rot2>;

    YawFactor(const gtsam::SharedNoiseModel& noise, gtsam::Key yaw_prev,
              gtsam::Key vyaw_prev, gtsam::Key yaw_cur, double dt);

    gtsam::Vector evaluateError(
        const gtsam::Rot2& yaw_prev, const double& vyaw_prev,
        const gtsam::Rot2& yaw_cur, gtsam::OptionalMatrixType H1,
        gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const override;

private:
    double dt_ = 0.0;
};

// 速度平滑因子:
//   residual = V(k) - V(k-1)
// 只表达常速模型中的速度随机游走，不再和位置状态绑成一个大 Vector。
class VelocityFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::Vector3> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::Vector3>;

    VelocityFactor(const gtsam::SharedNoiseModel& noise, gtsam::Key v_prev,
                   gtsam::Key v_cur);

    gtsam::Vector evaluateError(
        const gtsam::Vector3& v_prev, const gtsam::Vector3& v_cur,
        gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const override;
};

// 角速度平滑因子:
//   residual = W(k) - W(k-1)
class VyawFactor : public gtsam::NoiseModelFactorN<double, double> {
public:
    using Base = gtsam::NoiseModelFactorN<double, double>;

    VyawFactor(const gtsam::SharedNoiseModel& noise, gtsam::Key w_prev,
               gtsam::Key w_cur);

    gtsam::Vector evaluateError(
        const double& w_prev, const double& w_cur,
        gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const override;
};

// 偶数装甲板几何因子：H_i(k), radius_a, yaw(k), center(k)。
// 残差约束装甲板 pose 与中心状态之间的切向/径向/Z/yaw 关系。
class ArmorRadiusCenterZFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, double, gtsam::Rot2,
                                      gtsam::Point3> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, double, gtsam::Rot2,
                                          gtsam::Point3>;

    ArmorRadiusCenterZFactor(const gtsam::SharedNoiseModel& noise,
                             gtsam::Key armor_pose_key, gtsam::Key radius_key,
                             gtsam::Key center_yaw_key, gtsam::Key center_key,
                             const Eigen::Isometry3d& T_camera_to_odom,
                             int armor_index, double radius_min = kRadiusMin,
                             double radius_max = kRadiusMax);

    gtsam::Vector evaluateError(
        const gtsam::Pose3& armor_pose_camera, const double& radius_u,
        const gtsam::Rot2& center_yaw, const gtsam::Point3& center,
        gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
        gtsam::OptionalMatrixType H3, gtsam::OptionalMatrixType H4) const override;

private:
    Eigen::Isometry3d T_camera_to_odom_;
    int armor_index_ = 0;
    double radius_min_ = kRadiusMin;
    double radius_max_ = kRadiusMax;
};

// 奇数装甲板几何因子：H_i(k), radius_b, dz, yaw(k), center(k)。
// 相比偶数装甲板额外引入 dz，描述上下层装甲板高度差。
class ArmorRadiusDZFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, double, double, gtsam::Rot2,
                                      gtsam::Point3> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, double, double,
                                          gtsam::Rot2, gtsam::Point3>;

    ArmorRadiusDZFactor(const gtsam::SharedNoiseModel& noise,
                        gtsam::Key armor_pose_key, gtsam::Key radius_key,
                        gtsam::Key dz_key, gtsam::Key center_yaw_key,
                        gtsam::Key center_key,
                        const Eigen::Isometry3d& T_camera_to_odom,
                        int armor_index, double radius_min = kRadiusMin,
                        double radius_max = kRadiusMax);

    gtsam::Vector evaluateError(
        const gtsam::Pose3& armor_pose_camera, const double& radius_u,
        const double& dz, const gtsam::Rot2& center_yaw,
        const gtsam::Point3& center, gtsam::OptionalMatrixType H1,
        gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
        gtsam::OptionalMatrixType H4, gtsam::OptionalMatrixType H5) const override;

private:
    Eigen::Isometry3d T_camera_to_odom_;
    int armor_index_ = 0;
    double radius_min_ = kRadiusMin;
    double radius_max_ = kRadiusMax;
};

// 前哨站几何因子：H_i(k), outpost_radius, dz_1, dz_2, base_z, base_xy, yaw(k), center(k)。
// 三块装甲板共用半径，index 1/2 分别使用独立高度偏移 dz_1/dz_2。
class ArmorOutpostGeometryFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, double, double, double,
                                      double, gtsam::Point2, gtsam::Rot2,
                                      gtsam::Point3> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, double, double, double,
                                          double, gtsam::Point2, gtsam::Rot2,
                                          gtsam::Point3>;

    ArmorOutpostGeometryFactor(
        const gtsam::SharedNoiseModel& noise,
        gtsam::Key armor_pose_key,
        gtsam::Key radius_key,
        gtsam::Key dz1_key,
        gtsam::Key dz2_key,
        gtsam::Key base_z_key,
        gtsam::Key base_xy_key,
        gtsam::Key center_yaw_key,
        gtsam::Key center_key,
        const Eigen::Isometry3d& T_camera_to_odom,
        int armor_index,
        double radius_min = kRadiusMin,
        double radius_max = kRadiusMax);

    gtsam::Vector evaluateError(
        const gtsam::Pose3& armor_pose_camera,
        const double& radius_u,
        const double& dz1,
        const double& dz2,
        const double& base_z,
        const gtsam::Point2& base_xy,
        const gtsam::Rot2& center_yaw,
        const gtsam::Point3& center,
        gtsam::OptionalMatrixType H1,
        gtsam::OptionalMatrixType H2,
        gtsam::OptionalMatrixType H3,
        gtsam::OptionalMatrixType H4,
        gtsam::OptionalMatrixType H5,
        gtsam::OptionalMatrixType H6,
        gtsam::OptionalMatrixType H7,
        gtsam::OptionalMatrixType H8) const override;

private:
    Eigen::Isometry3d T_camera_to_odom_;
    int armor_index_ = 0;
    double radius_min_ = kRadiusMin;
    double radius_max_ = kRadiusMax;
};

// 单角点像素重投影因子。该因子只连接装甲板 Pose3，
// Pose3 再通过几何因子连接到中心/yaw/半径/dz 主状态。
class ArmorTypedReprojFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

    ArmorTypedReprojFactor(const gtsam::SharedNoiseModel& noise,
                           gtsam::Key armor_pose_key,
                           const Eigen::Vector3d& corner_local,
                           const Eigen::Matrix3d& K,
                           const std::array<double, 5>& dist,
                           const Eigen::Vector2d& observed_pixel);

    gtsam::Vector evaluateError(
        const gtsam::Pose3& armor_pose_camera,
        gtsam::OptionalMatrixType H) const override;

private:
    gtsam::Point3 armor_point_;
    gtsam::Cal3DS2 calib_;
    gtsam::Point2 px_obs_;
};

// auto_aim EdgeLoss 风格的直接重投影因子：不再经过辅助 Pose3，而是从
// center/yaw/radius 直接预测四角点，并对四条边计算归一化长度误差和角度误差。
class ArmorEdgeCenterZReprojFactor
    : public gtsam::NoiseModelFactorN<double, gtsam::Rot2, gtsam::Point3> {
public:
    using Base = gtsam::NoiseModelFactorN<double, gtsam::Rot2, gtsam::Point3>;

    ArmorEdgeCenterZReprojFactor(
        const gtsam::SharedNoiseModel& noise,
        gtsam::Key radius_key,
        gtsam::Key center_yaw_key,
        gtsam::Key center_key,
        const Eigen::Isometry3d& T_camera_to_odom,
        int armor_index,
        double armor_pitch,
        const Eigen::Matrix3d& K,
        const std::array<double, 5>& dist,
        const std::array<Eigen::Vector2d, 4>& observed_pixels,
        double slope_k = 2.0,
        double radius_min = kRadiusMin,
        double radius_max = kRadiusMax);

    gtsam::Vector evaluateError(
        const double& radius_u,
        const gtsam::Rot2& center_yaw,
        const gtsam::Point3& center,
        gtsam::OptionalMatrixType H1,
        gtsam::OptionalMatrixType H2,
        gtsam::OptionalMatrixType H3) const override;

private:
    gtsam::Vector computeError(
        const double& radius_u,
        const gtsam::Rot2& center_yaw,
        const gtsam::Point3& center) const;

    Eigen::Isometry3d T_camera_to_odom_;
    int armor_index_ = 0;
    double armor_pitch_ = 15.0 * M_PI / 180.0;
    Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();
    std::array<double, 5> dist_{};
    std::array<Eigen::Vector2d, 4> observed_pixels_{};
    double slope_k_ = 2.0;
    double radius_min_ = kRadiusMin;
    double radius_max_ = kRadiusMax;
};

class ArmorEdgeDZReprojFactor
    : public gtsam::NoiseModelFactorN<double, double, gtsam::Rot2, gtsam::Point3> {
public:
    using Base = gtsam::NoiseModelFactorN<double, double, gtsam::Rot2, gtsam::Point3>;

    ArmorEdgeDZReprojFactor(
        const gtsam::SharedNoiseModel& noise,
        gtsam::Key radius_key,
        gtsam::Key dz_key,
        gtsam::Key center_yaw_key,
        gtsam::Key center_key,
        const Eigen::Isometry3d& T_camera_to_odom,
        int armor_index,
        double armor_pitch,
        const Eigen::Matrix3d& K,
        const std::array<double, 5>& dist,
        const std::array<Eigen::Vector2d, 4>& observed_pixels,
        double slope_k = 2.0,
        double radius_min = kRadiusMin,
        double radius_max = kRadiusMax);

    gtsam::Vector evaluateError(
        const double& radius_u,
        const double& dz,
        const gtsam::Rot2& center_yaw,
        const gtsam::Point3& center,
        gtsam::OptionalMatrixType H1,
        gtsam::OptionalMatrixType H2,
        gtsam::OptionalMatrixType H3,
        gtsam::OptionalMatrixType H4) const override;

private:
    gtsam::Vector computeError(
        const double& radius_u,
        const double& dz,
        const gtsam::Rot2& center_yaw,
        const gtsam::Point3& center) const;

    Eigen::Isometry3d T_camera_to_odom_;
    int armor_index_ = 0;
    double armor_pitch_ = 15.0 * M_PI / 180.0;
    Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();
    std::array<double, 5> dist_{};
    std::array<Eigen::Vector2d, 4> observed_pixels_{};
    double slope_k_ = 2.0;
    double radius_min_ = kRadiusMin;
    double radius_max_ = kRadiusMax;
};

class ArmorEdgeOutpostReprojFactor
    : public gtsam::NoiseModelFactorN<double, double, double, double,
                                      gtsam::Point2, gtsam::Rot2,
                                      gtsam::Point3> {
public:
    using Base = gtsam::NoiseModelFactorN<double, double, double, double,
                                          gtsam::Point2, gtsam::Rot2,
                                          gtsam::Point3>;

    ArmorEdgeOutpostReprojFactor(
        const gtsam::SharedNoiseModel& noise,
        gtsam::Key radius_key,
        gtsam::Key dz1_key,
        gtsam::Key dz2_key,
        gtsam::Key base_z_key,
        gtsam::Key base_xy_key,
        gtsam::Key center_yaw_key,
        gtsam::Key center_key,
        const Eigen::Isometry3d& T_camera_to_odom,
        int armor_index,
        double armor_pitch,
        const Eigen::Matrix3d& K,
        const std::array<double, 5>& dist,
        const std::array<Eigen::Vector2d, 4>& observed_pixels,
        double slope_k = 2.0,
        double radius_min = kRadiusMin,
        double radius_max = kRadiusMax);

    gtsam::Vector evaluateError(
        const double& radius_u,
        const double& dz1,
        const double& dz2,
        const double& base_z,
        const gtsam::Point2& base_xy,
        const gtsam::Rot2& center_yaw,
        const gtsam::Point3& center,
        gtsam::OptionalMatrixType H1,
        gtsam::OptionalMatrixType H2,
        gtsam::OptionalMatrixType H3,
        gtsam::OptionalMatrixType H4,
        gtsam::OptionalMatrixType H5,
        gtsam::OptionalMatrixType H6,
        gtsam::OptionalMatrixType H7) const override;

private:
    gtsam::Vector computeError(
        const double& radius_u,
        const double& dz1,
        const double& dz2,
        const double& base_z,
        const gtsam::Point2& base_xy,
        const gtsam::Rot2& center_yaw,
        const gtsam::Point3& center) const;

    Eigen::Isometry3d T_camera_to_odom_;
    int armor_index_ = 0;
    double armor_pitch_ = -0.26;
    Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();
    std::array<double, 5> dist_{};
    std::array<Eigen::Vector2d, 4> observed_pixels_{};
    double slope_k_ = 2.0;
    double radius_min_ = kRadiusMin;
    double radius_max_ = kRadiusMax;
};

class ArmorEdgeYawReprojFactor : public gtsam::NoiseModelFactorN<gtsam::Rot2> {
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Rot2>;

    ArmorEdgeYawReprojFactor(
        const gtsam::SharedNoiseModel& noise,
        gtsam::Key center_yaw_key,
        const Eigen::Vector3d& center,
        double radius,
        double dz,
        int armor_index,
        double armor_pitch,
        const Eigen::Isometry3d& T_camera_to_odom,
        const Eigen::Matrix3d& K,
        const std::array<double, 5>& dist,
        const std::array<Eigen::Vector2d, 4>& observed_pixels,
        double slope_k = 2.0,
        int armor_count = 4);

    gtsam::Vector evaluateError(
        const gtsam::Rot2& center_yaw,
        gtsam::OptionalMatrixType H) const override;

private:
    gtsam::Vector computeError(const gtsam::Rot2& center_yaw) const;

    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    double radius_ = 0.25;
    double dz_ = 0.0;
    int armor_index_ = 0;
    double armor_pitch_ = 15.0 * M_PI / 180.0;
    Eigen::Isometry3d T_camera_to_odom_ = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();
    std::array<double, 5> dist_{};
    std::array<Eigen::Vector2d, 4> observed_pixels_{};
    double slope_k_ = 2.0;
    int armor_count_ = 4;
};

// 图运行时缓存：保存上一帧状态和匹配结果，用于预测初值和索引连续性。
struct ArmorCvPixelRuntime {
    TrackerState state;
    std::vector<int> matched_indices;
    std::vector<double> match_costs;
    int last_armor_index = -1;
};

// 图输出会被 ArmorGraphTracker 转成 ROS 层需要的 TrackerUpdateResult。
struct ArmorCvPixelOutput {
    auto_graph::SolveResult solve_result;
    TrackerState state;
    std::vector<PredictedArmor> predicted_armors;
    std::vector<int> matched_indices;
    std::vector<double> match_costs;
};

// 面向调用层的 CV + 像素观测图封装。它直接组织变量声明、图初始化、单帧写图、
// 求解和输出转换；GraphOptimizer 只负责 typed GTSAM 生命周期。
class ArmorCvPixelGraph {
public:
    explicit ArmorCvPixelGraph(TrackerConfig config);

    void initialize(
        const auto_aim_interfaces::msg::Armors& armors_msg,
        const TrackerState* outpost_reinit_hint = nullptr);
    ArmorCvPixelOutput update(
        const auto_aim_interfaces::msg::Armors& armors_msg,
        double dt,
        const Eigen::Isometry3d& T_camera_to_odom);
    void reset();

    bool initialized() const { return initialized_; }
    uint64_t frameId() const;
    const TrackerState& state() const { return runtime_.state; }

private:
    struct Variables {
        auto_graph::Var<gtsam::Point3> center;
        auto_graph::Var<gtsam::Vector3> velocity;
        auto_graph::Var<gtsam::Rot2> yaw;
        auto_graph::Var<double> vyaw;
        auto_graph::Var<double> radius_a;
        auto_graph::Var<double> radius_b;
        auto_graph::Var<double> dz;
        auto_graph::Var<double> outpost_radius;
        auto_graph::Var<double> outpost_dz_1;
        auto_graph::Var<double> outpost_dz_2;
        auto_graph::Var<double> outpost_base_z;
        auto_graph::Var<gtsam::Point2> outpost_base_xy;
    };

    static Variables declareVariables(auto_graph::GraphOptimizer& optimizer);

    void addMotionFactors(double dt);
    void addObservationFactors(
        const auto_aim_interfaces::msg::Armors& armors_msg,
        const Eigen::Isometry3d& T_camera_to_odom);
    void addArmorObservationFactors(
        const auto_aim_interfaces::msg::Armor& armor,
        int armor_index,
        const Eigen::Isometry3d& T_camera_to_odom,
        const gtsam::SharedNoiseModel& pixel_noise,
        const gtsam::SharedNoiseModel& pose_prior_noise,
        const gtsam::SharedNoiseModel& geo_noise);
    ArmorCvPixelOutput makeOutput(const auto_graph::SolveResult& solve_result);

    TrackerConfig config_;
    ArmorCvPixelRuntime runtime_;
    auto_graph::GraphOptimizer optimizer_;
    Variables vars_;
    bool initialized_ = false;
};

}  // namespace filter_test::graph_optimizer

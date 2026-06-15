#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <array>
#include <set>
#include <utility>

#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>

#include "filter_test/graph_optimizer/armor_tracker.hpp"
#include "filter_test/graph_optimizer/rune_model.hpp"
#include "filter_test/graph_optimizer_test.hpp"
#include "filter_test/visualization_marker_utils.hpp"

// 注意：由于GTSAM未安装C++版本，此测试仅验证不依赖GTSAM的部分
// 完整测试需要安装GTSAM C++库

namespace {

auto_aim_interfaces::msg::RuneTarget makeRuneTargetFromPixels(
    const std::array<Eigen::Vector2d, 5>& pixels) {
    auto target = auto_aim_interfaces::msg::RuneTarget();
    target.is_detected = true;
    target.r_center.x = static_cast<float>(pixels[0].x());
    target.r_center.y = static_cast<float>(pixels[0].y());
    target.near_point.x = static_cast<float>(pixels[1].x());
    target.near_point.y = static_cast<float>(pixels[1].y());
    target.left_point.x = static_cast<float>(pixels[2].x());
    target.left_point.y = static_cast<float>(pixels[2].y());
    target.far_point.x = static_cast<float>(pixels[3].x());
    target.far_point.y = static_cast<float>(pixels[3].y());
    target.right_point.x = static_cast<float>(pixels[4].x());
    target.right_point.y = static_cast<float>(pixels[4].y());
    return target;
}

}  // namespace

// ==================== 测试基础数学工具 ====================

TEST(MathUtilsTest, AngleNormalization) {
    // 测试角度归一化
    auto normalizeAngle = [](double angle) -> double {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    };

    EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-10);
    EXPECT_NEAR(normalizeAngle(M_PI), M_PI, 1e-10);
    EXPECT_NEAR(normalizeAngle(-M_PI), -M_PI, 1e-10);
    EXPECT_NEAR(normalizeAngle(3.0 * M_PI), M_PI, 1e-10);
    EXPECT_NEAR(normalizeAngle(-3.0 * M_PI), -M_PI, 1e-10);
    EXPECT_NEAR(normalizeAngle(M_PI / 2.0), M_PI / 2.0, 1e-10);
}

TEST(MathUtilsTest, ShortestAngularDistance) {
    auto shortestAngularDistance = [](double from, double to) -> double {
        double diff = to - from;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        return diff;
    };

    EXPECT_NEAR(shortestAngularDistance(0.0, 0.0), 0.0, 1e-10);
    EXPECT_NEAR(shortestAngularDistance(0.0, M_PI), M_PI, 1e-10);
    EXPECT_NEAR(shortestAngularDistance(0.0, -M_PI), -M_PI, 1e-10);
    EXPECT_NEAR(shortestAngularDistance(M_PI - 0.1, -M_PI + 0.1), 0.2, 1e-10);
}

// ==================== 测试Eigen操作 ====================

TEST(EigenTest, MatrixOperations) {
    Eigen::MatrixXd A(2, 2);
    A << 1, 2,
         3, 4;

    Eigen::MatrixXd B(2, 2);
    B << 5, 6,
         7, 8;

    // 矩阵乘法
    Eigen::MatrixXd C = A * B;
    EXPECT_NEAR(C(0, 0), 19, 1e-10);  // 1*5 + 2*7
    EXPECT_NEAR(C(0, 1), 22, 1e-10);  // 1*6 + 2*8
    EXPECT_NEAR(C(1, 0), 43, 1e-10);  // 3*5 + 4*7
    EXPECT_NEAR(C(1, 1), 50, 1e-10);  // 3*6 + 4*8

    // 转置
    Eigen::MatrixXd AT = A.transpose();
    EXPECT_NEAR(AT(0, 0), 1, 1e-10);
    EXPECT_NEAR(AT(0, 1), 3, 1e-10);
    EXPECT_NEAR(AT(1, 0), 2, 1e-10);
    EXPECT_NEAR(AT(1, 1), 4, 1e-10);

    // 逆矩阵
    Eigen::MatrixXd A_inv = A.inverse();
    Eigen::MatrixXd I = A * A_inv;
    EXPECT_NEAR(I(0, 0), 1.0, 1e-10);
    EXPECT_NEAR(I(0, 1), 0.0, 1e-10);
    EXPECT_NEAR(I(1, 0), 0.0, 1e-10);
    EXPECT_NEAR(I(1, 1), 1.0, 1e-10);
}

TEST(EigenTest, VectorOperations) {
    Eigen::VectorXd v(3);
    v << 1.0, 2.0, 3.0;

    Eigen::VectorXd w(3);
    w << 4.0, 5.0, 6.0;

    // 点积
    double dot = v.dot(w);
    EXPECT_NEAR(dot, 32.0, 1e-10);  // 1*4 + 2*5 + 3*6

    // 范数
    double norm = v.norm();
    EXPECT_NEAR(norm, std::sqrt(14.0), 1e-10);  // sqrt(1+4+9)

    // 加法
    Eigen::VectorXd sum = v + w;
    EXPECT_NEAR(sum[0], 5.0, 1e-10);
    EXPECT_NEAR(sum[1], 7.0, 1e-10);
    EXPECT_NEAR(sum[2], 9.0, 1e-10);
}

// ==================== 测试卡尔曼滤波基础 ====================

TEST(KalmanFilterTest, PredictStep) {
    // 简单的匀速模型
    int n = 2;  // 状态维度：[位置, 速度]
    double dt = 0.1;

    // 状态转移矩阵
    Eigen::MatrixXd F(n, n);
    F << 1, dt,
         0, 1;

    // 初始状态
    Eigen::VectorXd x(n);
    x << 0.0, 1.0;  // 位置0，速度1

    // 初始协方差
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(n, n) * 0.1;

    // 过程噪声
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n, n) * 0.01;

    // 预测
    Eigen::VectorXd x_pred = F * x;
    Eigen::MatrixXd P_pred = F * P * F.transpose() + Q;

    // 验证预测状态
    EXPECT_NEAR(x_pred[0], 0.1, 1e-10);  // 0 + 0.1 * 1
    EXPECT_NEAR(x_pred[1], 1.0, 1e-10);  // 速度不变

    // 验证协方差传播
    EXPECT_GT(P_pred(0, 0), P(0, 0));  // 位置不确定性增加
    EXPECT_GT(P_pred(1, 1), P(1, 1));  // 速度不确定性增加
}

TEST(KalmanFilterTest, UpdateStep) {
    // 简单的线性观测模型
    int n = 2;  // 状态维度
    int m = 1;  // 观测维度

    // 观测矩阵
    Eigen::MatrixXd H(m, n);
    H << 1, 0;  // 只观测位置

    // 预测状态
    Eigen::VectorXd x_pred(n);
    x_pred << 0.1, 1.0;

    // 预测协方差
    Eigen::MatrixXd P_pred = Eigen::MatrixXd::Identity(n, n) * 0.1;

    // 观测值
    Eigen::VectorXd z(m);
    z << 0.12;

    // 观测噪声
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m, m) * 0.01;

    // 新息
    Eigen::VectorXd innovation = z - H * x_pred;

    // 新息协方差
    Eigen::MatrixXd S = H * P_pred * H.transpose() + R;

    // 卡尔曼增益
    Eigen::MatrixXd K = P_pred * H.transpose() * S.inverse();

    // 更新状态
    Eigen::VectorXd x_updated = x_pred + K * innovation;

    // 更新协方差
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd P_updated = (I - K * H) * P_pred;

    // 验证更新后的状态更接近观测
    EXPECT_NEAR(x_updated[0], 0.12, 0.05);  // 应该接近0.12
    EXPECT_GT(P_pred(0, 0), P_updated(0, 0));  // 不确定性减小
}

// ==================== 测试NIS计算 ====================

TEST(NISTest, ComputeNIS) {
    int measure_dim = 2;

    Eigen::VectorXd innovation(measure_dim);
    innovation << 1.0, 0.5;

    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(measure_dim, measure_dim);

    // 手动计算NIS
    double nis = innovation.transpose() * S.inverse() * innovation;
    EXPECT_NEAR(nis, 1.25, 1e-10);  // 1^2 + 0.5^2 = 1.25

    // 使用Cholesky分解
    Eigen::LLT<Eigen::MatrixXd> llt(S);
    Eigen::VectorXd z = llt.solve(innovation);
    double nis_chol = innovation.transpose() * z;
    EXPECT_NEAR(nis_chol, 1.25, 1e-10);
}

TEST(NISTest, ChiSquareThreshold) {
    // 测试卡方分布的分位数（95%置信度）
    // df=1: 3.841
    // df=2: 5.991
    // df=3: 7.815

    // 简单的查表实现
    auto chiSquareThreshold = [](int df) -> double {
        static const double table[] = {
            3.841, 5.991, 7.815, 9.488, 11.070,
            12.592, 14.067, 15.507, 16.919, 18.307
        };
        if (df >= 1 && df <= 10) {
            return table[df - 1];
        }
        return df + 3.0 * std::sqrt(2.0 * df);  // 近似
    };

    EXPECT_NEAR(chiSquareThreshold(1), 3.841, 0.001);
    EXPECT_NEAR(chiSquareThreshold(2), 5.991, 0.001);
    EXPECT_NEAR(chiSquareThreshold(3), 7.815, 0.001);
}

TEST(ArmorObservationNoiseTest, StandardNoiseScalesWithDistance) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.pixel_sigma = 1.0;
    config.pixel_sigma_distance_quadratic = 0.05;
    config.observation_noise_reference_distance = 5.0;
    config.pose_prior_sigma = 0.1;
    config.pose_prior_distance_scale = 0.02;
    config.geo_noise.tangential = 0.02;
    config.geo_noise.radial = 0.03;
    config.geo_radial_distance_scale = 0.01;
    config.geo_noise.yaw = 0.05;
    config.geo_yaw_distance_scale = 0.01;

    auto_aim_interfaces::msg::Armor near_armor;
    near_armor.number = "small";
    near_armor.pose.position.x = 3.0;
    near_armor.pose.position.y = 4.0;
    near_armor.pose.position.z = 0.0;

    auto_aim_interfaces::msg::Armor far_armor = near_armor;
    far_armor.pose.position.x = 9.0;
    far_armor.pose.position.y = 0.0;

    const auto near_noise =
        filter_test::graph_optimizer::observationNoiseForArmor(config, near_armor);
    const auto far_noise =
        filter_test::graph_optimizer::observationNoiseForArmor(config, far_armor);

    EXPECT_NEAR(near_noise.pixel_sigma, 1.0, 1e-12);
    EXPECT_GT(far_noise.pixel_sigma, near_noise.pixel_sigma);
    EXPECT_GT(far_noise.pose_prior_sigma, near_noise.pose_prior_sigma);
    EXPECT_GT(far_noise.geo_noise.tangential, near_noise.geo_noise.tangential);
    EXPECT_GT(far_noise.geo_noise.radial, near_noise.geo_noise.radial);
    EXPECT_GT(far_noise.geo_noise.yaw, near_noise.geo_noise.yaw);
}

TEST(ArmorObservationNoiseTest, OutpostUsesDedicatedObservationNoise) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.pixel_sigma = 1.0;
    config.outpost_pixel_sigma = 2.0;
    config.observation_noise_reference_distance = 5.0;
    config.pixel_sigma_distance_quadratic = 0.0;
    config.geo_noise.height = 0.01;
    config.geo_noise.yaw = 0.05;
    config.outpost_geo_noise.height = 0.005;
    config.outpost_geo_noise.yaw = 0.01;

    auto_aim_interfaces::msg::Armor armor;
    armor.number = "outpost";
    armor.pose.position.x = 0.0;
    armor.pose.position.y = 5.0;
    armor.pose.position.z = 0.0;

    const auto noise =
        filter_test::graph_optimizer::observationNoiseForArmor(config, armor);

    EXPECT_NEAR(noise.pixel_sigma, 2.0, 1e-12);
    EXPECT_NEAR(noise.geo_noise.height, 0.005, 1e-12);
    EXPECT_NEAR(noise.geo_noise.yaw, 0.01, 1e-12);
}

TEST(RuneGraphFactorTest, ReprojectionResidualIsZeroForObservedProjection) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 1000.0;
    K(1, 1) = 1000.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const gtsam::Key pose_key = gtsam::Symbol('q', 0);
    filter_test::graph_optimizer::RuneBladeReprojFactor factor(
        auto_graph::isotropicNoise(2, 1.0),
        pose_key,
        Eigen::Vector3d::Zero(),
        K,
        dist,
        Eigen::Vector2d(640.0, 360.0));

    const gtsam::Pose3 pose(
        gtsam::Rot3::Identity(),
        gtsam::Point3(0.0, 0.0, 5.0));
    const auto residual = factor.evaluateError(pose, {});

    ASSERT_EQ(residual.size(), 2);
    EXPECT_NEAR(residual.x(), 0.0, 1e-9);
    EXPECT_NEAR(residual.y(), 0.0, 1e-9);
}

TEST(RuneGraphFactorTest, DirectRuneRollLossIsZeroForObservedProjection) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 900.0;
    K(1, 1) = 900.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const Eigen::Vector3d center(0.0, 0.0, 5.0);
    const double roll = 0.35;
    const double normal_yaw = 0.0;
    const double normal_pitch = -M_PI_2;
    const int blade_index = 1;
    const auto pixels = filter_test::graph_optimizer::projectRuneBladePixels(
        center, normal_yaw, normal_pitch,
        roll + blade_index * filter_test::graph_optimizer::kRuneSlotAngle,
        Eigen::Isometry3d::Identity(), K, dist);

    filter_test::graph_optimizer::RuneBladeDirectReprojFactor factor(
        auto_graph::isotropicNoise(4, 1.0),
        gtsam::Symbol('c', 0), gtsam::Symbol('r', 0), gtsam::Symbol('n', 0),
        Eigen::Isometry3d::Identity(), blade_index, normal_pitch, K, dist, pixels);

    const auto residual = factor.evaluateError(
        auto_graph::eigenToPoint3(center),
        gtsam::Rot2::fromAngle(roll),
        gtsam::Rot2::fromAngle(normal_yaw),
        nullptr, nullptr, nullptr);

    ASSERT_EQ(residual.size(), 4);
    EXPECT_TRUE(residual.isZero(1e-9)) << residual.transpose();
}

TEST(RuneGraphFactorTest, DirectRuneRollLossRespondsToRollChange) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 900.0;
    K(1, 1) = 900.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const Eigen::Vector3d center(0.0, 0.0, 5.0);
    const double roll = 0.35;
    const double normal_yaw = 0.0;
    const double normal_pitch = -M_PI_2;
    const int blade_index = 0;
    const auto pixels = filter_test::graph_optimizer::projectRuneBladePixels(
        center, normal_yaw, normal_pitch, roll,
        Eigen::Isometry3d::Identity(), K, dist);

    filter_test::graph_optimizer::RuneBladeDirectReprojFactor factor(
        auto_graph::isotropicNoise(4, 1.0),
        gtsam::Symbol('c', 0), gtsam::Symbol('r', 0), gtsam::Symbol('n', 0),
        Eigen::Isometry3d::Identity(), blade_index, normal_pitch, K, dist, pixels);

    const auto residual = factor.evaluateError(
        auto_graph::eigenToPoint3(center),
        gtsam::Rot2::fromAngle(roll + 0.10),
        gtsam::Rot2::fromAngle(normal_yaw),
        nullptr, nullptr, nullptr);

    ASSERT_EQ(residual.size(), 4);
    EXPECT_GT(residual.cwiseAbs().sum(), 0.05);
}

TEST(RuneGraphFactorTest, DirectRuneRollOnlyLossOnlyExposesRollJacobian) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 900.0;
    K(1, 1) = 900.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const Eigen::Vector3d center(0.0, 0.0, 5.0);
    const double roll = 0.35;
    const double normal_yaw = 0.0;
    const double normal_pitch = -M_PI_2;
    const int blade_index = 2;
    const auto pixels = filter_test::graph_optimizer::projectRuneBladePixels(
        center, normal_yaw, normal_pitch,
        roll + blade_index * filter_test::graph_optimizer::kRuneSlotAngle,
        Eigen::Isometry3d::Identity(), K, dist);

    filter_test::graph_optimizer::RuneBladeRollDirectReprojFactor factor(
        auto_graph::isotropicNoise(4, 1.0),
        gtsam::Symbol('r', 0),
        center,
        normal_yaw,
        Eigen::Isometry3d::Identity(), blade_index, normal_pitch, K, dist, pixels);

    gtsam::Matrix H;
    const auto residual = factor.evaluateError(
        gtsam::Rot2::fromAngle(roll),
        &H);

    ASSERT_EQ(residual.size(), 4);
    EXPECT_TRUE(residual.isZero(1e-9)) << residual.transpose();
    ASSERT_EQ(H.rows(), 4);
    ASSERT_EQ(H.cols(), 1);

    const auto moved_residual = factor.evaluateError(
        gtsam::Rot2::fromAngle(roll + 0.10),
        nullptr);
    EXPECT_GT(moved_residual.cwiseAbs().sum(), 0.05);
}

TEST(RuneGraphFactorTest, GeometryFactorConstrainsNormalYaw) {
    const Eigen::Vector3d center(0.2, -0.1, 3.0);
    const double roll = 0.4;
    const double normal_yaw = 0.3;
    const double normal_pitch = 0.0;
    Eigen::Isometry3d blade_pose = Eigen::Isometry3d::Identity();
    blade_pose.translation() = center;
    blade_pose.linear() = filter_test::graph_optimizer::runeBladeRotation(
        normal_yaw, normal_pitch, roll);

    filter_test::graph_optimizer::RuneBladeGeometryFactor factor(
        auto_graph::isotropicNoise(5, 1.0),
        gtsam::Symbol('q', 0), gtsam::Symbol('c', 0), gtsam::Symbol('r', 0),
        gtsam::Symbol('n', 0), Eigen::Isometry3d::Identity(), 0, normal_pitch);

    gtsam::Matrix H4;
    const auto residual = factor.evaluateError(
        gtsam::Pose3(gtsam::Rot3(blade_pose.rotation()),
                     auto_graph::eigenToPoint3(blade_pose.translation())),
        auto_graph::eigenToPoint3(center),
        gtsam::Rot2::fromAngle(roll),
        gtsam::Rot2::fromAngle(normal_yaw),
        nullptr, nullptr, nullptr, &H4);

    ASSERT_EQ(residual.size(), 5);
    EXPECT_TRUE(residual.isZero(1e-9)) << residual.transpose();
    ASSERT_EQ(H4.rows(), 5);
    ASSERT_EQ(H4.cols(), 1);
    EXPECT_GT(std::abs(H4(4, 0)), 0.5);
}

TEST(RuneGraphFactorTest, FivePointPnPRecoversBladePose) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 1000.0;
    K(1, 1) = 1000.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const Eigen::Vector3d center(0.1, -0.2, 4.5);
    const double roll = 0.25;
    const auto pixels = filter_test::graph_optimizer::projectRuneBladePixels(
        center, 0.0, 0.0, roll, Eigen::Isometry3d::Identity(), K, dist);

    const auto result = filter_test::graph_optimizer::estimateRuneBladePosePnP(pixels, K, dist);

    ASSERT_TRUE(result.success);
    EXPECT_TRUE(auto_graph::point3ToEigen(result.pose_camera.translation()).isApprox(center, 1e-4));
}

TEST(RuneBigCurveFitterTest, FitsFiveParameterBigRuneCurve) {
    filter_test::graph_optimizer::RuneBigCurveFitterConfig config;
    config.min_data_size = 20;
    config.max_data_size = 120;
    config.max_iterations = 50;
    filter_test::graph_optimizer::RuneBigCurveFitter fitter(config);

    const std::array<double, 5> truth{0.47, 1.942, -0.4, 1.178, 0.2};
    for (int i = 0; i < 80; ++i) {
        const double t = static_cast<double>(i) * 0.02;
        fitter.addSample(t, filter_test::graph_optimizer::runeBigCurveAngle(t, truth));
    }

    ASSERT_TRUE(fitter.fitOnce());
    const auto params = fitter.params();
    EXPECT_NEAR(params[1], truth[1], 0.08);
    EXPECT_NEAR(params[3], truth[3], 0.08);
    EXPECT_NEAR(fitter.angleAt(1.0), filter_test::graph_optimizer::runeBigCurveAngle(1.0, truth), 0.08);
    EXPECT_NEAR(fitter.velocityAt(1.0), filter_test::graph_optimizer::runeBigCurveVelocity(1.0, truth), 0.25);
}

TEST(RuneGeometryTest, TargetPositionUsesRollAroundBladeAxis) {
    const Eigen::Vector3d center(1.0, 2.0, 3.0);

    const auto roll0 = filter_test::graph_optimizer::runeTargetPosition(
        center, 0.0, 0.0, 0.0);
    EXPECT_TRUE(roll0.isApprox(
        center + Eigen::Vector3d(0.0, 0.0, filter_test::graph_optimizer::kRuneBladeRadius),
        1e-12));

    const auto roll90 = filter_test::graph_optimizer::runeTargetPosition(
        center, 0.0, 0.0, M_PI_2);
    EXPECT_TRUE(roll90.isApprox(
        center + Eigen::Vector3d(0.0, -filter_test::graph_optimizer::kRuneBladeRadius, 0.0),
        1e-12));
}

TEST(RuneCvGraphTest, ColdStartEmptyObservationKeepsConfiguredState) {
    filter_test::graph_optimizer::RuneTrackerConfig config;
    config.optimizer.cold_start_frames = 1;
    config.optimizer.smoother_lag = 0.0;
    config.initial_center = Eigen::Vector3d(0.0, -5.0, 1.5);
    config.normal_yaw = M_PI_2;
    config.normal_pitch = 0.0;

    filter_test::graph_optimizer::RuneCvGraph graph(config);
    auto_aim_interfaces::msg::RuneTargets msg;
    Eigen::Isometry3d T_camera_to_odom = Eigen::Isometry3d::Identity();

    const auto output = graph.update(msg, 0.01, T_camera_to_odom);

    EXPECT_TRUE(graph.initialized());
    EXPECT_EQ(output.observed_count, 0);
    EXPECT_TRUE(output.solve_result.cold_start);
    EXPECT_FALSE(output.solve_result.failed);
    EXPECT_TRUE(output.state.center.isApprox(config.initial_center, 1e-12));
    EXPECT_NEAR(output.state.normal_yaw, config.normal_yaw, 1e-12);
    EXPECT_TRUE(output.target_position.isApprox(
        config.initial_center +
            Eigen::Vector3d(0.0, 0.0, filter_test::graph_optimizer::kRuneBladeRadius),
        1e-12));
}

TEST(RuneCvGraphTest, ColdStartObservationAnchorsFirstBladeWithoutIndex) {
    filter_test::graph_optimizer::RuneTrackerConfig config;
    config.optimizer.cold_start_frames = 1;
    config.optimizer.smoother_lag = 0.0;
    config.initial_center = Eigen::Vector3d(0.0, 0.0, 5.0);
    config.normal_yaw = 0.0;
    config.normal_pitch = -M_PI_2;
    config.pixel_sigma = 0.5;
    config.center_sigma = 0.001;
    config.roll_sigma = 5.0;
    config.vroll_sigma = 0.1;
    config.pose_prior_sigma = 0.001;
    config.pnp_pose_prior_sigma = 0.001;
    config.normal_yaw_sigma = 0.001;
    config.direct_reproj_sigma = 0.001;
    config.camera_matrix = Eigen::Matrix3d::Identity();
    config.camera_matrix(0, 0) = 900.0;
    config.camera_matrix(1, 1) = 900.0;
    config.camera_matrix(0, 2) = 640.0;
    config.camera_matrix(1, 2) = 360.0;

    const int blade_index = 3;
    const double base_roll = 0.0;
    const auto pixels = filter_test::graph_optimizer::projectRuneBladePixels(
        config.initial_center, config.normal_yaw, config.normal_pitch,
        base_roll + blade_index * filter_test::graph_optimizer::kRuneSlotAngle,
        Eigen::Isometry3d::Identity(), config.camera_matrix, config.distortion);

    auto_aim_interfaces::msg::RuneTargets msg;
    msg.targets.push_back(makeRuneTargetFromPixels(pixels));

    filter_test::graph_optimizer::RuneCvGraph graph(config);
    const auto output = graph.update(msg, 0.01, Eigen::Isometry3d::Identity());

    EXPECT_EQ(output.observed_count, 1);
    ASSERT_EQ(output.matched_blade_indices.size(), 1u);
    EXPECT_EQ(output.matched_blade_indices[0], 0);
    EXPECT_FALSE(output.solve_result.failed);
}

TEST(RuneCvGraphTest, OutputKeepsRawPnPTargetPositionForMatchedObservation) {
    filter_test::graph_optimizer::RuneTrackerConfig config;
    config.optimizer.cold_start_frames = 1;
    config.optimizer.smoother_lag = 0.0;
    config.initial_center = Eigen::Vector3d(0.0, 0.0, 5.0);
    config.normal_yaw = 0.0;
    config.normal_pitch = -M_PI_2;
    config.camera_matrix = Eigen::Matrix3d::Identity();
    config.camera_matrix(0, 0) = 900.0;
    config.camera_matrix(1, 1) = 900.0;
    config.camera_matrix(0, 2) = 640.0;
    config.camera_matrix(1, 2) = 360.0;

    const double roll = 0.2;
    auto pixels = filter_test::graph_optimizer::projectRuneBladePixels(
        config.initial_center, config.normal_yaw, config.normal_pitch, roll,
        Eigen::Isometry3d::Identity(), config.camera_matrix, config.distortion);
    pixels[1].x() += 1.5;
    pixels[2].y() -= 0.8;

    const auto pnp = filter_test::graph_optimizer::estimateRuneBladePosePnP(
        pixels, config.camera_matrix, config.distortion);
    ASSERT_TRUE(pnp.success);

    auto_aim_interfaces::msg::RuneTargets msg;
    msg.targets.push_back(makeRuneTargetFromPixels(pixels));

    filter_test::graph_optimizer::RuneCvGraph graph(config);
    const auto output = graph.update(msg, 0.01, Eigen::Isometry3d::Identity());

    ASSERT_EQ(output.observed_pnp_poses.size(), 1u);
    EXPECT_TRUE(output.observed_pnp_poses[0].translation().isApprox(
        auto_graph::point3ToEigen(pnp.pose_camera.translation()), 1e-6));
    ASSERT_EQ(output.observed_pnp_target_positions.size(), 1u);
    const Eigen::Vector3d expected_target =
        output.observed_pnp_poses[0] *
        Eigen::Vector3d(0.0, 0.0, filter_test::graph_optimizer::kRuneBladeRadius);
    EXPECT_TRUE(output.observed_pnp_target_positions[0].isApprox(expected_target, 1e-6));
    EXPECT_GT((output.observed_pnp_target_positions[0] -
               output.observed_pnp_poses[0].translation()).norm(), 0.5);
    EXPECT_FALSE(output.solve_result.failed);
}

TEST(RuneCvGraphTest, ColdStartTwoObservationsInferRelativeBladeSlots) {
    filter_test::graph_optimizer::RuneTrackerConfig config;
    config.optimizer.cold_start_frames = 1;
    config.optimizer.smoother_lag = 0.0;
    config.initial_center = Eigen::Vector3d(0.0, 0.0, 5.0);
    config.normal_yaw = 0.0;
    config.normal_pitch = -M_PI_2;
    config.camera_matrix = Eigen::Matrix3d::Identity();
    config.camera_matrix(0, 0) = 900.0;
    config.camera_matrix(1, 1) = 900.0;
    config.camera_matrix(0, 2) = 640.0;
    config.camera_matrix(1, 2) = 360.0;

    const double base_roll = 0.1;
    const auto first_pixels = filter_test::graph_optimizer::projectRuneBladePixels(
        config.initial_center, config.normal_yaw, config.normal_pitch,
        base_roll + 1.0 * filter_test::graph_optimizer::kRuneSlotAngle,
        Eigen::Isometry3d::Identity(), config.camera_matrix, config.distortion);
    const auto second_pixels = filter_test::graph_optimizer::projectRuneBladePixels(
        config.initial_center, config.normal_yaw, config.normal_pitch,
        base_roll + 3.0 * filter_test::graph_optimizer::kRuneSlotAngle,
        Eigen::Isometry3d::Identity(), config.camera_matrix, config.distortion);

    auto_aim_interfaces::msg::RuneTargets msg;
    msg.targets.push_back(makeRuneTargetFromPixels(first_pixels));
    msg.targets.push_back(makeRuneTargetFromPixels(second_pixels));

    filter_test::graph_optimizer::RuneCvGraph graph(config);
    const auto output = graph.update(msg, 0.01, Eigen::Isometry3d::Identity());

    EXPECT_EQ(output.observed_count, 2);
    ASSERT_EQ(output.matched_blade_indices.size(), 2u);
    EXPECT_EQ(output.matched_blade_indices[0], 0);
    EXPECT_EQ(output.matched_blade_indices[1], 2);
    EXPECT_FALSE(output.solve_result.failed);
}

TEST(RuneCvGraphTest, InitializedObservationMatchesPredictedSlotWithoutIndex) {
    filter_test::graph_optimizer::RuneTrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.smoother_lag = 0.0;
    config.initial_center = Eigen::Vector3d(0.0, 0.0, 5.0);
    config.normal_yaw = 0.0;
    config.normal_pitch = -M_PI_2;
    config.pixel_sigma = 0.5;
    config.center_sigma = 0.001;
    config.roll_sigma = 5.0;
    config.vroll_sigma = 0.1;
    config.pose_prior_sigma = 0.001;
    config.pnp_pose_prior_sigma = 0.001;
    config.normal_yaw_sigma = 0.001;
    config.direct_reproj_sigma = 0.001;
    config.camera_matrix = Eigen::Matrix3d::Identity();
    config.camera_matrix(0, 0) = 900.0;
    config.camera_matrix(1, 1) = 900.0;
    config.camera_matrix(0, 2) = 640.0;
    config.camera_matrix(1, 2) = 360.0;

    filter_test::graph_optimizer::RuneCvGraph graph(config);
    auto_aim_interfaces::msg::RuneTargets empty_msg;
    (void)graph.update(empty_msg, 0.01, Eigen::Isometry3d::Identity());

    const int physical_slot = 3;
    const auto pixels = filter_test::graph_optimizer::projectRuneBladePixels(
        config.initial_center, config.normal_yaw, config.normal_pitch,
        static_cast<double>(physical_slot) * filter_test::graph_optimizer::kRuneSlotAngle,
        Eigen::Isometry3d::Identity(), config.camera_matrix, config.distortion);

    auto_aim_interfaces::msg::RuneTargets msg;
    msg.targets.push_back(makeRuneTargetFromPixels(pixels));

    const auto output = graph.update(msg, 0.01, Eigen::Isometry3d::Identity());

    EXPECT_EQ(output.observed_count, 1);
    ASSERT_EQ(output.matched_blade_indices.size(), 1u);
    EXPECT_EQ(output.matched_blade_indices[0], physical_slot);
    EXPECT_FALSE(output.solve_result.failed);
}

TEST(RuneCvGraphTest, DuplicateSlotMatchKeepsBestObservationOnly) {
    filter_test::graph_optimizer::RuneTrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.smoother_lag = 0.0;
    config.initial_center = Eigen::Vector3d(0.0, 0.0, 5.0);
    config.normal_yaw = 0.0;
    config.normal_pitch = -M_PI_2;
    config.camera_matrix = Eigen::Matrix3d::Identity();
    config.camera_matrix(0, 0) = 900.0;
    config.camera_matrix(1, 1) = 900.0;
    config.camera_matrix(0, 2) = 640.0;
    config.camera_matrix(1, 2) = 360.0;

    filter_test::graph_optimizer::RuneCvGraph graph(config);
    auto_aim_interfaces::msg::RuneTargets empty_msg;
    (void)graph.update(empty_msg, 0.01, Eigen::Isometry3d::Identity());

    const auto pixels = filter_test::graph_optimizer::projectRuneBladePixels(
        config.initial_center, config.normal_yaw, config.normal_pitch,
        2.0 * filter_test::graph_optimizer::kRuneSlotAngle,
        Eigen::Isometry3d::Identity(), config.camera_matrix, config.distortion);

    auto_aim_interfaces::msg::RuneTargets msg;
    msg.targets.push_back(makeRuneTargetFromPixels(pixels));
    msg.targets.push_back(makeRuneTargetFromPixels(pixels));

    const auto output = graph.update(msg, 0.01, Eigen::Isometry3d::Identity());

    EXPECT_EQ(output.observed_count, 1);
    ASSERT_EQ(output.matched_blade_indices.size(), 1u);
    EXPECT_EQ(output.matched_blade_indices[0], 2);
    EXPECT_FALSE(output.solve_result.failed);
}

TEST(RuneCvGraphTest, RejectsObservationOutsideRollMatchGate) {
    filter_test::graph_optimizer::RuneTrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.smoother_lag = 0.0;
    config.initial_center = Eigen::Vector3d(0.0, 0.0, 5.0);
    config.normal_yaw = 0.0;
    config.normal_pitch = -M_PI_2;
    config.match_max_roll_diff = 0.05;
    config.camera_matrix = Eigen::Matrix3d::Identity();
    config.camera_matrix(0, 0) = 900.0;
    config.camera_matrix(1, 1) = 900.0;
    config.camera_matrix(0, 2) = 640.0;
    config.camera_matrix(1, 2) = 360.0;

    filter_test::graph_optimizer::RuneCvGraph graph(config);
    auto_aim_interfaces::msg::RuneTargets empty_msg;
    (void)graph.update(empty_msg, 0.01, Eigen::Isometry3d::Identity());

    const auto pixels = filter_test::graph_optimizer::projectRuneBladePixels(
        config.initial_center, config.normal_yaw, config.normal_pitch, 0.20,
        Eigen::Isometry3d::Identity(), config.camera_matrix, config.distortion);

    auto_aim_interfaces::msg::RuneTargets msg;
    msg.targets.push_back(makeRuneTargetFromPixels(pixels));

    const auto output = graph.update(msg, 0.01, Eigen::Isometry3d::Identity());

    EXPECT_EQ(output.observed_count, 0);
    EXPECT_TRUE(output.matched_blade_indices.empty());
    EXPECT_FALSE(output.solve_result.failed);
}

// ==================== 测试马氏距离 ====================

TEST(MahalanobisTest, ComputeDistance) {
    int dim = 2;

    Eigen::VectorXd innovation(dim);
    innovation << 1.0, 0.0;

    // 协方差矩阵（方差为4）
    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(dim, dim) * 4.0;

    // 马氏距离
    double mahalanobis_sq = innovation.transpose() * S.inverse() * innovation;
    double mahalanobis = std::sqrt(mahalanobis_sq);

    EXPECT_NEAR(mahalanobis, 0.5, 1e-10);  // sqrt(1/4) = 0.5
}

TEST(MahalanobisTest, CorrelatedCase) {
    int dim = 2;

    Eigen::VectorXd innovation(dim);
    innovation << 1.0, 1.0;

    // 相关的协方差矩阵
    Eigen::MatrixXd S(dim, dim);
    S << 1.0, 0.5,
         0.5, 1.0;

    // 马氏距离
    double mahalanobis_sq = innovation.transpose() * S.inverse() * innovation;

    // 验证是正数
    EXPECT_GT(mahalanobis_sq, 0.0);
}

// ==================== 测试运动模型线性化 ====================

TEST(LinearizationTest, SimpleCVModel) {
    // 简单的匀速运动模型
    int n = 2;
    double dt = 0.1;

    // 状态
    Eigen::VectorXd x(n);
    x << 1.0, 2.0;

    // 手动计算预测状态
    Eigen::VectorXd x_next(n);
    x_next[0] = x[0] + dt * x[1];  // 1.0 + 0.1 * 2.0 = 1.2
    x_next[1] = x[1];               // 2.0

    EXPECT_NEAR(x_next[0], 1.2, 1e-10);
    EXPECT_NEAR(x_next[1], 2.0, 1e-10);

    // 手动计算雅可比矩阵
    Eigen::MatrixXd F(n, n);
    F << 1, dt,
         0, 1;

    EXPECT_NEAR(F(0, 0), 1.0, 1e-10);
    EXPECT_NEAR(F(0, 1), 0.1, 1e-10);
    EXPECT_NEAR(F(1, 0), 0.0, 1e-10);
    EXPECT_NEAR(F(1, 1), 1.0, 1e-10);
}

TEST(LinearizationTest, AngleObservation) {
    // 角度观测模型：z = atan2(y, x)
    double x = 1.0;
    double y = 1.0;

    // 预测观测
    double z = std::atan2(y, x);
    EXPECT_NEAR(z, M_PI / 4.0, 1e-10);

    // 雅可比矩阵
    double r_sq = x * x + y * y;
    double dz_dx = -y / r_sq;  // -1/2
    double dz_dy = x / r_sq;   //  1/2

    EXPECT_NEAR(dz_dx, -0.5, 1e-10);
    EXPECT_NEAR(dz_dy, 0.5, 1e-10);
}

TEST(GraphOptimizerApiTest, DynamicAndStaticKeysUseExpectedFrames) {
    auto_graph::GraphOptimizer optimizer;
    auto center = optimizer.declareDynamic<gtsam::Point3>("center", 'x');
    auto yaw = optimizer.declareDynamic<gtsam::Rot2>("yaw", 'r');
    auto radius = optimizer.declareStatic<double>("radius_a", 'a');

    EXPECT_EQ(center.name, "center");
    EXPECT_EQ(yaw.name, "yaw");
    EXPECT_EQ(radius.name, "radius_a");
    EXPECT_EQ(optimizer.key(center, 3), gtsam::Symbol('x', 3));
    EXPECT_EQ(optimizer.key(yaw, 7), gtsam::Symbol('r', 7));
    EXPECT_EQ(optimizer.key(radius, 99), gtsam::Symbol('a', 0));
}

TEST(GraphOptimizerApiTest, WritesInitialAndFrameTypedValues) {
    auto_graph::GraphOptimizer optimizer;
    auto center = optimizer.declareDynamic<gtsam::Point3>("center", 'x');
    auto bias = optimizer.declareStatic<double>("bias", 'b');

    optimizer.beginInit();
    optimizer.addPrior(
        center, gtsam::Point3(0.0, 0.0, 0.0),
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
    optimizer.insert(bias, 2.0);
    optimizer.finishInit();

    EXPECT_EQ(optimizer.getFrameId(), 0u);
    EXPECT_EQ(optimizer.key(center), gtsam::Symbol('x', 0));
    EXPECT_EQ(optimizer.key(bias), gtsam::Symbol('b', 0));
    EXPECT_NEAR(optimizer.estimate<gtsam::Point3>(center).x(), 0.0, 1e-12);
    EXPECT_NEAR(optimizer.estimate<double>(bias), 2.0, 1e-12);

    const uint64_t frame_id = optimizer.beginFrame();
    optimizer.insert(bias, 2.0);
    optimizer.addPrior(center, gtsam::Point3(1.0, 2.0, 3.0),
                       gtsam::noiseModel::Isotropic::Sigma(3, 0.1));

    EXPECT_EQ(frame_id, 1u);
    EXPECT_EQ(optimizer.key(center), gtsam::Symbol('x', 1));
    EXPECT_EQ(optimizer.key(bias), gtsam::Symbol('b', 0));
    EXPECT_NEAR(optimizer.estimate<gtsam::Point3>(center).x(), 1.0, 1e-12);
    EXPECT_NEAR(optimizer.estimate<double>(bias), 2.0, 1e-12);
}

TEST(TypedGraphOptimizerTest, BeginFrameInsertedValueCanBeOptimized) {
    auto_graph::GraphOptimizerConfig config;
    config.cold_start_frames = 0;
    config.update_iterations = 1;
    config.verbose = false;

    auto_graph::GraphOptimizer optimizer(config);
    auto center = optimizer.declareDynamic<gtsam::Point3>("center", 'x');

    optimizer.beginInit();
    optimizer.addPrior(
        center, gtsam::Point3(0.0, 0.0, 0.0),
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
    optimizer.finishInit();

    optimizer.beginFrame();
    optimizer.addPrior(
        center, gtsam::Point3(1.0, 0.0, 0.0),
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1));

    auto result = optimizer.solve();

    ASSERT_TRUE(result.optimized);
    ASSERT_FALSE(result.failed);
    auto estimate = optimizer.estimate<gtsam::Point3>(center);
    EXPECT_NEAR(estimate.x(), 1.0, 1e-9);
}

TEST(TypedGraphOptimizerTest, FixedLagKeepsTypedStaticKeysTimestampFresh) {
    auto_graph::GraphOptimizerConfig config;
    config.cold_start_frames = 0;
    config.verbose = false;
    config.smoother_lag = 1.0;
    config.update_iterations = 1;

    auto_graph::GraphOptimizer optimizer(config);
    auto center = optimizer.declareDynamic<gtsam::Point3>("center", 'x');
    auto bias = optimizer.declareStatic<double>("bias", 'b');

    optimizer.beginInit();
    optimizer.addPrior(
        center, gtsam::Point3(0.0, 0.0, 0.0),
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
    optimizer.addPrior(
        bias, 2.0,
        gtsam::noiseModel::Isotropic::Sigma(1, 0.1));
    optimizer.finishInit();

    for (int frame_id = 1; frame_id <= 4; ++frame_id) {
        optimizer.beginFrame();
        optimizer.addPrior(
            center, gtsam::Point3(static_cast<double>(frame_id), 0.0, 0.0),
            gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
        optimizer.addPrior(
            bias, 2.0,
            gtsam::noiseModel::Isotropic::Sigma(1, 0.1));

        auto result = optimizer.solve();
        ASSERT_TRUE(result.optimized);
        ASSERT_FALSE(result.failed);
    }

    EXPECT_NEAR(optimizer.estimate<double>(bias), 2.0, 1e-9);
}

TEST(TypedMotionFactorTest, YawFactorWrapsAcrossPi) {
    filter_test::graph_optimizer::YawFactor factor(
        gtsam::noiseModel::Isotropic::Sigma(1, 1.0),
        gtsam::Symbol('r', 0), gtsam::Symbol('w', 0), gtsam::Symbol('r', 1), 0.0);

    auto err = factor.evaluateError(
        gtsam::Rot2::fromAngle(M_PI - 0.01),
        0.0,
        gtsam::Rot2::fromAngle(-M_PI + 0.01),
        nullptr, nullptr, nullptr);

    EXPECT_NEAR(std::abs(err[0]), 0.02, 1e-12);
}

TEST(TypedMotionFactorTest, VelocityFactorJacobiansHaveExpectedSigns) {
    filter_test::graph_optimizer::VelocityFactor factor(
        gtsam::noiseModel::Isotropic::Sigma(3, 1.0),
        gtsam::Symbol('v', 0), gtsam::Symbol('v', 1));

    gtsam::Matrix H1, H2;
    auto err = factor.evaluateError(
        gtsam::Vector3(1.0, 2.0, 3.0),
        gtsam::Vector3(1.5, 1.0, 4.0),
        &H1, &H2);

    EXPECT_TRUE(err.isApprox(gtsam::Vector3(0.5, -1.0, 1.0), 1e-12));
    EXPECT_TRUE(H1.isApprox(-gtsam::Matrix3::Identity(), 1e-12));
    EXPECT_TRUE(H2.isApprox(gtsam::Matrix3::Identity(), 1e-12));
}

TEST(TypedMotionFactorTest, MotionNoiseUsesConfiguredPerFrameSigmas) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.s2qxy = 0.08;
    config.s2qz = 0.02;
    config.s2qyaw = 0.18;
    config.vel_sigma = 0.04;
    config.vyaw_sigma = 0.05;

    const double dt = 0.02;
    const auto sigmas =
        filter_test::graph_optimizer::motionNoiseSigmasForConfig(config, false, dt);

    EXPECT_NEAR(sigmas.center_xy, std::sqrt(config.s2qxy), 1e-12);
    EXPECT_NEAR(sigmas.center_z, std::sqrt(config.s2qz), 1e-12);
    EXPECT_NEAR(sigmas.yaw, std::sqrt(config.s2qyaw), 1e-12);
    EXPECT_NEAR(sigmas.velocity_xy, config.vel_sigma, 1e-12);
    EXPECT_NEAR(sigmas.velocity_z, config.vel_sigma, 1e-12);
    EXPECT_NEAR(sigmas.vyaw, config.vyaw_sigma, 1e-12);
}

TEST(TypedArmorFactorTest, RadiusCenterZFactorUsesRot2YawResidual) {
    const double observed_yaw = -M_PI + 0.01;
    const double center_yaw = M_PI - 0.01;
    const double radius = filter_test::graph_optimizer::radiusToState(0.3);
    gtsam::Pose3 armor_pose(
        gtsam::Rot3::RzRyRx(0.0, 0.0, observed_yaw),
        gtsam::Point3(-0.3, 0.0, 0.5));
    gtsam::Point3 center(0.0, 0.0, 0.5);

    filter_test::graph_optimizer::ArmorRadiusCenterZFactor factor(
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Ones()),
        gtsam::Symbol('h', 0), gtsam::Symbol('a', 0), gtsam::Symbol('r', 0),
        gtsam::Symbol('x', 0), Eigen::Isometry3d::Identity(), 0, 0.10, 0.60);

    auto err = factor.evaluateError(
        armor_pose, radius, gtsam::Rot2::fromAngle(center_yaw), center,
        nullptr, nullptr, nullptr, nullptr);

    EXPECT_NEAR(std::abs(err[3]), 0.02, 1e-12);
}

TEST(TypedArmorFactorTest, RadiusCenterZFactorPoseJacobianMatchesFiniteDifference) {
    Eigen::Isometry3d T_camera_to_odom{Eigen::Isometry3d::Identity()};
    T_camera_to_odom.pretranslate(Eigen::Vector3d(0.3, -0.2, 0.15));
    T_camera_to_odom.rotate(
        Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(-0.12, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()));

    filter_test::graph_optimizer::ArmorRadiusCenterZFactor factor(
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Ones()),
        gtsam::Symbol('h', 0), gtsam::Symbol('a', 0), gtsam::Symbol('r', 0),
        gtsam::Symbol('x', 0), T_camera_to_odom, 0, 0.10, 0.60);

    const gtsam::Pose3 armor_pose(
        gtsam::Rot3::RzRyRx(0.08, 0.18, 0.6),
        gtsam::Point3(1.0, -2.0, 0.3));
    const double radius = filter_test::graph_optimizer::radiusToState(0.28);
    const gtsam::Rot2 center_yaw = gtsam::Rot2::fromAngle(-0.2);
    const gtsam::Point3 center(1.4, -1.5, 0.45);

    gtsam::Matrix H1;
    const auto err = factor.evaluateError(
        armor_pose, radius, center_yaw, center, &H1, nullptr, nullptr, nullptr);

    ASSERT_EQ(H1.rows(), 4);
    ASSERT_EQ(H1.cols(), 6);
    constexpr double eps = 1e-6;
    for (int col = 0; col < 6; ++col) {
        gtsam::Vector6 delta = gtsam::Vector6::Zero();
        delta(col) = eps;
        const auto err_perturbed = factor.evaluateError(
            armor_pose.retract(delta), radius, center_yaw, center,
            nullptr, nullptr, nullptr, nullptr);
        const auto numeric = (err_perturbed - err) / eps;
        EXPECT_TRUE(H1.col(col).isApprox(numeric, 1e-4)) << "col=" << col
            << " H=" << H1.col(col).transpose()
            << " numeric=" << numeric.transpose();
    }
}

TEST(TypedArmorFactorTest, RadiusDZFactorPoseJacobianMatchesFiniteDifference) {
    Eigen::Isometry3d T_camera_to_odom{Eigen::Isometry3d::Identity()};
    T_camera_to_odom.pretranslate(Eigen::Vector3d(-0.15, 0.25, 0.08));
    T_camera_to_odom.rotate(
        Eigen::AngleAxisd(-0.18, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitX()));

    filter_test::graph_optimizer::ArmorRadiusDZFactor factor(
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Ones()),
        gtsam::Symbol('h', 0), gtsam::Symbol('b', 0), gtsam::Symbol('z', 0),
        gtsam::Symbol('r', 0), gtsam::Symbol('x', 0),
        T_camera_to_odom, 1, 0.10, 0.60);

    const gtsam::Pose3 armor_pose(
        gtsam::Rot3::RzRyRx(-0.06, 0.12, -0.4),
        gtsam::Point3(-0.8, 1.2, 0.62));
    const double radius = filter_test::graph_optimizer::radiusToState(0.36);
    const double dz = 0.14;
    const gtsam::Rot2 center_yaw = gtsam::Rot2::fromAngle(-0.1);
    const gtsam::Point3 center(-0.45, 1.1, 0.5);

    gtsam::Matrix H1;
    const auto err = factor.evaluateError(
        armor_pose, radius, dz, center_yaw, center,
        &H1, nullptr, nullptr, nullptr, nullptr);

    ASSERT_EQ(H1.rows(), 4);
    ASSERT_EQ(H1.cols(), 6);
    constexpr double eps = 1e-6;
    for (int col = 0; col < 6; ++col) {
        gtsam::Vector6 delta = gtsam::Vector6::Zero();
        delta(col) = eps;
        const auto err_perturbed = factor.evaluateError(
            armor_pose.retract(delta), radius, dz, center_yaw, center,
            nullptr, nullptr, nullptr, nullptr, nullptr);
        const auto numeric = (err_perturbed - err) / eps;
        EXPECT_TRUE(H1.col(col).isApprox(numeric, 1e-4)) << "col=" << col
            << " H=" << H1.col(col).transpose()
            << " numeric=" << numeric.transpose();
    }
}

TEST(TypedArmorFactorTest, OutpostGeometryFactorConstrainsHeight) {
    filter_test::graph_optimizer::ArmorOutpostGeometryFactor factor(
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Ones()),
        gtsam::Symbol('h', 0), gtsam::Symbol('o', 0), gtsam::Symbol('p', 0),
        gtsam::Symbol('q', 0), gtsam::Symbol('s', 0), gtsam::Symbol('u', 0),
        gtsam::Symbol('r', 0), gtsam::Symbol('x', 0),
        Eigen::Isometry3d::Identity(), 1, 0.10, 0.60);

    const gtsam::Pose3 armor_pose(
        gtsam::Rot3::RzRyRx(0.0, 0.0, 0.4),
        gtsam::Point3(-0.2, 0.1, 2.3));
    const double radius = filter_test::graph_optimizer::radiusToState(0.2765);
    const double base_z = 1.5;
    const double dz1 = 0.15;
    const double dz2 = -0.1;
    const gtsam::Point2 base_xy(0.0, 0.0);
    const gtsam::Rot2 center_yaw = gtsam::Rot2::fromAngle(0.4);
    const gtsam::Point3 center(0.0, 0.0, 9.0);

    gtsam::Matrix H1, H3, H4, H5, H6, H8;
    const auto err = factor.evaluateError(
        armor_pose, radius, dz1, dz2, base_z, base_xy, center_yaw, center,
        &H1, nullptr, &H3, &H4, &H5, &H6, nullptr, &H8);

    // index=1: z_err = base_z + dz1 - armor.z = 1.5 + 0.15 - 2.3 = -0.65
    ASSERT_EQ(err.size(), 4);
    EXPECT_NEAR(err[2], base_z + 0.15 - 2.3, 1e-12);
    ASSERT_EQ(H1.rows(), 4);
    ASSERT_EQ(H3.rows(), 4);
    ASSERT_EQ(H4.rows(), 4);
    ASSERT_EQ(H5.rows(), 4);
    ASSERT_EQ(H6.rows(), 4);
    ASSERT_EQ(H8.rows(), 4);
    // H3 (dz1): index==1 时 z 行导数为 1
    EXPECT_NEAR(H3(2, 0), 1.0, 1e-12);
    // H4 (dz2): index!=2 时 z 行导数为 0
    EXPECT_NEAR(H4(2, 0), 0.0, 1e-12);
    // H5 (base_z): 高度基准承担 z 残差。
    EXPECT_NEAR(H5(2, 0), 1.0, 1e-12);
    // H6 (base_xy): 平面几何由静态基准中心承担。
    const double armor_yaw =
        center_yaw.theta() + filter_test::graph_optimizer::armorYawOffset(1, 3);
    EXPECT_NEAR(H6(0, 0), -std::sin(armor_yaw), 1e-12);
    EXPECT_NEAR(H6(1, 0), std::cos(armor_yaw), 1e-12);
    // H8 (center): outpost 几何不再拉动态 center.xyz。
    EXPECT_TRUE(H8.isZero(1e-12));
}

TEST(TypedArmorFactorTest, OutpostGeometryFactorPlanarResidualIgnoresPoseYawBranch) {
    filter_test::graph_optimizer::ArmorOutpostGeometryFactor factor(
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Ones()),
        gtsam::Symbol('h', 0), gtsam::Symbol('o', 0), gtsam::Symbol('p', 0),
        gtsam::Symbol('q', 0), gtsam::Symbol('s', 0), gtsam::Symbol('u', 0),
        gtsam::Symbol('r', 0), gtsam::Symbol('x', 0),
        Eigen::Isometry3d::Identity(), 1, 0.10, 0.60);

    const double center_yaw_value = 0.4;
    const double armor_yaw =
        center_yaw_value + filter_test::graph_optimizer::armorYawOffset(1, 3);
    const double radius_value = 0.2765;
    const double base_z = 1.5;
    const double dz1 = 0.15;
    const double dz2 = -0.1;
    const gtsam::Point2 base_xy(1.0, 2.0);
    const gtsam::Point3 center(1.0, 2.0, 9.0);
    const gtsam::Point3 armor_position(
        center.x() - radius_value * std::cos(armor_yaw),
        center.y() - radius_value * std::sin(armor_yaw),
        base_z + dz1);
    const double radius =
        filter_test::graph_optimizer::radiusToState(radius_value);
    const gtsam::Rot2 center_yaw = gtsam::Rot2::fromAngle(center_yaw_value);

    const gtsam::Pose3 correct_pose(
        gtsam::Rot3::RzRyRx(0.0, 0.0, armor_yaw),
        armor_position);
    const gtsam::Pose3 wrong_yaw_pose(
        gtsam::Rot3::RzRyRx(0.0, 0.0, armor_yaw + M_PI),
        armor_position);

    const auto correct_err = factor.evaluateError(
        correct_pose, radius, dz1, dz2, base_z, base_xy, center_yaw, center,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    const auto wrong_yaw_err = factor.evaluateError(
        wrong_yaw_pose, radius, dz1, dz2, base_z, base_xy, center_yaw, center,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

    EXPECT_NEAR(correct_err[0], 0.0, 1e-12);
    EXPECT_NEAR(correct_err[1], 0.0, 1e-12);
    EXPECT_NEAR(correct_err[2], 0.0, 1e-12);
    EXPECT_NEAR(wrong_yaw_err[0], 0.0, 1e-12);
    EXPECT_NEAR(wrong_yaw_err[1], 0.0, 1e-12);
    EXPECT_NEAR(wrong_yaw_err[2], 0.0, 1e-12);
    EXPECT_GT(std::abs(wrong_yaw_err[3]), 1.0);
}

TEST(TypedArmorFactorTest, EdgeCenterZReprojFactorIsZeroForObservedProjection) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 900.0;
    K(1, 1) = 900.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const Eigen::Vector3d center(0.0, 0.0, 4.0);
    const double yaw = 0.25;
    const double radius = 0.30;
    const int armor_index = 0;
    const double armor_pitch = 15.0 * M_PI / 180.0;
    const auto pixels = filter_test::graph_optimizer::projectArmorPixels(
        center, yaw, radius, 0.0, armor_index, armor_pitch,
        Eigen::Isometry3d::Identity(), K, dist);

    filter_test::graph_optimizer::ArmorEdgeCenterZReprojFactor factor(
        auto_graph::isotropicNoise(4, 1.0),
        gtsam::Symbol('a', 0), gtsam::Symbol('r', 0), gtsam::Symbol('x', 0),
        Eigen::Isometry3d::Identity(), armor_index, armor_pitch, K, dist, pixels);

    const auto residual = factor.evaluateError(
        filter_test::graph_optimizer::radiusToState(radius),
        gtsam::Rot2::fromAngle(yaw),
        auto_graph::eigenToPoint3(center),
        nullptr, nullptr, nullptr);

    ASSERT_EQ(residual.size(), 4);
    EXPECT_TRUE(residual.isZero(1e-9)) << residual.transpose();
}

TEST(TypedArmorFactorTest, EdgeCenterZReprojFactorRespondsToYawChange) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 900.0;
    K(1, 1) = 900.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const Eigen::Vector3d center(0.0, 0.0, 4.0);
    const double yaw = 0.25;
    const double radius = 0.30;
    const int armor_index = 0;
    const double armor_pitch = 15.0 * M_PI / 180.0;
    const auto pixels = filter_test::graph_optimizer::projectArmorPixels(
        center, yaw, radius, 0.0, armor_index, armor_pitch,
        Eigen::Isometry3d::Identity(), K, dist);

    filter_test::graph_optimizer::ArmorEdgeCenterZReprojFactor factor(
        auto_graph::isotropicNoise(4, 1.0),
        gtsam::Symbol('a', 0), gtsam::Symbol('r', 0), gtsam::Symbol('x', 0),
        Eigen::Isometry3d::Identity(), armor_index, armor_pitch, K, dist, pixels);

    const auto residual = factor.evaluateError(
        filter_test::graph_optimizer::radiusToState(radius),
        gtsam::Rot2::fromAngle(yaw + 0.10),
        auto_graph::eigenToPoint3(center),
        nullptr, nullptr, nullptr);

    ASSERT_EQ(residual.size(), 4);
    EXPECT_GT(residual.cwiseAbs().sum(), 0.05);
}

TEST(TypedArmorFactorTest, EdgeDZReprojFactorRespondsToDzChange) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 900.0;
    K(1, 1) = 900.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const Eigen::Vector3d center(0.1, -0.2, 4.0);
    const double yaw = -0.30;
    const double radius = 0.36;
    const double dz = 0.14;
    const int armor_index = 1;
    const double armor_pitch = 15.0 * M_PI / 180.0;
    const auto pixels = filter_test::graph_optimizer::projectArmorPixels(
        center, yaw, radius, dz, armor_index, armor_pitch,
        Eigen::Isometry3d::Identity(), K, dist);

    filter_test::graph_optimizer::ArmorEdgeDZReprojFactor factor(
        auto_graph::isotropicNoise(4, 1.0),
        gtsam::Symbol('b', 0), gtsam::Symbol('z', 0), gtsam::Symbol('r', 0),
        gtsam::Symbol('x', 0), Eigen::Isometry3d::Identity(), armor_index,
        armor_pitch, K, dist, pixels);

    const auto zero_residual = factor.evaluateError(
        filter_test::graph_optimizer::radiusToState(radius), dz,
        gtsam::Rot2::fromAngle(yaw),
        auto_graph::eigenToPoint3(center),
        nullptr, nullptr, nullptr, nullptr);
    const auto dz_residual = factor.evaluateError(
        filter_test::graph_optimizer::radiusToState(radius), dz + 0.08,
        gtsam::Rot2::fromAngle(yaw),
        auto_graph::eigenToPoint3(center),
        nullptr, nullptr, nullptr, nullptr);

    ASSERT_EQ(zero_residual.size(), 4);
    EXPECT_TRUE(zero_residual.isZero(1e-9)) << zero_residual.transpose();
    EXPECT_GT(dz_residual.cwiseAbs().sum(), 0.02);
}

TEST(TypedArmorFactorTest, EdgeCenterZReprojFactorAcceptsOutpostPitch) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 900.0;
    K(1, 1) = 900.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const Eigen::Vector3d center(0.0, 0.0, 4.0);
    const double yaw = 0.15;
    const double radius = 0.2765;
    const int armor_index = 2;
    const double outpost_pitch = -0.26;
    const auto pixels = filter_test::graph_optimizer::projectArmorPixels(
        center, yaw, radius, 0.0, armor_index, outpost_pitch,
        Eigen::Isometry3d::Identity(), K, dist);

    filter_test::graph_optimizer::ArmorEdgeCenterZReprojFactor factor(
        auto_graph::isotropicNoise(4, 1.0),
        gtsam::Symbol('a', 0), gtsam::Symbol('r', 0), gtsam::Symbol('x', 0),
        Eigen::Isometry3d::Identity(), armor_index, outpost_pitch, K, dist, pixels);

    const auto residual = factor.evaluateError(
        filter_test::graph_optimizer::radiusToState(radius),
        gtsam::Rot2::fromAngle(yaw),
        auto_graph::eigenToPoint3(center),
        nullptr, nullptr, nullptr);

    ASSERT_EQ(residual.size(), 4);
    EXPECT_TRUE(residual.isZero(1e-9)) << residual.transpose();
}

TEST(TypedArmorFactorTest, EdgeOutpostReprojFactorIsZeroForThirdPlateProjection) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 900.0;
    K(1, 1) = 900.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const Eigen::Vector3d center(0.0, 0.0, 4.0);
    const Eigen::Vector3d dynamic_center(1.2, -0.8, 9.0);
    const double yaw = 0.15;
    const double radius = 0.2765;
    const double dz = -0.1;
    const int armor_index = 2;
    const double outpost_pitch = -0.26;
    const auto pixels = filter_test::graph_optimizer::projectArmorPixels(
        center, yaw, radius, dz, armor_index, outpost_pitch,
        Eigen::Isometry3d::Identity(), K, dist, 3);

    filter_test::graph_optimizer::ArmorEdgeOutpostReprojFactor factor(
        auto_graph::isotropicNoise(4, 1.0),
        gtsam::Symbol('o', 0), gtsam::Symbol('p', 0), gtsam::Symbol('q', 0),
        gtsam::Symbol('s', 0), gtsam::Symbol('u', 0), gtsam::Symbol('r', 0),
        gtsam::Symbol('x', 0), Eigen::Isometry3d::Identity(), armor_index,
        outpost_pitch, K, dist, pixels);

    const auto residual = factor.evaluateError(
        filter_test::graph_optimizer::radiusToState(radius),
        0.2, dz, center.z(),
        gtsam::Point2(center.x(), center.y()),
        gtsam::Rot2::fromAngle(yaw),
        auto_graph::eigenToPoint3(dynamic_center),
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

    ASSERT_EQ(residual.size(), 4);
    EXPECT_TRUE(residual.isZero(1e-9)) << residual.transpose();
}

TEST(TypedArmorFactorTest, EdgeOutpostYawReprojFactorOnlyDependsOnYaw) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 900.0;
    K(1, 1) = 900.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const Eigen::Vector3d center(0.0, 0.0, 4.0);
    const double yaw = 0.15;
    const double radius = 0.2765;
    const double dz = -0.1;
    const int armor_index = 2;
    const double outpost_pitch = -0.26;
    const auto pixels = filter_test::graph_optimizer::projectArmorPixels(
        center, yaw, radius, dz, armor_index, outpost_pitch,
        Eigen::Isometry3d::Identity(), K, dist, 3);

    filter_test::graph_optimizer::ArmorEdgeYawReprojFactor factor(
        auto_graph::isotropicNoise(4, 1.0),
        gtsam::Symbol('r', 0), center, radius, dz, armor_index,
        outpost_pitch, Eigen::Isometry3d::Identity(), K, dist, pixels,
        2.0, 3);

    gtsam::Matrix H;
    const auto zero_residual =
        factor.evaluateError(gtsam::Rot2::fromAngle(yaw), &H);
    const auto yaw_residual =
        factor.evaluateError(gtsam::Rot2::fromAngle(yaw + 0.10), nullptr);

    ASSERT_EQ(zero_residual.size(), 4);
    EXPECT_TRUE(zero_residual.isZero(1e-9)) << zero_residual.transpose();
    ASSERT_EQ(H.rows(), 4);
    ASSERT_EQ(H.cols(), 1);
    EXPECT_GT(yaw_residual.cwiseAbs().sum(), 0.02);
}

TEST(TypedArmorFactorTest, EdgeYawReprojIgnoresCommonPixelTranslation) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 900.0;
    K(1, 1) = 900.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const Eigen::Vector3d center(0.0, 0.0, 4.0);
    const double yaw = 0.15;
    const double radius = 0.2765;
    const double dz = 0.0;
    const int armor_index = 0;
    const double armor_pitch = -0.26;
    auto observed_pixels = filter_test::graph_optimizer::projectArmorPixels(
        center, yaw, radius, dz, armor_index, armor_pitch,
        Eigen::Isometry3d::Identity(), K, dist, 4);
    for (auto& pixel : observed_pixels) {
        pixel += Eigen::Vector2d(8.0, -5.0);
    }

    filter_test::graph_optimizer::ArmorEdgeYawReprojFactor factor(
        auto_graph::isotropicNoise(4, 1.0),
        gtsam::Symbol('r', 0), center, radius, dz, armor_index,
        armor_pitch, Eigen::Isometry3d::Identity(), K, dist,
        observed_pixels, 2.0, 4);

    const auto residual =
        factor.evaluateError(gtsam::Rot2::fromAngle(yaw), nullptr);

    ASSERT_EQ(residual.size(), 4);
    EXPECT_TRUE(residual.isZero(1e-9)) << residual.transpose();
}

TEST(TypedArmorFactorTest, EdgeYawReprojSmallYawPerturbationScalesLinearly) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 900.0;
    K(1, 1) = 900.0;
    K(0, 2) = 640.0;
    K(1, 2) = 360.0;
    std::array<double, 5> dist{0.0, 0.0, 0.0, 0.0, 0.0};

    const Eigen::Vector3d center(0.0, 0.0, 4.0);
    const double yaw = 0.15;
    const double radius = 0.2765;
    const double dz = 0.0;
    const int armor_index = 0;
    const double armor_pitch = -0.26;
    const auto observed_pixels = filter_test::graph_optimizer::projectArmorPixels(
        center, yaw, radius, dz, armor_index, armor_pitch,
        Eigen::Isometry3d::Identity(), K, dist, 4);

    filter_test::graph_optimizer::ArmorEdgeYawReprojFactor factor(
        auto_graph::isotropicNoise(4, 1.0),
        gtsam::Symbol('r', 0), center, radius, dz, armor_index,
        armor_pitch, Eigen::Isometry3d::Identity(), K, dist,
        observed_pixels, 2.0, 4);

    const auto small_residual =
        factor.evaluateError(gtsam::Rot2::fromAngle(yaw + 0.005), nullptr);
    const auto large_residual =
        factor.evaluateError(gtsam::Rot2::fromAngle(yaw + 0.010), nullptr);

    const double ratio =
        large_residual.cwiseAbs().sum() / small_residual.cwiseAbs().sum();
    EXPECT_GT(ratio, 1.75);
    EXPECT_LT(ratio, 2.25);
}


namespace {

auto_graph::GraphOptimizer makeTypedPointOptimizer(int cold_start_frames) {
    auto_graph::GraphOptimizerConfig config;
    config.cold_start_frames = cold_start_frames;
    config.verbose = false;

    auto_graph::GraphOptimizer optimizer(config);
    auto center = optimizer.declareDynamic<gtsam::Point3>("center", 'x');
    optimizer.beginInit();
    optimizer.addPrior(
        center, gtsam::Point3(0.0, 0.0, 0.0),
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
    optimizer.finishInit();
    return optimizer;
}

}  // namespace

TEST(GraphOptimizerSolveResultTest, UninitializedOptimizerReportsNoAttempt) {
    auto_graph::GraphOptimizer optimizer;

    auto result = optimizer.solve();

    EXPECT_EQ(result.frame_id, 0u);
    EXPECT_FALSE(result.attempted);
    EXPECT_FALSE(result.optimized);
    EXPECT_FALSE(result.cold_start);
    EXPECT_FALSE(result.failed);
    EXPECT_TRUE(result.error_message.empty());
}

TEST(GraphOptimizerSolveResultTest, ColdStartReportsAccumulationWithoutFailure) {
    auto optimizer = makeTypedPointOptimizer(1);

    auto result = optimizer.solve();

    EXPECT_EQ(result.frame_id, 0u);
    EXPECT_FALSE(result.attempted);
    EXPECT_FALSE(result.optimized);
    EXPECT_TRUE(result.cold_start);
    EXPECT_FALSE(result.failed);
    EXPECT_TRUE(result.error_message.empty());
}

TEST(GraphOptimizerSolveResultTest, InvalidFactorReportsFailure) {
    auto optimizer = makeTypedPointOptimizer(0);
    optimizer.beginFrame();
    optimizer.addAuxPrior<gtsam::Pose3>(
        gtsam::Symbol('x', 1),
        gtsam::Pose3(),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Ones()));

    auto result = optimizer.solve();

    EXPECT_EQ(result.frame_id, 1u);
    EXPECT_TRUE(result.attempted);
    EXPECT_FALSE(result.optimized);
    EXPECT_FALSE(result.cold_start);
    EXPECT_TRUE(result.failed);
    EXPECT_FALSE(result.error_message.empty());
}

TEST(GraphOptimizerVisualizationTest, RadiusConversionUsesTrackerRadiusBounds) {
    EXPECT_NEAR(filter_test::graphOptimizerRadiusFromState(0.0), 0.35, 1e-12);
}

TEST(GraphOptimizerVisualizationTest, ObservedArmorMarkerPitchMatchesSimulation) {
    tf2::Matrix3x3 standard_rotation(
        filter_test::observedArmorMarkerQuaternion(0.0, false));
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    standard_rotation.getRPY(roll, pitch, yaw);

    EXPECT_NEAR(roll, 0.0, 1e-12);
    EXPECT_NEAR(pitch, 15.0 * M_PI / 180.0, 1e-12);
    EXPECT_NEAR(yaw, 0.0, 1e-12);

    tf2::Matrix3x3 outpost_rotation(
        filter_test::observedArmorMarkerQuaternion(0.3, true));
    outpost_rotation.getRPY(roll, pitch, yaw);

    EXPECT_NEAR(roll, 0.0, 1e-12);
    EXPECT_NEAR(pitch, -0.26, 1e-12);
    EXPECT_NEAR(yaw, 0.3, 1e-12);
}

TEST(GraphOptimizerVisualizationTest, OutpostTrackerTargetUsesAutoaimDzMapping) {
    filter_test::graph_optimizer::TrackerState state;
    state.armor_count = 3;
    state.radius_1 = 0.2765;
    state.radius_2 = 0.2765;
    state.dz = 0.2;
    state.outpost_dz_2 = -0.1;

    auto_aim_interfaces::msg::EnemyInfo enemy;
    filter_test::fillTrackerTargetEnemyFromState(state, enemy);

    EXPECT_NEAR(enemy.radius_1, 0.2765, 1e-12);
    EXPECT_NEAR(enemy.radius_2, 0.2765, 1e-12);
    EXPECT_NEAR(enemy.dz, 0.2, 1e-12);
    EXPECT_NEAR(enemy.dz2, -0.1, 1e-12);
}

TEST(GraphOptimizerTrackerCoreTest, PredictedArmorsUseCenterYawRadiusAndDz) {
    filter_test::graph_optimizer::TrackerState state;
    state.center << 1.0, 2.0, 0.5;
    state.yaw = 0.0;
    state.radius_1 = 0.3;
    state.radius_2 = 0.4;
    state.dz = 0.1;

    auto armors = filter_test::graph_optimizer::predictedArmorsFromState(state);
    ASSERT_EQ(armors.size(), 4u);

    EXPECT_NEAR(armors[0].position.x(), 0.7, 1e-12);
    EXPECT_NEAR(armors[0].position.y(), 2.0, 1e-12);
    EXPECT_NEAR(armors[0].position.z(), 0.5, 1e-12);
    EXPECT_NEAR(armors[1].position.x(), 1.0, 1e-12);
    EXPECT_NEAR(armors[1].position.y(), 1.6, 1e-12);
    EXPECT_NEAR(armors[1].position.z(), 0.6, 1e-12);
    EXPECT_NEAR(armors[2].position.x(), 1.3, 1e-12);
    EXPECT_NEAR(armors[2].position.y(), 2.0, 1e-12);
    EXPECT_NEAR(armors[3].position.x(), 1.0, 1e-12);
    EXPECT_NEAR(armors[3].position.y(), 2.4, 1e-12);
}

TEST(GraphOptimizerTrackerCoreTest, PredictedOutpostArmorsUseThreePlateGeometry) {
    filter_test::graph_optimizer::TrackerState state;
    state.armor_count = 3;
    state.center << 1.0, 2.0, 0.5;
    state.yaw = 0.1;
    state.radius_1 = 0.2765;
    state.radius_2 = 0.2765;
    state.dz = 0.2;
    state.outpost_dz_2 = -0.1;

    const auto armors =
        filter_test::graph_optimizer::predictedArmorsFromState(state);

    ASSERT_EQ(armors.size(), 3u);
    for (int i = 0; i < 3; ++i) {
        const double ay = state.yaw +
            filter_test::graph_optimizer::armorYawOffset(i, 3);
        const double dz = i == 0 ? 0.0 : (i == 1 ? state.dz : state.outpost_dz_2);
        EXPECT_EQ(armors[static_cast<std::size_t>(i)].index, i);
        EXPECT_NEAR(armors[static_cast<std::size_t>(i)].yaw, ay, 1e-12);
        EXPECT_NEAR(
            armors[static_cast<std::size_t>(i)].position.x(),
            state.center.x() - state.radius_1 * std::cos(ay), 1e-12);
        EXPECT_NEAR(
            armors[static_cast<std::size_t>(i)].position.y(),
            state.center.y() - state.radius_1 * std::sin(ay), 1e-12);
        EXPECT_NEAR(
            armors[static_cast<std::size_t>(i)].position.z(),
            state.center.z() + dz, 1e-12);
    }
}

TEST(GraphOptimizerTrackerCoreTest, MatchArmorKeepsYawContinuityAcrossPi) {
    filter_test::graph_optimizer::TrackerState state;
    state.center << 0.0, 0.0, 0.5;
    state.yaw = M_PI - 0.05;
    state.radius_1 = 0.3;
    state.radius_2 = 0.4;
    state.dz = 0.1;

    auto pred = filter_test::graph_optimizer::predictedArmorsFromState(state).front();
    auto_aim_interfaces::msg::Armor obs;
    obs.pose.position.x = pred.position.x();
    obs.pose.position.y = pred.position.y();
    obs.pose.position.z = pred.position.z();
    obs.yaw = -M_PI + 0.03;

    int matched = filter_test::graph_optimizer::matchArmorIndex(state, obs, -1);
    EXPECT_EQ(matched, 0);
}

TEST(GraphOptimizerTrackerCoreTest, MatchOutpostArmorsAssignsThreeUniqueIndices) {
    filter_test::graph_optimizer::TrackerState state;
    state.armor_count = 3;
    state.center << 0.0, 0.0, 0.5;
    state.yaw = -0.2;
    state.radius_1 = 0.2765;
    state.radius_2 = 0.2765;
    state.dz = 0.2;
    state.outpost_dz_2 = -0.1;

    const auto predicted =
        filter_test::graph_optimizer::predictedArmorsFromState(state);
    auto_aim_interfaces::msg::Armors observations;
    for (const auto& pred : predicted) {
        auto_aim_interfaces::msg::Armor obs;
        obs.number = "outpost";
        obs.pose.position.x = pred.position.x();
        obs.pose.position.y = pred.position.y();
        obs.pose.position.z = pred.position.z();
        obs.yaw = pred.yaw;
        observations.armors.push_back(obs);
    }

    const auto matched = filter_test::graph_optimizer::matchArmorIndicesUnique(
        state, observations, -1, 100.0);

    ASSERT_EQ(matched.size(), 3u);
    EXPECT_EQ(std::set<int>(matched.begin(), matched.end()).size(), 3u);
    EXPECT_EQ(*std::min_element(matched.begin(), matched.end()), 0);
    EXPECT_EQ(*std::max_element(matched.begin(), matched.end()), 2);
}

TEST(GraphOptimizerTrackerCoreTest, MatchOutpostArmorIgnoresPriorityIndex) {
    filter_test::graph_optimizer::TrackerState state;
    state.armor_count = 3;
    state.center << 0.0, 0.0, 0.5;
    state.yaw = 0.0;
    state.radius_1 = 0.2765;
    state.radius_2 = 0.2765;
    state.dz = 0.2;
    state.outpost_dz_2 = -0.1;

    const auto predicted =
        filter_test::graph_optimizer::predictedArmorsFromState(state);
    auto_aim_interfaces::msg::Armor obs;
    obs.number = "outpost";
    obs.priority = 2;
    obs.pose.position.x = predicted[0].position.x();
    obs.pose.position.y = predicted[0].position.y();
    obs.pose.position.z = predicted[0].position.z();
    obs.yaw = predicted[0].yaw;

    auto_aim_interfaces::msg::Armors observations;
    observations.armors.push_back(obs);

    const auto matched = filter_test::graph_optimizer::matchArmorIndicesUnique(
        state, observations, -1, 100.0);

    ASSERT_EQ(matched.size(), 1u);
    EXPECT_EQ(matched[0], 0);
}

TEST(GraphOptimizerTrackerCoreTest, MatchOutpostArmorUsesPositionWhenYawDisagrees) {
    filter_test::graph_optimizer::TrackerState state;
    state.armor_count = 3;
    state.center << 0.0, 0.0, 0.0;
    state.yaw = 0.0;
    state.radius_1 = 0.5;
    state.radius_2 = 0.5;
    state.dz = 0.2;
    state.outpost_dz_2 = -0.1;

    const auto predicted =
        filter_test::graph_optimizer::predictedArmorsFromState(state);
    auto_aim_interfaces::msg::Armor obs;
    obs.number = "outpost";
    obs.pose.position.x = predicted[0].position.x();
    obs.pose.position.y = predicted[0].position.y();
    obs.pose.position.z = predicted[0].position.z();
    obs.yaw = predicted[2].yaw;

    auto_aim_interfaces::msg::Armors observations;
    observations.armors.push_back(obs);

    const auto matched = filter_test::graph_optimizer::matchArmorIndicesUnique(
        state, observations, -1, 100.0);

    ASSERT_EQ(matched.size(), 1u);
    EXPECT_EQ(matched[0], 0);
}

TEST(GraphOptimizerTrackerCoreTest, MatchOutpostArmorIgnoresCenterBearingJitter) {
    filter_test::graph_optimizer::TrackerState state;
    state.armor_count = 3;
    state.center << 1.0, 2.0, 1.5;
    state.yaw = 0.2;
    state.radius_1 = 0.2765;
    state.radius_2 = 0.2765;
    state.dz = 0.2;
    state.outpost_dz_2 = -0.1;

    const auto predicted =
        filter_test::graph_optimizer::predictedArmorsFromState(state);
    auto_aim_interfaces::msg::Armor obs;
    obs.number = "outpost";
    obs.pose.position.x = predicted[1].position.x();
    obs.pose.position.y = predicted[1].position.y();
    obs.pose.position.z = predicted[1].position.z();
    obs.yaw = predicted[0].yaw;

    auto_aim_interfaces::msg::Armors observations;
    observations.armors.push_back(obs);

    const auto matched = filter_test::graph_optimizer::matchArmorIndicesUnique(
        state, observations, -1, 100.0);

    ASSERT_EQ(matched.size(), 1u);
    EXPECT_EQ(matched[0], 0);
}

TEST(GraphOptimizerTrackerCoreTest, MatchOutpostArmorRejectsCombinedCostAboveGate) {
    filter_test::graph_optimizer::TrackerState state;
    state.armor_count = 3;
    state.center << 0.0, 0.0, 0.0;
    state.yaw = 0.0;
    state.radius_1 = 0.2765;
    state.radius_2 = 0.2765;
    state.dz = 0.2;
    state.outpost_dz_2 = -0.1;

    const auto predicted =
        filter_test::graph_optimizer::predictedArmorsFromState(state);
    const auto midpoint =
        0.5 * (predicted[0].position + predicted[1].position);
    const double mid_yaw =
        filter_test::graph_optimizer::armorYawOffset(1, 3) * 0.5 + 0.02;

    auto_aim_interfaces::msg::Armor obs;
    obs.number = "outpost";
    obs.pose.position.x = midpoint.x();
    obs.pose.position.y = midpoint.y();
    obs.pose.position.z = midpoint.z();
    obs.yaw = mid_yaw;

    auto_aim_interfaces::msg::Armors observations;
    observations.armors.push_back(obs);

    const auto matched = filter_test::graph_optimizer::matchArmorIndicesUnique(
        state, observations, -1);

    ASSERT_EQ(matched.size(), 1u);
    EXPECT_EQ(matched[0], -1);
}

TEST(GraphOptimizerTrackerCoreTest, MatchOutpostArmorUsesPositionLikeStandardModel) {
    filter_test::graph_optimizer::TrackerState state;
    state.armor_count = 3;
    state.center << 0.0, 0.0, 0.0;
    state.yaw = 0.0;
    state.radius_1 = 0.2765;
    state.radius_2 = 0.2765;
    state.dz = 0.2;
    state.outpost_dz_2 = -0.1;

    const auto predicted =
        filter_test::graph_optimizer::predictedArmorsFromState(state);
    ASSERT_GE(predicted.size(), 2u);

    auto_aim_interfaces::msg::Armor obs;
    obs.number = "outpost";
    obs.pose.position.x = predicted[1].position.x();
    obs.pose.position.y = predicted[1].position.y();
    obs.pose.position.z = predicted[1].position.z();
    obs.yaw = 0.80;

    auto_aim_interfaces::msg::Armors observations;
    observations.armors.push_back(obs);

    const auto matched = filter_test::graph_optimizer::matchArmorIndicesUnique(
        state, observations, -1, 100.0);

    ASSERT_EQ(matched.size(), 1u);
    EXPECT_EQ(matched[0], 1);
}


TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphInitializeSetsStateAndFrameId) {
    filter_test::graph_optimizer::TrackerConfig config;
    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor obs;
    obs.pose.position.x = 1.0;
    obs.pose.position.y = 0.0;
    obs.pose.position.z = 0.5;
    obs.pose.orientation.w = 1.0;
    obs.yaw = 0.0;
    observations.armors.push_back(obs);

    graph.initialize(observations);

    EXPECT_TRUE(graph.initialized());
    EXPECT_EQ(graph.frameId(), 0u);
    EXPECT_NEAR(graph.state().center.x(), 1.25, 1e-12);
    EXPECT_NEAR(graph.state().center.y(), 0.0, 1e-12);
    EXPECT_NEAR(graph.state().center.z(), 0.5, 1e-12);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphInitializesOutpostState) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.outpost_radius = 0.2765;
    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);

    const Eigen::Vector3d center(1.2765, 0.0, 0.5);
    constexpr double dz1 = 0.15;
    constexpr double dz2 = -0.06;
    auto make_obs = [&](int index, double dz) {
        auto_aim_interfaces::msg::Armor armor;
        const double yaw =
            filter_test::graph_optimizer::armorYawOffset(index, 3);
        armor.number = "outpost";
        armor.pose.position.x =
            center.x() - config.outpost_radius * std::cos(yaw);
        armor.pose.position.y =
            center.y() - config.outpost_radius * std::sin(yaw);
        armor.pose.position.z = center.z() + dz;
        armor.pose.orientation.w = 1.0;
        armor.yaw = yaw;
        return armor;
    };

    auto_aim_interfaces::msg::Armors observations;
    observations.armors.push_back(make_obs(0, 0.0));
    observations.armors.push_back(make_obs(2, dz2));
    observations.armors.push_back(make_obs(1, dz1));

    graph.initialize(observations);

    EXPECT_TRUE(graph.initialized());
    EXPECT_EQ(graph.state().armor_count, 3);
    EXPECT_NEAR(graph.state().center.x(), 1.2765, 1e-12);
    EXPECT_NEAR(graph.state().center.z(), center.z(), 1e-12);
    EXPECT_NEAR(graph.state().radius_1, 0.2765, 1e-12);
    EXPECT_NEAR(graph.state().radius_2, 0.2765, 1e-12);
    EXPECT_NEAR(graph.state().dz, dz1, 1e-12);
    EXPECT_NEAR(graph.state().outpost_dz_2, dz2, 1e-12);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphKeepsYawContinuousAcrossPi) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.geo_noise.tangential = 0.001;
    config.geo_noise.radial = 0.001;
    config.geo_noise.height = 0.001;
    config.geo_noise.yaw = 0.001;
    config.use_edge_reproj_factor = false;

    constexpr double radius = 0.25;
    const Eigen::Vector3d center(1.0, 0.0, 0.5);
    const double init_yaw = M_PI - 0.03;
    const double observed_yaw = M_PI + 0.04;

    auto make_obs = [&](double armor_yaw) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor armor;
        armor.pose.position.x = center.x() - radius * std::cos(armor_yaw);
        armor.pose.position.y = center.y() - radius * std::sin(armor_yaw);
        armor.pose.position.z = center.z();
        armor.pose.orientation.z = std::sin(armor_yaw / 2.0);
        armor.pose.orientation.w = std::cos(armor_yaw / 2.0);
        armor.yaw = armor_yaw;
        observations.armors.push_back(armor);
        return observations;
    };

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(make_obs(init_yaw));

    const auto output =
        graph.update(make_obs(observed_yaw), 0.01, Eigen::Isometry3d::Identity());

    ASSERT_FALSE(output.solve_result.failed);
    EXPECT_NEAR(output.state.yaw, observed_yaw, 0.05);
    EXPECT_GT(output.state.yaw, M_PI);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostInitUsesMostCentralSlotZero) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.outpost_radius = 0.2765;
    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);

    const Eigen::Vector3d center(1.2765, 0.0, 0.5);
    constexpr double dz1 = 0.15;
    constexpr double dz2 = -0.06;
    auto make_obs = [&](int index, double dz) {
        auto_aim_interfaces::msg::Armor armor;
        const double yaw =
            filter_test::graph_optimizer::armorYawOffset(index, 3);
        armor.number = "outpost";
        armor.pose.position.x =
            center.x() - config.outpost_radius * std::cos(yaw);
        armor.pose.position.y =
            center.y() - config.outpost_radius * std::sin(yaw);
        armor.pose.position.z = center.z() + dz;
        armor.pose.orientation.w = 1.0;
        armor.yaw = yaw;
        return armor;
    };

    auto_aim_interfaces::msg::Armors observations;
    observations.armors.push_back(make_obs(2, dz2));
    observations.armors.push_back(make_obs(0, 0.0));
    observations.armors.push_back(make_obs(1, dz1));

    graph.initialize(observations);

    EXPECT_TRUE(graph.initialized());
    EXPECT_EQ(graph.state().armor_count, 3);
    EXPECT_NEAR(graph.state().center.z(), center.z(), 1e-12);
    EXPECT_NEAR(
        auto_graph::shortestAngularDistance(0.0, graph.state().yaw),
        0.0, 1e-12);
    EXPECT_NEAR(graph.state().dz, dz1, 1e-12);
    EXPECT_NEAR(graph.state().outpost_dz_2, dz2, 1e-12);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostInitTreatsFirstObservationAsSlotZero) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.outpost_radius = 0.2765;
    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);

    const Eigen::Vector3d center(1.0, 2.0, 0.0);
    constexpr double observed_dz = -0.1;
    const double center_yaw = 0.3;
    const int observed_index = 2;
    const double armor_yaw =
        center_yaw + filter_test::graph_optimizer::armorYawOffset(observed_index, 3);
    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor obs;
    obs.number = "outpost";
    obs.priority = 0;
    obs.pose.position.x = center.x() - config.outpost_radius * std::cos(armor_yaw);
    obs.pose.position.y = center.y() - config.outpost_radius * std::sin(armor_yaw);
    obs.pose.position.z = center.z() + observed_dz;
    obs.pose.orientation.w = 1.0;
    obs.yaw = armor_yaw;
    observations.armors.push_back(obs);

    graph.initialize(observations);

    EXPECT_EQ(graph.state().armor_count, 3);
    EXPECT_NEAR(graph.state().center.x(), center.x(), 1e-6);
    EXPECT_NEAR(graph.state().center.y(), center.y(), 1e-6);
    EXPECT_NEAR(graph.state().center.z(), obs.pose.position.z, 1e-12);
    EXPECT_NEAR(
        auto_graph::shortestAngularDistance(armor_yaw, graph.state().yaw),
        0.0, 1e-6);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostInitUsesConfiguredVyawPrior) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.outpost_radius = 0.2765;
    config.outpost_initial_vyaw = -2.51327412287;
    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor obs;
    obs.number = "outpost";
    obs.pose.position.x = -config.outpost_radius;
    obs.pose.position.y = 0.0;
    obs.pose.position.z = 0.0;
    obs.pose.orientation.w = 1.0;
    obs.yaw = 0.0;
    observations.armors.push_back(obs);

    graph.initialize(observations);

    EXPECT_EQ(graph.state().armor_count, 3);
    EXPECT_NEAR(graph.state().vyaw, config.outpost_initial_vyaw, 1e-12);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostSinglePlateShiftDoesNotCreateVelocity) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.outpost_radius = 0.2765;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.outpost_geo_noise.tangential = 0.001;
    config.outpost_geo_noise.radial = 0.001;
    config.outpost_geo_noise.height = 0.001;
    config.outpost_geo_noise.yaw = 1e9;

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);

    auto make_observations = [&](const Eigen::Vector3d& center) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        obs.number = "outpost";
        obs.pose.position.x = center.x() - config.outpost_radius;
        obs.pose.position.y = center.y();
        obs.pose.position.z = center.z();
        obs.pose.orientation.w = 1.0;
        obs.yaw = 0.0;
        observations.armors.push_back(obs);
        return observations;
    };

    graph.initialize(make_observations(Eigen::Vector3d(0.0, 0.0, 0.0)));

    const auto actual = graph.update(
        make_observations(Eigen::Vector3d(0.01, -0.004, 0.002)),
        0.2, Eigen::Isometry3d::Identity());

    ASSERT_FALSE(actual.solve_result.failed);
    EXPECT_NEAR(actual.state.velocity.norm(), 0.0, 1e-9);
    EXPECT_LT(std::abs(actual.state.center.x()), 0.01);
    EXPECT_LT(std::abs(actual.state.center.y()), 0.004);
    EXPECT_NEAR(actual.state.center.z(), 0.002, 5e-3);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostMotionNoiseDoesNotMoveStaticBaseXY) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.s2qxy = 1e-6;
    config.s2qz = 1e-6;
    config.vel_sigma = 1e-6;
    config.outpost_s2qxy = 0.5;
    config.outpost_s2qz = 0.5;
    config.outpost_vel_sigma = 0.5;
    config.outpost_radius = 0.2765;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.outpost_geo_noise.tangential = 0.001;
    config.outpost_geo_noise.radial = 0.001;
    config.outpost_geo_noise.height = 0.001;
    config.outpost_geo_noise.yaw = 1e9;

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);

    auto make_observations = [&](const Eigen::Vector3d& center) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        obs.number = "outpost";
        obs.pose.position.x = center.x() - config.outpost_radius;
        obs.pose.position.y = center.y();
        obs.pose.position.z = center.z();
        obs.pose.orientation.w = 1.0;
        obs.yaw = 0.0;
        observations.armors.push_back(obs);
        return observations;
    };

    graph.initialize(make_observations(Eigen::Vector3d(0.0, 0.0, 0.0)));

    const auto actual = graph.update(
        make_observations(Eigen::Vector3d(0.03, 0.0, 0.0)),
        0.2, Eigen::Isometry3d::Identity());

    ASSERT_FALSE(actual.solve_result.failed);
    EXPECT_NEAR(actual.state.velocity.norm(), 0.0, 1e-9);
    EXPECT_GT(actual.state.center.x(), 0.0);
    EXPECT_LT(actual.state.center.x(), 0.02);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostBaseXYResistsSinglePlateSwitchBias) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.outpost_radius = 0.2765;
    config.outpost_s2qxy = 0.5;
    config.outpost_vel_sigma = 0.5;
    config.outpost_prior_noise.radius = 0.001;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;
    config.outpost_geo_noise.tangential = 0.01;
    config.outpost_geo_noise.radial = 0.01;
    config.outpost_geo_noise.height = 1e9;
    config.outpost_geo_noise.yaw = 0.01;

    const Eigen::Vector3d center(1.0, 2.0, 1.5);
    const double center_yaw = 0.2;
    const Eigen::Vector2d biased_center_offset(0.30, -0.18);

    auto make_observations = [&](int slot, const Eigen::Vector2d& center_bias) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        const double armor_yaw =
            center_yaw + filter_test::graph_optimizer::armorYawOffset(slot, 3);
        obs.number = "outpost";
        obs.pose.position.x =
            center.x() + center_bias.x() -
            config.outpost_radius * std::cos(armor_yaw);
        obs.pose.position.y =
            center.y() + center_bias.y() -
            config.outpost_radius * std::sin(armor_yaw);
        obs.pose.position.z = center.z();
        obs.pose.orientation.z = std::sin(armor_yaw / 2.0);
        obs.pose.orientation.w = std::cos(armor_yaw / 2.0);
        obs.yaw = armor_yaw;
        observations.armors.push_back(obs);
        return observations;
    };

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(make_observations(0, Eigen::Vector2d::Zero()));

    filter_test::graph_optimizer::ArmorCvPixelOutput actual;
    for (int i = 0; i < 10; ++i) {
        actual = graph.update(
            make_observations(0, Eigen::Vector2d::Zero()),
            0.02, Eigen::Isometry3d::Identity());
        ASSERT_FALSE(actual.solve_result.failed);
    }

    actual = graph.update(
        make_observations(1, biased_center_offset),
        0.02, Eigen::Isometry3d::Identity());

    ASSERT_FALSE(actual.solve_result.failed);
    ASSERT_EQ(actual.matched_indices.size(), 1u);
    EXPECT_EQ(actual.matched_indices[0], 1);
    EXPECT_NEAR(actual.state.center.x(), center.x(), 0.05);
    EXPECT_NEAR(actual.state.center.y(), center.y(), 0.05);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostEdgeReprojDoesNotMoveBaseXY) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.outpost_radius = 0.2765;
    config.outpost_s2qxy = 0.5;
    config.outpost_vel_sigma = 0.5;
    config.outpost_prior_noise.radius = 0.001;
    config.use_edge_reproj_factor = true;
    config.edge_reproj_sigma = 0.001;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;
    config.outpost_geo_noise.tangential = 1e9;
    config.outpost_geo_noise.radial = 1e9;
    config.outpost_geo_noise.height = 1e9;
    config.outpost_geo_noise.yaw = 1e9;

    const Eigen::Vector3d center(1.0, 2.0, 1.5);
    const double center_yaw = 0.2;
    const Eigen::Vector2d pixel_shift(8.0, -5.0);

    auto make_observations = [&](int slot, bool shift_pixels) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        const double armor_yaw =
            center_yaw + filter_test::graph_optimizer::armorYawOffset(slot, 3);
        obs.number = "outpost";
        obs.pose.position.x =
            center.x() - config.outpost_radius * std::cos(armor_yaw);
        obs.pose.position.y =
            center.y() - config.outpost_radius * std::sin(armor_yaw);
        obs.pose.position.z = center.z();
        obs.pose.orientation.z = std::sin(armor_yaw / 2.0);
        obs.pose.orientation.w = std::cos(armor_yaw / 2.0);
        obs.yaw = armor_yaw;

        const auto pixels = filter_test::graph_optimizer::projectArmorPixels(
            center, center_yaw, config.outpost_radius, 0.0, slot,
            config.outpost_armor_pitch, Eigen::Isometry3d::Identity(),
            config.camera_matrix, config.distortion, 3);
        for (std::size_t i = 0; i < pixels.size(); ++i) {
            const Eigen::Vector2d p =
                shift_pixels ? pixels[i] + pixel_shift : pixels[i];
            obs.detected_points[i].x = static_cast<float>(p.x());
            obs.detected_points[i].y = static_cast<float>(p.y());
        }
        observations.armors.push_back(obs);
        return observations;
    };

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(make_observations(0, false));

    const int slot = 1;
    const auto actual = graph.update(
        make_observations(slot, true), 0.02, Eigen::Isometry3d::Identity());

    ASSERT_FALSE(actual.solve_result.failed);
    ASSERT_EQ(actual.matched_indices.size(), 1u);
    EXPECT_EQ(actual.matched_indices[0], slot);
    EXPECT_NEAR(actual.state.center.x(), center.x(), 0.01);
    EXPECT_NEAR(actual.state.center.y(), center.y(), 0.01);
    EXPECT_NEAR(actual.state.radius_1, config.outpost_radius, 0.005);
    EXPECT_NEAR(actual.state.dz, 0.0, 0.01);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphTracksUnnumberedOutpostSinglePlate) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 5;
    config.s2qyaw = 0.02;
    config.vyaw_sigma = 0.05;
    config.outpost_radius = 0.2765;
    config.outpost_initial_vyaw = -2.51327412287;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;
    config.outpost_geo_noise.height = 0.05;

    const Eigen::Vector3d center(1.0, 2.0, 0.0);
    const double initial_yaw = 0.3;
    const double angular_velocity = config.outpost_initial_vyaw;
    const int physical_index = 2;
    constexpr double observed_dz = -0.1;

    auto make_observations = [&](double center_yaw) {
        auto_aim_interfaces::msg::Armors observations;
        const double armor_yaw = center_yaw +
            filter_test::graph_optimizer::armorYawOffset(physical_index, 3);
        auto_aim_interfaces::msg::Armor obs;
        obs.number = "outpost";
        obs.priority = -1;
        obs.pose.position.x =
            center.x() - config.outpost_radius * std::cos(armor_yaw);
        obs.pose.position.y =
            center.y() - config.outpost_radius * std::sin(armor_yaw);
        obs.pose.position.z = center.z() + observed_dz;
        obs.pose.orientation.z = std::sin(armor_yaw / 2.0);
        obs.pose.orientation.w = std::cos(armor_yaw / 2.0);
        obs.yaw = armor_yaw;
        observations.armors.push_back(obs);
        return observations;
    };

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(make_observations(initial_yaw));

    filter_test::graph_optimizer::ArmorCvPixelOutput actual;
    constexpr double dt = 0.02;
    for (int frame = 1; frame <= 8; ++frame) {
        const double center_yaw = initial_yaw + angular_velocity * dt * frame;
        actual = graph.update(
            make_observations(center_yaw), dt, Eigen::Isometry3d::Identity());
        ASSERT_FALSE(actual.solve_result.failed);
        ASSERT_EQ(actual.matched_indices.size(), 1u);
        EXPECT_EQ(actual.matched_indices[0], 0);
    }

    const double expected_yaw = auto_graph::normalizeAngle(
        initial_yaw + filter_test::graph_optimizer::armorYawOffset(physical_index, 3) +
        angular_velocity * dt * 8.0);
    EXPECT_NEAR(actual.state.center.x(), center.x(), 0.02);
    EXPECT_NEAR(actual.state.center.y(), center.y(), 0.02);
    EXPECT_NEAR(actual.state.center.z(), center.z() + observed_dz, 0.02);
    EXPECT_NEAR(
        auto_graph::shortestAngularDistance(expected_yaw, actual.state.yaw),
        0.0, 0.08);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphUpdateReportsColdStartState) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 1;

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor obs;
    obs.pose.position.x = 1.0;
    obs.pose.position.y = 0.0;
    obs.pose.position.z = 0.5;
    obs.pose.orientation.w = 1.0;
    obs.yaw = 0.0;
    observations.armors.push_back(obs);

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(observations);
    const auto actual =
        graph.update(observations, 0.25, Eigen::Isometry3d::Identity());

    ASSERT_EQ(actual.matched_indices.size(), 1u);
    EXPECT_EQ(actual.matched_indices[0], 0);
    EXPECT_EQ(actual.solve_result.frame_id, 1u);
    EXPECT_TRUE(actual.solve_result.cold_start);
    EXPECT_FALSE(actual.solve_result.failed);
    EXPECT_FALSE(actual.solve_result.optimized);
    EXPECT_EQ(graph.frameId(), 1u);
    EXPECT_NEAR(actual.state.center.x(), 1.25, 1e-12);
    EXPECT_NEAR(actual.state.center.y(), 0.0, 1e-12);
    EXPECT_NEAR(actual.state.center.z(), 0.5, 1e-12);
    EXPECT_NEAR(actual.state.yaw, 0.0, 1e-12);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphUpdateOptimizesWithoutColdStart) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor obs;
    obs.pose.position.x = 1.0;
    obs.pose.position.y = 0.0;
    obs.pose.position.z = 0.5;
    obs.pose.orientation.w = 1.0;
    obs.yaw = 0.0;
    observations.armors.push_back(obs);

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(observations);
    const auto actual =
        graph.update(observations, 0.01, Eigen::Isometry3d::Identity());

    EXPECT_EQ(actual.solve_result.frame_id, 1u);
    EXPECT_FALSE(actual.solve_result.failed);
    EXPECT_TRUE(actual.solve_result.optimized);
    ASSERT_EQ(actual.matched_indices.size(), 1u);
    EXPECT_EQ(actual.matched_indices[0], 0);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphUpdatesOutpostWithThreePredictions) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.outpost_radius = 0.2765;
    config.use_edge_reproj_factor = false;
    config.pixel_sigma = 100.0;
    constexpr double dz1 = 0.2;
    constexpr double dz2 = -0.1;

    filter_test::graph_optimizer::TrackerState state;
    state.armor_count = 3;
    state.center << 1.0, 2.0, 0.5;
    state.yaw = 0.1;
    state.radius_1 = config.outpost_radius;
    state.radius_2 = config.outpost_radius;
    state.dz = dz1;
    state.outpost_dz_2 = dz2;
    const auto predicted =
        filter_test::graph_optimizer::predictedArmorsFromState(state);

    auto_aim_interfaces::msg::Armors observations;
    for (const auto& pred : predicted) {
        auto_aim_interfaces::msg::Armor obs;
        obs.number = "outpost";
        obs.pose.position.x = pred.position.x();
        obs.pose.position.y = pred.position.y();
        obs.pose.position.z = pred.position.z();
        obs.pose.orientation.w = 1.0;
        obs.yaw = pred.yaw;
        observations.armors.push_back(obs);
    }

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(observations);
    const auto actual =
        graph.update(observations, 0.01, Eigen::Isometry3d::Identity());

    EXPECT_FALSE(actual.solve_result.failed);
    ASSERT_EQ(actual.matched_indices.size(), 3u);
    EXPECT_EQ(std::set<int>(actual.matched_indices.begin(),
                            actual.matched_indices.end()).size(), 3u);
    ASSERT_EQ(actual.predicted_armors.size(), 3u);
    EXPECT_EQ(actual.state.armor_count, 3);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostUpdateIgnoresPriorityIndex) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 1;
    config.outpost_radius = 0.2765;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 100.0;

    auto_aim_interfaces::msg::Armors init_observations;
    auto_aim_interfaces::msg::Armor init_obs;
    init_obs.number = "outpost";
    init_obs.priority = 0;
    init_obs.pose.position.x = -config.outpost_radius;
    init_obs.pose.position.y = 0.0;
    init_obs.pose.position.z = 0.5;
    init_obs.pose.orientation.w = 1.0;
    init_obs.yaw = 0.0;
    init_observations.armors.push_back(init_obs);

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(init_observations);

    const auto predicted =
        filter_test::graph_optimizer::predictedArmorsFromState(graph.state());
    auto_aim_interfaces::msg::Armors update_observations;
    auto_aim_interfaces::msg::Armor update_obs;
    update_obs.number = "outpost";
    update_obs.priority = 2;
    update_obs.pose.position.x = predicted[0].position.x();
    update_obs.pose.position.y = predicted[0].position.y();
    update_obs.pose.position.z = predicted[0].position.z();
    update_obs.pose.orientation.w = 1.0;
    update_obs.yaw = predicted[0].yaw;
    update_observations.armors.push_back(update_obs);

    const auto actual =
        graph.update(update_observations, 0.01, Eigen::Isometry3d::Identity());

    ASSERT_EQ(actual.matched_indices.size(), 1u);
    EXPECT_EQ(actual.matched_indices[0], 0);
    EXPECT_FALSE(actual.solve_result.failed);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostRadiusPriorResistsSinglePlateBias) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.s2qxy = 1e-6;
    config.s2qz = 1e-6;
    config.vel_sigma = 1e-6;
    config.outpost_radius = 0.2765;
    config.outpost_prior_noise.radius = 0.005;
    config.outpost_prior_noise.dz = 0.005;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;

    auto_aim_interfaces::msg::Armors init_observations;
    auto_aim_interfaces::msg::Armor init_obs;
    init_obs.number = "outpost";
    init_obs.priority = 0;
    init_obs.pose.position.x = -config.outpost_radius;
    init_obs.pose.position.y = 0.0;
    init_obs.pose.position.z = 0.5;
    init_obs.pose.orientation.w = 1.0;
    init_obs.yaw = 0.0;
    init_observations.armors.push_back(init_obs);

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(init_observations);

    auto_aim_interfaces::msg::Armors biased_observations;
    auto_aim_interfaces::msg::Armor biased_obs = init_obs;
    biased_obs.pose.position.x = -0.13;
    biased_observations.armors.push_back(biased_obs);

    const auto actual =
        graph.update(biased_observations, 0.01, Eigen::Isometry3d::Identity());

    ASSERT_FALSE(actual.solve_result.failed);
    EXPECT_NEAR(actual.state.radius_1, config.outpost_radius, 0.03);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostSinglePlateSlotUpdatesDz) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.outpost_radius = 0.2765;
    config.outpost_prior_noise.radius = 0.005;
    config.outpost_prior_noise.dz = 0.25;
    config.outpost_s2qz = 1e-6;
    config.outpost_vel_sigma = 1e-6;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;
    config.outpost_geo_noise.tangential = 0.001;
    config.outpost_geo_noise.radial = 0.001;
    config.outpost_geo_noise.height = 0.001;
    config.outpost_geo_noise.yaw = 0.001;

    const Eigen::Vector3d center(1.0, 2.0, 1.5);
    const double yaw = 0.2;
    const double dz1 = 0.2;

    auto make_obs = [&](int slot, double z) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        const double armor_yaw =
            yaw + filter_test::graph_optimizer::armorYawOffset(slot, 3);
        obs.number = "outpost";
        obs.pose.position.x =
            center.x() - config.outpost_radius * std::cos(armor_yaw);
        obs.pose.position.y =
            center.y() - config.outpost_radius * std::sin(armor_yaw);
        obs.pose.position.z = z;
        obs.pose.orientation.z = std::sin(armor_yaw / 2.0);
        obs.pose.orientation.w = std::cos(armor_yaw / 2.0);
        obs.yaw = armor_yaw;
        observations.armors.push_back(obs);
        return observations;
    };

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(make_obs(0, center.z()));

    const auto actual =
        graph.update(make_obs(1, center.z() + dz1),
                     0.01, Eigen::Isometry3d::Identity());

    ASSERT_FALSE(actual.solve_result.failed);
    ASSERT_EQ(actual.matched_indices.size(), 1u);
    EXPECT_EQ(actual.matched_indices[0], 1);
    EXPECT_NEAR(actual.state.center.z(), center.z(), 0.03);
    EXPECT_NEAR(actual.state.dz, dz1, 0.03);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostDefaultWeightsKeepCenterZAndUpdateDz) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.outpost_radius = 0.2765;
    config.outpost_prior_noise.radius = 0.005;
    config.outpost_prior_noise.dz = 0.25;
    config.outpost_vel_sigma = 0.005;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;
    config.outpost_geo_noise.tangential = 0.001;
    config.outpost_geo_noise.radial = 0.001;
    config.outpost_geo_noise.height = 0.08;
    config.outpost_geo_noise.yaw = 0.001;

    const Eigen::Vector3d center(1.0, 2.0, 1.5);
    const double yaw = 0.2;
    const double dz1 = 0.20;
    const double dz2 = -0.16;

    auto make_obs = [&](int slot, double z) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        const double armor_yaw =
            yaw + filter_test::graph_optimizer::armorYawOffset(slot, 3);
        obs.number = "outpost";
        obs.pose.position.x =
            center.x() - config.outpost_radius * std::cos(armor_yaw);
        obs.pose.position.y =
            center.y() - config.outpost_radius * std::sin(armor_yaw);
        obs.pose.position.z = z;
        obs.pose.orientation.z = std::sin(armor_yaw / 2.0);
        obs.pose.orientation.w = std::cos(armor_yaw / 2.0);
        obs.yaw = armor_yaw;
        observations.armors.push_back(obs);
        return observations;
    };

    {
        filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
        graph.initialize(make_obs(0, center.z()));

        filter_test::graph_optimizer::ArmorCvPixelOutput actual;
        for (int i = 0; i < 8; ++i) {
            actual = graph.update(
                make_obs(1, center.z() + dz1),
                0.01, Eigen::Isometry3d::Identity());
        }

        ASSERT_FALSE(actual.solve_result.failed);
        ASSERT_EQ(actual.matched_indices.size(), 1u);
        EXPECT_EQ(actual.matched_indices[0], 1);
        EXPECT_NEAR(actual.state.center.z(), center.z(), 0.05);
        EXPECT_NEAR(actual.state.dz, dz1, 0.08);
        EXPECT_NEAR(actual.state.outpost_dz_2, 0.0, 0.08);
    }

    {
        filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
        graph.initialize(make_obs(0, center.z()));

        filter_test::graph_optimizer::ArmorCvPixelOutput actual;
        for (int i = 0; i < 8; ++i) {
            actual = graph.update(
                make_obs(2, center.z() + dz2),
                0.01, Eigen::Isometry3d::Identity());
        }

        ASSERT_FALSE(actual.solve_result.failed);
        ASSERT_EQ(actual.matched_indices.size(), 1u);
        EXPECT_EQ(actual.matched_indices[0], 2);
        EXPECT_NEAR(actual.state.center.z(), center.z(), 0.05);
        EXPECT_NEAR(actual.state.dz, 0.0, 0.08);
        EXPECT_NEAR(actual.state.outpost_dz_2, dz2, 0.08);
    }
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostLongSlot1ObservationKeepsBaseZ) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.outpost_radius = 0.2765;
    config.outpost_prior_noise.radius = 0.005;
    config.outpost_prior_noise.dz = 0.25;
    config.outpost_s2qz = 0.0004;
    config.outpost_vel_sigma = 0.005;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;
    config.outpost_geo_noise.tangential = 0.001;
    config.outpost_geo_noise.radial = 0.001;
    config.outpost_geo_noise.height = 0.08;
    config.outpost_geo_noise.yaw = 0.001;

    const Eigen::Vector3d center(1.0, 2.0, 1.5);
    const double yaw = 0.2;
    const double dz1 = 0.20;

    auto make_obs = [&](int slot, double z) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        const double armor_yaw =
            yaw + filter_test::graph_optimizer::armorYawOffset(slot, 3);
        obs.number = "outpost";
        obs.pose.position.x =
            center.x() - config.outpost_radius * std::cos(armor_yaw);
        obs.pose.position.y =
            center.y() - config.outpost_radius * std::sin(armor_yaw);
        obs.pose.position.z = z;
        obs.pose.orientation.z = std::sin(armor_yaw / 2.0);
        obs.pose.orientation.w = std::cos(armor_yaw / 2.0);
        obs.yaw = armor_yaw;
        observations.armors.push_back(obs);
        return observations;
    };

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(make_obs(0, center.z()));

    filter_test::graph_optimizer::ArmorCvPixelOutput actual;
    for (int i = 0; i < 200; ++i) {
        actual = graph.update(
            make_obs(1, center.z() + dz1),
            0.01, Eigen::Isometry3d::Identity());
    }

    ASSERT_FALSE(actual.solve_result.failed);
    ASSERT_EQ(actual.matched_indices.size(), 1u);
    EXPECT_EQ(actual.matched_indices[0], 1);
    EXPECT_NEAR(actual.state.center.z(), center.z(), 0.05);
    EXPECT_NEAR(actual.state.dz, dz1, 0.05);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostSinglePlateSlot2UpdatesDz2) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.outpost_radius = 0.2765;
    config.outpost_prior_noise.radius = 0.005;
    config.outpost_prior_noise.dz = 0.25;
    config.outpost_s2qz = 1e-6;
    config.outpost_vel_sigma = 1e-6;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;
    config.outpost_geo_noise.tangential = 0.001;
    config.outpost_geo_noise.radial = 0.001;
    config.outpost_geo_noise.height = 0.001;
    config.outpost_geo_noise.yaw = 0.001;

    const Eigen::Vector3d center(1.0, 2.0, 1.5);
    const double yaw = 0.2;
    const double dz2 = -0.18;

    auto make_obs = [&](int slot, double z) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        const double armor_yaw =
            yaw + filter_test::graph_optimizer::armorYawOffset(slot, 3);
        obs.number = "outpost";
        obs.pose.position.x =
            center.x() - config.outpost_radius * std::cos(armor_yaw);
        obs.pose.position.y =
            center.y() - config.outpost_radius * std::sin(armor_yaw);
        obs.pose.position.z = z;
        obs.pose.orientation.z = std::sin(armor_yaw / 2.0);
        obs.pose.orientation.w = std::cos(armor_yaw / 2.0);
        obs.yaw = armor_yaw;
        observations.armors.push_back(obs);
        return observations;
    };

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(make_obs(0, center.z()));

    const auto actual =
        graph.update(make_obs(2, center.z() + dz2),
                     0.01, Eigen::Isometry3d::Identity());

    ASSERT_FALSE(actual.solve_result.failed);
    ASSERT_EQ(actual.matched_indices.size(), 1u);
    EXPECT_EQ(actual.matched_indices[0], 2);
    EXPECT_NEAR(actual.state.center.z(), center.z(), 0.03);
    EXPECT_NEAR(actual.state.dz, 0.0, 0.03);
    EXPECT_NEAR(actual.state.outpost_dz_2, dz2, 0.03);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostSinglePlateSlot0UpdatesCenterZOnly) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.outpost_radius = 0.2765;
    config.outpost_prior_noise.radius = 0.005;
    config.outpost_prior_noise.dz = 0.25;
    config.outpost_s2qz = 0.2;
    config.outpost_vel_sigma = 0.5;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;
    config.outpost_geo_noise.tangential = 0.001;
    config.outpost_geo_noise.radial = 0.001;
    config.outpost_geo_noise.height = 0.001;
    config.outpost_geo_noise.yaw = 0.001;

    const Eigen::Vector3d center(1.0, 2.0, 1.5);
    const double yaw = 0.2;
    const double observed_z = center.z() + 0.25;

    auto make_obs = [&](double z) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        obs.number = "outpost";
        obs.pose.position.x =
            center.x() - config.outpost_radius * std::cos(yaw);
        obs.pose.position.y =
            center.y() - config.outpost_radius * std::sin(yaw);
        obs.pose.position.z = z;
        obs.pose.orientation.z = std::sin(yaw / 2.0);
        obs.pose.orientation.w = std::cos(yaw / 2.0);
        obs.yaw = yaw;
        observations.armors.push_back(obs);
        return observations;
    };

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(make_obs(center.z()));

    const auto actual =
        graph.update(make_obs(observed_z), 0.01, Eigen::Isometry3d::Identity());

    ASSERT_FALSE(actual.solve_result.failed);
    ASSERT_EQ(actual.matched_indices.size(), 1u);
    EXPECT_EQ(actual.matched_indices[0], 0);
    EXPECT_NEAR(actual.state.center.z(), observed_z, 0.03);
    EXPECT_NEAR(actual.state.dz, 0.0, 0.03);
    EXPECT_NEAR(actual.state.outpost_dz_2, 0.0, 0.03);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphOutpostSinglePlateKeepsYawSlotWhenZJumps) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.outpost_radius = 0.2765;
    config.outpost_prior_noise.radius = 0.005;
    config.outpost_prior_noise.dz = 0.25;
    config.outpost_s2qz = 1e-6;
    config.outpost_vel_sigma = 1e-6;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;
    config.outpost_geo_noise.tangential = 0.001;
    config.outpost_geo_noise.radial = 0.001;
    config.outpost_geo_noise.height = 0.001;
    config.outpost_geo_noise.yaw = 1.0;

    const Eigen::Vector3d center_xy(1.0, 2.0, 0.0);
    const double repeated_yaw = -1.2;
    const double base_z = 1.7;
    const double low_z = 1.4;

    auto make_obs = [&](double z) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        obs.number = "outpost";
        obs.pose.position.x =
            center_xy.x() - config.outpost_radius * std::cos(repeated_yaw);
        obs.pose.position.y =
            center_xy.y() - config.outpost_radius * std::sin(repeated_yaw);
        obs.pose.position.z = z;
        obs.pose.orientation.z = std::sin(repeated_yaw / 2.0);
        obs.pose.orientation.w = std::cos(repeated_yaw / 2.0);
        obs.yaw = repeated_yaw;
        observations.armors.push_back(obs);
        return observations;
    };

    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);
    graph.initialize(make_obs(base_z));

    const auto actual =
        graph.update(make_obs(low_z), 0.01, Eigen::Isometry3d::Identity());

    ASSERT_FALSE(actual.solve_result.failed);
    ASSERT_EQ(actual.matched_indices.size(), 1u);
    EXPECT_EQ(actual.matched_indices[0], 0);
    EXPECT_NEAR(actual.state.dz, 0.0, 0.03);
}

TEST(GraphOptimizerFacadeTest, ArmorCvPixelGraphResetClearsInitializationState) {
    filter_test::graph_optimizer::TrackerConfig config;
    filter_test::graph_optimizer::ArmorCvPixelGraph graph(config);

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor obs;
    obs.pose.position.x = 1.0;
    obs.pose.position.y = 0.0;
    obs.pose.position.z = 0.5;
    obs.pose.orientation.w = 1.0;
    obs.yaw = 0.0;
    observations.armors.push_back(obs);

    graph.initialize(observations);
    ASSERT_TRUE(graph.initialized());

    graph.reset();

    EXPECT_FALSE(graph.initialized());
    EXPECT_EQ(graph.frameId(), 0u);
    EXPECT_NEAR(graph.state().center.norm(), 0.0, 1e-12);
}

TEST(GraphOptimizerTrackerCoreTest, ArmorGraphTrackerDrivesUpdatePipeline) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 1;
    filter_test::graph_optimizer::ArmorGraphTracker tracker(config);

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor obs;
    obs.pose.position.x = 1.0;
    obs.pose.position.y = 0.0;
    obs.pose.position.z = 0.5;
    obs.pose.orientation.w = 1.0;
    obs.yaw = 0.0;
    observations.armors.push_back(obs);

    (void)tracker.update({observations, 0.01, Eigen::Isometry3d::Identity()});
    auto result = tracker.update({observations, 0.25, Eigen::Isometry3d::Identity()});

    EXPECT_TRUE(result.accepted_frame);
    EXPECT_TRUE(result.initialized);
    EXPECT_EQ(result.matched_indices.size(), observations.armors.size());
    EXPECT_EQ(tracker.frameId(), 1u);
}

TEST(GraphOptimizerTrackerCoreTest, InitializationFrameIsAcceptedButNotSolved) {
    filter_test::graph_optimizer::TrackerConfig config;
    filter_test::graph_optimizer::ArmorGraphTracker tracker(config);

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor obs;
    obs.pose.position.x = 1.0;
    obs.pose.position.y = 0.0;
    obs.pose.position.z = 0.5;
    obs.yaw = 0.0;
    observations.armors.push_back(obs);

    auto result = tracker.update({observations, 0.01, Eigen::Isometry3d::Identity()});

    EXPECT_TRUE(result.accepted_frame);
    EXPECT_TRUE(result.initialized);
    EXPECT_FALSE(result.solved);
    EXPECT_FALSE(result.cold_start);
    EXPECT_FALSE(result.solve_failed);
    EXPECT_TRUE(result.solve_error.empty());
}

TEST(GraphOptimizerTrackerCoreTest, ColdStartFrameIsSolvedForPublishingCompatibility) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 1;
    filter_test::graph_optimizer::ArmorGraphTracker tracker(config);

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor obs;
    obs.pose.position.x = 1.0;
    obs.pose.position.y = 0.0;
    obs.pose.position.z = 0.5;
    obs.yaw = 0.0;
    observations.armors.push_back(obs);

    (void)tracker.update({observations, 0.01, Eigen::Isometry3d::Identity()});
    auto result = tracker.update({observations, 0.01, Eigen::Isometry3d::Identity()});

    EXPECT_TRUE(result.accepted_frame);
    EXPECT_TRUE(result.solved);
    EXPECT_TRUE(result.cold_start);
    EXPECT_FALSE(result.solve_failed);
    EXPECT_TRUE(result.solve_error.empty());
}

TEST(GraphOptimizerTrackerCoreTest, OutlierObservationDoesNotBecomeMatchedFactor) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    filter_test::graph_optimizer::ArmorGraphTracker tracker(config);

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor init_obs;
    init_obs.pose.position.x = 1.0;
    init_obs.pose.position.y = 0.0;
    init_obs.pose.position.z = 0.5;
    init_obs.pose.orientation.w = 1.0;
    init_obs.yaw = 0.0;
    observations.armors.push_back(init_obs);

    (void)tracker.update({observations, 0.01, Eigen::Isometry3d::Identity()});

    auto_aim_interfaces::msg::Armors outlier_observations;
    auto_aim_interfaces::msg::Armor outlier;
    outlier.pose.position.x = 10.0;
    outlier.pose.position.y = -10.0;
    outlier.pose.position.z = 3.0;
    outlier.pose.orientation.w = 1.0;
    outlier.yaw = M_PI;
    outlier_observations.armors.push_back(outlier);

    auto result =
        tracker.update({outlier_observations, 0.01, Eigen::Isometry3d::Identity()});

    ASSERT_EQ(result.matched_indices.size(), 1u);
    EXPECT_EQ(result.matched_indices[0], -1);
    EXPECT_FALSE(result.solve_failed);
}

TEST(GraphOptimizerTrackerCoreTest, UnmatchedFrameResetsTrackerForReinitialization) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    filter_test::graph_optimizer::ArmorGraphTracker tracker(config);

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor init_obs;
    init_obs.pose.position.x = 1.0;
    init_obs.pose.position.y = 0.0;
    init_obs.pose.position.z = 0.5;
    init_obs.pose.orientation.w = 1.0;
    init_obs.yaw = 0.0;
    observations.armors.push_back(init_obs);

    auto init_result =
        tracker.update({observations, 0.01, Eigen::Isometry3d::Identity()});
    ASSERT_TRUE(init_result.initialized);

    auto_aim_interfaces::msg::Armors outlier_observations;
    auto_aim_interfaces::msg::Armor outlier;
    outlier.pose.position.x = 10.0;
    outlier.pose.position.y = -10.0;
    outlier.pose.position.z = 3.0;
    outlier.pose.orientation.w = 1.0;
    outlier.yaw = M_PI;
    outlier_observations.armors.push_back(outlier);

    auto lost_result =
        tracker.update({outlier_observations, 0.01, Eigen::Isometry3d::Identity()});

    EXPECT_FALSE(lost_result.accepted_frame);
    EXPECT_FALSE(lost_result.initialized);
    EXPECT_FALSE(tracker.initialized());
    ASSERT_EQ(lost_result.matched_indices.size(), 1u);
    EXPECT_EQ(lost_result.matched_indices[0], -1);

    auto reinit_result =
        tracker.update({observations, 0.01, Eigen::Isometry3d::Identity()});
    EXPECT_TRUE(reinit_result.accepted_frame);
    EXPECT_TRUE(reinit_result.initialized);
    EXPECT_FALSE(reinit_result.solved);
    EXPECT_EQ(tracker.frameId(), 0u);
}

TEST(GraphOptimizerTrackerCoreTest, OutpostAmbiguousMarginDoesNotRejectYawNearestMatch) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.match_max_cost = 100.0;
    config.outpost_ambiguous_match_margin = 0.25;
    filter_test::graph_optimizer::ArmorGraphTracker tracker(config);

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor init_obs;
    init_obs.number = "outpost";
    init_obs.pose.position.x = 1.0;
    init_obs.pose.position.y = 0.0;
    init_obs.pose.position.z = 0.5;
    init_obs.pose.orientation.w = 1.0;
    init_obs.yaw = 0.0;
    observations.armors.push_back(init_obs);

    auto init_result =
        tracker.update({observations, 0.01, Eigen::Isometry3d::Identity()});
    ASSERT_TRUE(init_result.initialized);
    const auto predicted =
        filter_test::graph_optimizer::predictedArmorsFromState(init_result.state);
    ASSERT_GE(predicted.size(), 2u);

    auto_aim_interfaces::msg::Armors ambiguous_observations;
    auto_aim_interfaces::msg::Armor ambiguous_obs;
    ambiguous_obs.number = "outpost";
    const auto midpoint = 0.5 * (predicted[0].position + predicted[1].position);
    ambiguous_obs.pose.position.x = midpoint.x();
    ambiguous_obs.pose.position.y = midpoint.y();
    ambiguous_obs.pose.position.z = midpoint.z();
    ambiguous_obs.pose.orientation.w = 1.0;
    ambiguous_obs.yaw = filter_test::graph_optimizer::armorYawOffset(1, 3) * 0.5 + 0.02;
    ambiguous_observations.armors.push_back(ambiguous_obs);

    auto matched_result =
        tracker.update({ambiguous_observations, 0.01, Eigen::Isometry3d::Identity()});

    EXPECT_TRUE(matched_result.accepted_frame);
    EXPECT_TRUE(matched_result.initialized);
    EXPECT_TRUE(matched_result.solved);
    EXPECT_TRUE(tracker.initialized());
    EXPECT_TRUE(matched_result.reset_reason.empty());
    ASSERT_EQ(matched_result.matched_indices.size(), 1u);
    EXPECT_EQ(matched_result.matched_indices[0], 1);
    EXPECT_GT(tracker.frameId(), 0u);
}

TEST(GraphOptimizerTrackerCoreTest, OutpostPoorMatchQualityDoesNotResetTracker) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.optimizer.update_iterations = 8;
    config.match_quality_window_size = 1;
    config.match_quality_failure_threshold = -1.0;
    config.match_quality_failure_ratio = 0.0;
    config.outpost_radius = 0.2765;
    config.outpost_prior_noise.radius = 0.005;
    config.outpost_prior_noise.dz = 0.25;
    config.outpost_s2qz = 1e-6;
    config.outpost_vel_sigma = 1e-6;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;
    config.outpost_geo_noise.tangential = 0.001;
    config.outpost_geo_noise.radial = 0.001;
    config.outpost_geo_noise.height = 0.001;
    config.outpost_geo_noise.yaw = 0.001;
    filter_test::graph_optimizer::ArmorGraphTracker tracker(config);

    const Eigen::Vector3d center(1.0, 2.0, 1.5);
    const double yaw = 0.2;
    const double dz1 = 0.20;

    auto make_obs = [&](int slot, double z) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        const double armor_yaw =
            yaw + filter_test::graph_optimizer::armorYawOffset(slot, 3);
        obs.number = "outpost";
        obs.pose.position.x =
            center.x() - config.outpost_radius * std::cos(armor_yaw);
        obs.pose.position.y =
            center.y() - config.outpost_radius * std::sin(armor_yaw);
        obs.pose.position.z = z;
        obs.pose.orientation.z = std::sin(armor_yaw / 2.0);
        obs.pose.orientation.w = std::cos(armor_yaw / 2.0);
        obs.yaw = armor_yaw;
        observations.armors.push_back(obs);
        return observations;
    };

    auto init_result =
        tracker.update({make_obs(0, center.z()), 0.01, Eigen::Isometry3d::Identity()});
    ASSERT_TRUE(init_result.initialized);

    const auto held_result = tracker.update({
        make_obs(1, center.z() + dz1), 0.01, Eigen::Isometry3d::Identity()});
    EXPECT_TRUE(held_result.accepted_frame);
    EXPECT_TRUE(held_result.initialized);
    EXPECT_TRUE(held_result.reset_reason.empty());
    EXPECT_TRUE(tracker.initialized());
}

TEST(GraphOptimizerTrackerCoreTest, OutpostColdStartMatchQualityWaitsForOptimizedState) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 3;
    config.match_quality_window_size = 1;
    config.match_quality_failure_threshold = -1.0;
    config.match_quality_failure_ratio = 0.0;
    config.outpost_radius = 0.2765;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;
    filter_test::graph_optimizer::ArmorGraphTracker tracker(config);

    const Eigen::Vector3d center(1.0, 2.0, 1.5);
    const double yaw = 0.2;
    const double dz1 = 0.20;

    auto make_obs = [&](int slot, double z) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        const double armor_yaw =
            yaw + filter_test::graph_optimizer::armorYawOffset(slot, 3);
        obs.number = "outpost";
        obs.pose.position.x =
            center.x() - config.outpost_radius * std::cos(armor_yaw);
        obs.pose.position.y =
            center.y() - config.outpost_radius * std::sin(armor_yaw);
        obs.pose.position.z = z;
        obs.pose.orientation.z = std::sin(armor_yaw / 2.0);
        obs.pose.orientation.w = std::cos(armor_yaw / 2.0);
        obs.yaw = armor_yaw;
        observations.armors.push_back(obs);
        return observations;
    };

    auto init_result =
        tracker.update({make_obs(0, center.z()), 0.01, Eigen::Isometry3d::Identity()});
    ASSERT_TRUE(init_result.initialized);
    ASSERT_FALSE(init_result.solved);

    auto cold_start_result = tracker.update({
        make_obs(1, center.z() + dz1), 0.01, Eigen::Isometry3d::Identity()});

    EXPECT_TRUE(cold_start_result.accepted_frame);
    EXPECT_TRUE(cold_start_result.initialized);
    EXPECT_TRUE(cold_start_result.cold_start);
    EXPECT_TRUE(cold_start_result.reset_reason.empty());
    EXPECT_TRUE(tracker.initialized());
}

TEST(GraphOptimizerTrackerCoreTest, OutpostOfflineSinglePlateSequenceRecoversThreeHeights) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 3;
    config.optimizer.update_iterations = 8;
    config.outpost_radius = 0.2765;
    config.outpost_s2qz = 0.0004;
    config.outpost_vel_sigma = 0.005;
    config.outpost_prior_noise.radius = 0.005;
    config.outpost_prior_noise.dz = 0.25;
    config.use_edge_reproj_factor = false;
    config.outpost_pixel_sigma = 1e9;
    config.pose_prior_sigma = 0.001;
    config.pose_prior_distance_scale = 0.0;
    config.outpost_geo_noise.tangential = 0.015;
    config.outpost_geo_noise.radial = 0.04;
    config.outpost_geo_noise.height = 0.08;
    config.outpost_geo_noise.yaw = 0.15;

    const Eigen::Vector3d center(1.0, 2.0, 1.5);
    const double yaw = 0.2;
    const std::array<double, 3> physical_z{
        center.z(), center.z() + 0.2, center.z() - 0.1};

    auto make_obs = [&](int slot) {
        auto_aim_interfaces::msg::Armors observations;
        auto_aim_interfaces::msg::Armor obs;
        const double armor_yaw =
            yaw + filter_test::graph_optimizer::armorYawOffset(slot, 3);
        obs.number = "outpost";
        obs.pose.position.x =
            center.x() - config.outpost_radius * std::cos(armor_yaw);
        obs.pose.position.y =
            center.y() - config.outpost_radius * std::sin(armor_yaw);
        obs.pose.position.z = physical_z[static_cast<std::size_t>(slot)];
        obs.pose.orientation.z = std::sin(armor_yaw / 2.0);
        obs.pose.orientation.w = std::cos(armor_yaw / 2.0);
        obs.yaw = armor_yaw;
        observations.armors.push_back(obs);
        return observations;
    };

    filter_test::graph_optimizer::ArmorGraphTracker tracker(config);
    filter_test::graph_optimizer::TrackerUpdateResult result;
    for (const auto& [slot, count] :
         std::array<std::pair<int, int>, 4>{{{0, 20}, {1, 80}, {2, 80}, {0, 20}}}) {
        for (int i = 0; i < count; ++i) {
            result = tracker.update({
                make_obs(slot), 0.01, Eigen::Isometry3d::Identity()});
            ASSERT_TRUE(result.initialized);
            ASSERT_TRUE(result.reset_reason.empty()) << result.reset_reason;
        }
    }

    ASSERT_TRUE(result.solved);
    ASSERT_FALSE(result.solve_failed);
    ASSERT_EQ(result.state.armor_count, 3);

    std::array<double, 3> actual_z{
        result.state.center.z(),
        result.state.center.z() + result.state.dz,
        result.state.center.z() + result.state.outpost_dz_2};
    auto expected_z = physical_z;
    std::sort(actual_z.begin(), actual_z.end());
    std::sort(expected_z.begin(), expected_z.end());

    for (std::size_t i = 0; i < actual_z.size(); ++i) {
        EXPECT_NEAR(actual_z[i], expected_z[i], 0.05)
            << "actual sorted z=[" << actual_z[0] << ", " << actual_z[1]
            << ", " << actual_z[2] << "]";
    }
}

TEST(GraphOptimizerTrackerCoreTest, RepeatedPoorMatchesResetTrackerLikeNisGate) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.match_quality_window_size = 3;
    config.match_quality_failure_threshold = 0.20;
    config.match_quality_failure_ratio = 0.60;
    config.match_max_cost = 1.5;
    filter_test::graph_optimizer::ArmorGraphTracker tracker(config);

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor init_obs;
    init_obs.pose.position.x = 1.0;
    init_obs.pose.position.y = 0.0;
    init_obs.pose.position.z = 0.5;
    init_obs.pose.orientation.w = 1.0;
    init_obs.yaw = 0.0;
    observations.armors.push_back(init_obs);

    auto init_result =
        tracker.update({observations, 0.01, Eigen::Isometry3d::Identity()});
    ASSERT_TRUE(init_result.initialized);

    auto_aim_interfaces::msg::Armors poor_observations;
    auto_aim_interfaces::msg::Armor poor_obs;
    poor_obs.pose.position.x = 1.10;
    poor_obs.pose.position.y = 0.0;
    poor_obs.pose.position.z = 0.5;
    poor_obs.pose.orientation.w = 1.0;
    poor_obs.yaw = 0.10;
    poor_observations.armors.push_back(poor_obs);

    for (int i = 0; i < 2; ++i) {
        auto result =
            tracker.update({poor_observations, 0.01, Eigen::Isometry3d::Identity()});
        EXPECT_TRUE(result.accepted_frame);
        EXPECT_TRUE(result.initialized);
    }

    auto reset_result =
        tracker.update({poor_observations, 0.01, Eigen::Isometry3d::Identity()});

    EXPECT_FALSE(reset_result.accepted_frame);
    EXPECT_FALSE(reset_result.initialized);
    EXPECT_FALSE(tracker.initialized());
}

TEST(GraphOptimizerTrackerCoreTest, GoodMatchesDoNotTripMatchQualityGate) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.optimizer.cold_start_frames = 0;
    config.match_quality_window_size = 3;
    config.match_quality_failure_threshold = 0.20;
    config.match_quality_failure_ratio = 0.60;
    filter_test::graph_optimizer::ArmorGraphTracker tracker(config);

    auto make_observations =
        [](const filter_test::graph_optimizer::PredictedArmor& pred) {
            auto_aim_interfaces::msg::Armors observations;
            auto_aim_interfaces::msg::Armor obs;
            obs.pose.position.x = pred.position.x();
            obs.pose.position.y = pred.position.y();
            obs.pose.position.z = pred.position.z();
            obs.pose.orientation.z = std::sin(pred.yaw / 2.0);
            obs.pose.orientation.w = std::cos(pred.yaw / 2.0);
            obs.yaw = pred.yaw;
            observations.armors.push_back(obs);
            return observations;
        };

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor obs;
    obs.pose.position.x = 1.0;
    obs.pose.position.y = 0.0;
    obs.pose.position.z = 0.5;
    obs.pose.orientation.w = 1.0;
    obs.yaw = 0.0;
    observations.armors.push_back(obs);

    auto init_result =
        tracker.update({observations, 0.01, Eigen::Isometry3d::Identity()});
    ASSERT_TRUE(init_result.initialized);
    ASSERT_FALSE(init_result.predicted_armors.empty());
    observations = make_observations(init_result.predicted_armors.front());

    for (int i = 0; i < 4; ++i) {
        auto result =
            tracker.update({observations, 0.01, Eigen::Isometry3d::Identity()});
        const std::string trace = result.match_costs.empty()
            ? "i=" + std::to_string(i) + " no cost"
            : "i=" + std::to_string(i) +
              " cost=" + std::to_string(result.match_costs.front());
        SCOPED_TRACE(trace);
        if (!result.match_costs.empty()) {
            EXPECT_LT(result.match_costs.front(), config.match_quality_failure_threshold);
        }
        EXPECT_TRUE(result.accepted_frame);
        EXPECT_TRUE(result.initialized);
        ASSERT_FALSE(result.predicted_armors.empty());
        observations = make_observations(result.predicted_armors.front());
    }

    EXPECT_TRUE(tracker.initialized());
}

TEST(GraphOptimizerTrackerCoreTest, MatchArmorsAssignsUniqueIndices) {
    filter_test::graph_optimizer::TrackerState state;
    state.center << 0.0, 0.0, 0.5;
    state.yaw = 0.0;
    state.radius_1 = 0.3;
    state.radius_2 = 0.4;
    state.dz = 0.1;

    auto predicted = filter_test::graph_optimizer::predictedArmorsFromState(state);
    auto_aim_interfaces::msg::Armors observations;
    for (int i = 0; i < 2; ++i) {
        auto_aim_interfaces::msg::Armor obs;
        obs.pose.position.x = predicted[static_cast<std::size_t>(i)].position.x();
        obs.pose.position.y = predicted[static_cast<std::size_t>(i)].position.y();
        obs.pose.position.z = predicted[static_cast<std::size_t>(i)].position.z();
        obs.yaw = predicted[static_cast<std::size_t>(i)].yaw;
        observations.armors.push_back(obs);
    }

    auto matched = filter_test::graph_optimizer::matchArmorIndicesUnique(
        state, observations, -1);

    ASSERT_EQ(matched.size(), 2u);
    EXPECT_NE(matched[0], matched[1]);
}

TEST(GraphOptimizerTrackerCoreTest, MatchArmorsRejectsLargeOutlier) {
    filter_test::graph_optimizer::TrackerState state;
    state.center << 0.0, 0.0, 0.5;
    state.yaw = 0.0;
    state.radius_1 = 0.3;
    state.radius_2 = 0.4;
    state.dz = 0.1;

    auto_aim_interfaces::msg::Armors observations;
    auto_aim_interfaces::msg::Armor obs;
    obs.pose.position.x = 10.0;
    obs.pose.position.y = -10.0;
    obs.pose.position.z = 3.0;
    obs.yaw = M_PI;
    observations.armors.push_back(obs);

    auto matched = filter_test::graph_optimizer::matchArmorIndicesUnique(
        state, observations, -1);

    ASSERT_EQ(matched.size(), 1u);
    EXPECT_EQ(matched[0], -1);
}

TEST(GraphOptimizerTrackerCoreTest, MatchArmorsMarksOverflowObservationsUnmatched) {
    filter_test::graph_optimizer::TrackerState state;
    state.center << 0.0, 0.0, 0.5;
    state.yaw = 0.0;
    state.radius_1 = 0.3;
    state.radius_2 = 0.4;
    state.dz = 0.1;

    auto predicted = filter_test::graph_optimizer::predictedArmorsFromState(state);
    auto_aim_interfaces::msg::Armors observations;
    for (int i = 0; i < 5; ++i) {
        auto_aim_interfaces::msg::Armor obs;
        const auto& pred = predicted[static_cast<std::size_t>(i % 4)];
        obs.pose.position.x = pred.position.x();
        obs.pose.position.y = pred.position.y();
        obs.pose.position.z = pred.position.z();
        obs.yaw = pred.yaw;
        observations.armors.push_back(obs);
    }

    auto matched = filter_test::graph_optimizer::matchArmorIndicesUnique(
        state, observations, -1);

    ASSERT_EQ(matched.size(), 5u);
    std::set<int> valid_indices;
    int unmatched_count = 0;
    for (int idx : matched) {
        if (idx < 0) {
            ++unmatched_count;
        } else {
            valid_indices.insert(idx);
        }
    }
    EXPECT_EQ(valid_indices.size(), 4u);
    EXPECT_EQ(unmatched_count, 1);
}

TEST(GraphOptimizerFrameTimeTest, RejectedFrameDoesNotCommitTimestamp) {
    filter_test::graph_optimizer::FrameTimeTracker tracker;
    builtin_interfaces::msg::Time t0;
    t0.sec = 10;
    builtin_interfaces::msg::Time t1;
    t1.sec = 11;
    builtin_interfaces::msg::Time t2;
    t2.sec = 12;

    EXPECT_NEAR(tracker.computeDt(t0), 0.01, 1e-12);
    tracker.commit(t0);

    EXPECT_NEAR(tracker.computeDt(t1), 1.0, 1e-12);
    // t1 is rejected before commit.

    EXPECT_NEAR(tracker.computeDt(t2), 0.01, 1e-12);
    tracker.commit(t2);
    EXPECT_NEAR(tracker.computeDt(t2), 0.01, 1e-12);
}

// ==================== 测试异常值检测 ====================

TEST(OutlierDetectionTest, GateCheck) {
    // 简单的门限检验
    double gate_threshold = 5.991;  // chi-square(2, 0.95)

    // 正常观测
    double nis_normal = 1.0;
    EXPECT_TRUE(nis_normal < gate_threshold);

    // 异常观测
    double nis_outlier = 100.0;
    EXPECT_FALSE(nis_outlier < gate_threshold);
}

TEST(OutlierDetectionTest, AdaptiveThreshold) {
    // 模拟自适应门限
    double base_threshold = 5.991;
    double adaptation_rate = 0.1;

    // 历史NIS值（合理范围）
    std::vector<double> history = {2.0, 3.0, 2.5, 3.5, 2.0};

    // 计算平均NIS
    double mean_nis = 0.0;
    for (double nis : history) {
        mean_nis += nis;
    }
    mean_nis /= history.size();

    // 期望的平均值（观测维度）
    double expected_mean = 2.0;

    // 自适应调整
    double ratio = mean_nis / expected_mean;
    double adaptive_threshold = base_threshold * (1.0 + adaptation_rate * (ratio - 1.0));

    // 验证自适应阈值在合理范围内
    EXPECT_GT(adaptive_threshold, base_threshold * 0.5);
    EXPECT_LT(adaptive_threshold, base_threshold * 3.0);
}

// ==================== 主函数 ====================

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

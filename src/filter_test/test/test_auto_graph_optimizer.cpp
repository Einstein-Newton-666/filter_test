#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <set>
#include <utility>

#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>

#include "filter_test/graph_optimizer/armor_tracker.hpp"
#include "filter_test/graph_optimizer_test.hpp"
#include "filter_test/visualization_marker_utils.hpp"

// 注意：由于GTSAM未安装C++版本，此测试仅验证不依赖GTSAM的部分
// 完整测试需要安装GTSAM C++库

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
    tf2::Matrix3x3 rotation(filter_test::observedArmorMarkerQuaternion(0.0));
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    rotation.getRPY(roll, pitch, yaw);

    EXPECT_NEAR(roll, 0.0, 1e-12);
    EXPECT_NEAR(pitch, 15.0 * M_PI / 180.0, 1e-12);
    EXPECT_NEAR(yaw, 0.0, 1e-12);
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

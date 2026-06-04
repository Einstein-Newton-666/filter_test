#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <set>
#include <utility>

#include <gtsam/nonlinear/PriorFactor.h>

#include "filter_test/auto_graph_optimizer/models/motion_model.hpp"
#include "filter_test/graph_optimizer/motion_models.hpp"
#include "filter_test/graph_optimizer_test.hpp"
#include "filter_test/graph_optimizer/tracker_core.hpp"
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

TEST(ArmorCenterFactorTest, YawJacobianMatchesFiniteDifference) {
    gtsam::Key k_armor = gtsam::Symbol('h', 0);
    gtsam::Key k_pos = gtsam::Symbol('a', 0);
    gtsam::Key k_yaw = gtsam::Symbol('b', 0);
    gtsam::Key k_radius = gtsam::Symbol('c', 0);
    gtsam::Key k_dz = gtsam::Symbol('d', 0);

    auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4::Ones());
    ArmorCenterFactor factor(k_armor, k_pos, k_yaw, k_radius, k_dz, noise,
                             1, Eigen::Isometry3d::Identity(), 0.10, 0.60);

    gtsam::Pose3 armor_pose(
        gtsam::Rot3::RzRyRx(0.08, 0.18, 0.6),
        gtsam::Point3(1.0, -2.0, 0.3));
    gtsam::Vector pos_vel(6);
    pos_vel << 1.4, 0.0, -1.5, 0.0, 0.45, 0.0;
    gtsam::Vector yaw_vyaw(2);
    yaw_vyaw << -0.2, 0.0;
    gtsam::Vector radius(2);
    radius << 0.0, 0.0;
    gtsam::Vector dz(1);
    dz << 0.12;

    gtsam::Matrix H1, H2, H3, H4, H5;
    gtsam::Vector err = factor.evaluateError(
        armor_pose, pos_vel, yaw_vyaw, radius, dz, &H1, &H2, &H3, &H4, &H5);

    const double eps = 1e-6;
    gtsam::Vector6 delta = gtsam::Vector6::Zero();
    delta(2) = eps;  // yaw-axis local rotation perturbation for this pose.
    gtsam::Pose3 perturbed = armor_pose.retract(delta);
    gtsam::Vector err_perturbed = factor.evaluateError(
        perturbed, pos_vel, yaw_vyaw, radius, dz, nullptr, nullptr, nullptr, nullptr, nullptr);
    double numeric = (err_perturbed[3] - err[3]) / eps;

    EXPECT_NEAR(H1(3, 2), numeric, 1e-5);
}

namespace {

struct Identity1DMotionModel : auto_graph::MotionModel<Identity1DMotionModel, 1> {
    explicit Identity1DMotionModel(double) {}

    template<typename T>
    void operator()(const T& x, T& x_next) const {
        x_next[0] = x[0];
    }
};

auto_graph::GraphOptimizer makeSingleVariableOptimizer(int cold_start_frames) {
    auto_graph::GraphOptimizerConfig config;
    config.cold_start_frames = cold_start_frames;
    config.verbose = false;
    auto_graph::GraphOptimizer optimizer(config);

    auto layout = auto_graph::VariableLayout(1).addDynamic("x", {0});
    Eigen::VectorXd x0(1);
    x0 << 0.0;
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(1, 1);
    optimizer.initialize(layout, x0, P0);
    return optimizer;
}

auto_graph::GraphOptimizer makeWindowResetOptimizer() {
    auto_graph::GraphOptimizerConfig config;
    config.cold_start_frames = 0;
    config.max_window_frames = 1;
    config.smoother_lag = 0.0;
    config.verbose = false;
    auto_graph::GraphOptimizer optimizer(config);

    auto layout = auto_graph::VariableLayout(1).addDynamic("x", {0});
    Eigen::VectorXd x0(1);
    x0 << 0.0;
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(1, 1);
    optimizer.initialize(layout, x0, P0);
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
    auto optimizer = makeSingleVariableOptimizer(1);
    optimizer.addCustomFactor<gtsam::PriorFactor<gtsam::Vector>>(
        optimizer.key("x", 0),
        gtsam::Vector1(0.0),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(1.0)));

    auto result = optimizer.solve();

    EXPECT_EQ(result.frame_id, 0u);
    EXPECT_FALSE(result.attempted);
    EXPECT_FALSE(result.optimized);
    EXPECT_TRUE(result.cold_start);
    EXPECT_FALSE(result.failed);
    EXPECT_TRUE(result.error_message.empty());
}

TEST(GraphOptimizerSolveResultTest, InvalidFactorReportsFailure) {
    auto optimizer = makeSingleVariableOptimizer(0);
    optimizer.addCustomFactor<gtsam::PriorFactor<gtsam::Pose3>>(
        optimizer.key("x", 0),
        gtsam::Pose3(),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Ones()));

    auto result = optimizer.solve();

    EXPECT_EQ(result.frame_id, 0u);
    EXPECT_TRUE(result.attempted);
    EXPECT_FALSE(result.optimized);
    EXPECT_FALSE(result.cold_start);
    EXPECT_TRUE(result.failed);
    EXPECT_FALSE(result.error_message.empty());
}

TEST(GraphOptimizerSolveResultTest, FrameIdReflectsWindowReset) {
    auto optimizer = makeWindowResetOptimizer();
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(1, 1) * 0.01;
    optimizer.predict<Identity1DMotionModel>(0.01, Q);
    ASSERT_EQ(optimizer.getFrameId(), 1u);

    optimizer.addCustomFactor<gtsam::PriorFactor<gtsam::Vector>>(
        optimizer.key("x", 1),
        gtsam::Vector1(0.0),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(1.0)));

    auto result = optimizer.solve();

    EXPECT_TRUE(result.optimized);
    EXPECT_FALSE(result.failed);
    EXPECT_EQ(optimizer.getFrameId(), 0u);
    EXPECT_EQ(result.frame_id, optimizer.getFrameId());
}

TEST(GraphOptimizerVisualizationTest, RadiusConversionUsesTrackerRadiusBounds) {
    EXPECT_NEAR(filter_test::graphOptimizerRadiusFromState(0.0), 0.35, 1e-12);
}

TEST(GraphOptimizerYPDModelTest, SingleArmorUsesLogisticRadiusState) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(11);
    x[0] = 1.0;
    x[2] = 0.0;
    x[4] = 0.0;
    x[6] = 0.0;
    x[8] = filter_test::graph_optimizer::radiusToState(0.25);
    x[9] = filter_test::graph_optimizer::radiusToState(0.40);

    filter_test::graph_optimizer::ArmorCVMeasureYPD model(0);
    const auto [z, H] = model.linearize(x);

    EXPECT_EQ(H.rows(), 4);
    EXPECT_EQ(H.cols(), 11);
    EXPECT_NEAR(z[0], 0.0, 1e-12);
    EXPECT_NEAR(z[1], 0.0, 1e-12);
    EXPECT_NEAR(z[2], 0.75, 1e-12);
    EXPECT_NEAR(z[3], 0.0, 1e-12);
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

TEST(GraphOptimizerTrackerCoreTest, CreatesRequestedObservationBackend) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.use_2d_observation = true;
    auto pixel_backend = filter_test::graph_optimizer::createObservationBackend(config);
    ASSERT_NE(pixel_backend, nullptr);
    EXPECT_STREQ(pixel_backend->name(), "pixel");

    config.use_2d_observation = false;
    auto ypd_backend = filter_test::graph_optimizer::createObservationBackend(config);
    ASSERT_NE(ypd_backend, nullptr);
    EXPECT_STREQ(ypd_backend->name(), "ypd");
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

TEST(GraphOptimizerTrackerCoreTest, MismatchedBackendInputFailsFrameWithoutSolve) {
    filter_test::graph_optimizer::TrackerConfig config;
    config.use_2d_observation = true;
    auto backend = filter_test::graph_optimizer::createObservationBackend(config);
    auto optimizer = makeSingleVariableOptimizer(0);

    auto_aim_interfaces::msg::Armors observations;
    observations.armors.emplace_back();
    Eigen::VectorXd predicted_state = Eigen::VectorXd::Zero(11);
    std::vector<int> matched_indices;
    filter_test::graph_optimizer::TrackerFrameContext context{
        config,
        observations,
        Eigen::Isometry3d::Identity(),
        predicted_state,
        matched_indices,
        0};

    auto result = backend->addFrameFactors(optimizer, context);

    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.error_message.find("matched_indices"), std::string::npos);
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
        obs.pose.position.x = predicted[0].position.x() + 0.01 * i;
        obs.pose.position.y = predicted[0].position.y();
        obs.pose.position.z = predicted[0].position.z();
        obs.yaw = predicted[0].yaw;
        observations.armors.push_back(obs);
    }

    auto matched = filter_test::graph_optimizer::matchArmorIndicesUnique(
        state, observations, -1);

    ASSERT_EQ(matched.size(), 2u);
    EXPECT_NE(matched[0], matched[1]);
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

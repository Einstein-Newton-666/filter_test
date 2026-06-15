#include <gtest/gtest.h>

#include "armor_simulation/camera_model.hpp"
#include "armor_simulation/armor_geometry.hpp"
#include "armor_simulation/detection_noise.hpp"
#include "armor_simulation/pnp_pose_utils.hpp"
#include "armor_simulation/rune_geometry.hpp"
#include "armor_simulation/simulation_common.hpp"

#include <angles/angles.h>
#include <armor_simulation/msg/rune_ground_truth.hpp>
#include <auto_aim_interfaces/msg/rune_info.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <Eigen/Dense>
#include <cmath>

TEST(CameraModelTest, EstimatePoseReportsFailureForDegeneratePixels) {
    armor_sim::CameraModel camera;
    std::array<Eigen::Vector2d, 4> pixels = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(0.0, 0.0),
    };

    auto result = camera.estimatePose(pixels, armor_sim::SMALL_ARMOR_WIDTH, armor_sim::SMALL_ARMOR_HEIGHT);

    EXPECT_FALSE(result.success);
}

TEST(SimulationCommonTest, CameraInfoConvertsToCameraIntrinsics) {
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.width = 1920;
    camera_info.height = 1440;
    camera_info.k = {2411.0, 0.0, 720.0,
                     0.0, 2411.0, 640.0,
                     0.0, 0.0, 1.0};
    camera_info.d = {-0.093, 0.154, 0.0001, -0.0006, 0.0};

    const auto intr = armor_sim::cameraIntrinsicsFromCameraInfo(camera_info);

    EXPECT_DOUBLE_EQ(intr.fx, 2411.0);
    EXPECT_DOUBLE_EQ(intr.fy, 2411.0);
    EXPECT_DOUBLE_EQ(intr.cx, 720.0);
    EXPECT_DOUBLE_EQ(intr.cy, 640.0);
    EXPECT_DOUBLE_EQ(intr.k1, -0.093);
    EXPECT_DOUBLE_EQ(intr.k2, 0.154);
    EXPECT_DOUBLE_EQ(intr.p1, 0.0001);
    EXPECT_DOUBLE_EQ(intr.p2, -0.0006);
    EXPECT_EQ(intr.image_width, 1920);
    EXPECT_EQ(intr.image_height, 1440);
}

TEST(SimulationCommonTest, CameraInfoRejectsMissingDistortion) {
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.width = 1920;
    camera_info.height = 1440;
    camera_info.k = {2411.0, 0.0, 720.0,
                     0.0, 2411.0, 640.0,
                     0.0, 0.0, 1.0};
    camera_info.d = {-0.093, 0.154};

    EXPECT_THROW(
        armor_sim::cameraIntrinsicsFromCameraInfo(camera_info),
        std::runtime_error);
}

TEST(DetectionNoiseTest, FullyCommonPixelNoisePreservesArmorShape) {
    armor_sim::DetectionNoiseParams params;
    params.pixel_noise_optimal = 2.0;
    params.pixel_noise_optimal_distance = 5.0;
    params.pixel_noise_curvature = 0.0;
    params.pixel_noise_common_ratio = 1.0;
    params.use_outliers = false;
    armor_sim::DetectionNoise noise(params, 7);

    std::array<Eigen::Vector2d, 4> pixels = {
        Eigen::Vector2d(10.0, 20.0),
        Eigen::Vector2d(10.0, 40.0),
        Eigen::Vector2d(50.0, 40.0),
        Eigen::Vector2d(50.0, 20.0),
    };

    auto noisy = noise.addArmorPixelNoise(pixels, 5.0);

    Eigen::Vector2d shift = noisy[0] - pixels[0];
    for (size_t i = 1; i < pixels.size(); ++i) {
        EXPECT_TRUE((noisy[i] - pixels[i]).isApprox(shift, 1e-12));
    }
}

TEST(DetectionNoiseTest, FullyCommonPixelNoisePreservesFivePointShape) {
    armor_sim::DetectionNoiseParams params;
    params.pixel_noise_optimal = 2.0;
    params.pixel_noise_optimal_distance = 5.0;
    params.pixel_noise_curvature = 0.0;
    params.pixel_noise_common_ratio = 1.0;
    params.use_outliers = false;
    armor_sim::DetectionNoise noise(params, 11);

    std::array<Eigen::Vector2d, 5> pixels = {
        Eigen::Vector2d(10.0, 20.0),
        Eigen::Vector2d(12.0, 25.0),
        Eigen::Vector2d(20.0, 30.0),
        Eigen::Vector2d(30.0, 25.0),
        Eigen::Vector2d(28.0, 20.0),
    };

    auto noisy = noise.addCorrelatedPixelNoise(pixels, 5.0);

    Eigen::Vector2d shift = noisy[0] - pixels[0];
    for (size_t i = 1; i < pixels.size(); ++i) {
        EXPECT_TRUE((noisy[i] - pixels[i]).isApprox(shift, 1e-12));
    }
}

TEST(PnPPoseUtilsTest, AmbiguityCorrectionKeepsQuaternionYawConsistent) {
    const double expected_yaw = 0.4;
    Eigen::Quaterniond orientation(
        Eigen::AngleAxisd(expected_yaw + M_PI, Eigen::Vector3d::UnitZ()));

    auto corrected = armor_sim::correctPlanarPnPAmbiguity(orientation, expected_yaw);

    EXPECT_NEAR(
        angles::shortest_angular_distance(corrected.yaw, expected_yaw),
        0.0, 1e-12);
    EXPECT_NEAR(
        angles::shortest_angular_distance(
            armor_sim::yawFromRotation(corrected.orientation.toRotationMatrix()),
            corrected.yaw),
        0.0, 1e-12);
}

TEST(RuneGeometryTest, BladeLocalPointsFollowDetectorOrder) {
    const auto points = armor_sim::runeBladeLocalPoints();

    ASSERT_EQ(points.size(), 5u);
    EXPECT_TRUE(points[0].isApprox(Eigen::Vector3d(0.0, 0.0, 0.0), 1e-12));
    EXPECT_TRUE(points[1].isApprox(Eigen::Vector3d(0.0, 0.0, 0.5415), 1e-12));
    EXPECT_TRUE(points[2].isApprox(
        Eigen::Vector3d(0.0, 0.160, armor_sim::RUNE_BLADE_RADIUS), 1e-12));
    EXPECT_TRUE(points[3].isApprox(Eigen::Vector3d(0.0, 0.0, 0.8585), 1e-12));
    EXPECT_TRUE(points[4].isApprox(
        Eigen::Vector3d(0.0, -0.160, armor_sim::RUNE_BLADE_RADIUS), 1e-12));
}

TEST(RuneGeometryTest, BladeCornerPointsFormConvexTarget) {
    const auto points = armor_sim::runeBladeLocalPoints();
    const std::array<size_t, 4> polygon{1, 2, 3, 4};
    double previous_cross = 0.0;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const Eigen::Vector2d a(points[polygon[i]].y(), points[polygon[i]].z());
        const Eigen::Vector2d b(points[polygon[(i + 1) % polygon.size()]].y(),
                                points[polygon[(i + 1) % polygon.size()]].z());
        const Eigen::Vector2d c(points[polygon[(i + 2) % polygon.size()]].y(),
                                points[polygon[(i + 2) % polygon.size()]].z());
        const Eigen::Vector2d ab = b - a;
        const Eigen::Vector2d bc = c - b;
        const double cross = ab.x() * bc.y() - ab.y() * bc.x();
        EXPECT_GT(std::abs(cross), 1e-12);
        if (i > 0) {
            EXPECT_GT(previous_cross * cross, 0.0);
        }
        previous_cross = cross;
    }
}

TEST(RuneGeometryTest, TargetPositionUsesHitCenter) {
    const Eigen::Vector3d center(0.0, -5.0, 1.5);
    const double normal_yaw = 0.4;
    const double normal_pitch = -0.2;
    const double roll = 0.3;
    const int blade_index = 2;

    const auto world_points = armor_sim::computeRuneBladeWorldPoints(
        center, normal_yaw, normal_pitch,
        roll + static_cast<double>(blade_index) * armor_sim::RUNE_SLOT_ANGLE);
    const auto target = armor_sim::computeRuneBladeTargetPosition(
        center, normal_yaw, normal_pitch, roll, blade_index);
    const Eigen::Matrix3d R =
        armor_sim::runeBaseRotation(normal_yaw, normal_pitch) *
        Eigen::AngleAxisd(
            roll + static_cast<double>(blade_index) * armor_sim::RUNE_SLOT_ANGLE,
            Eigen::Vector3d::UnitX()).toRotationMatrix();

    EXPECT_TRUE(target.isApprox(
        center + R * Eigen::Vector3d(0.0, 0.0, armor_sim::RUNE_BLADE_RADIUS), 1e-12));
    EXPECT_FALSE(target.isApprox(
        world_points[static_cast<size_t>(armor_sim::RunePointIndex::Far)], 1e-6));
    EXPECT_FALSE(target.isApprox(center, 1e-6));
}

TEST(RuneMotionTest, BigRuneRandomVelocityIsSeededAndNoisy) {
    armor_sim::BigRuneVelocityConfig config;
    config.random_seed = 123;
    config.velocity_noise_std = 0.05;

    armor_sim::BigRuneMotionModel model_a(config);
    armor_sim::BigRuneMotionModel model_b(config);

    const double v0_a = model_a.sampleVelocity(0.01);
    const double v0_b = model_b.sampleVelocity(0.01);
    const double v1_a = model_a.sampleVelocity(0.01);

    EXPECT_NEAR(v0_a, v0_b, 1e-12);
    EXPECT_NE(v0_a, v1_a);
    EXPECT_GE(std::abs(v0_a), config.min_abs_velocity);
    EXPECT_LE(std::abs(v0_a), config.max_abs_velocity);
}

TEST(RuneSimulationTest, ActiveBladeSelectorUsesModeCountsAndSwitchInterval) {
    armor_sim::RuneActiveBladeSelector selector(5, 3.0, 42);

    selector.update(0.0, 1);
    const auto first_small = selector.activeMask();
    EXPECT_EQ(selector.activeCount(), 1);

    selector.update(2.9, 1);
    EXPECT_EQ(selector.activeMask(), first_small);

    selector.update(3.1, 2);
    EXPECT_EQ(selector.activeCount(), 2);
    EXPECT_NE(selector.activeMask(), first_small);
}

TEST(RuneSimulationTest, TargetBladeSelectionPrefersObservedThenActive) {
    EXPECT_EQ(armor_sim::selectRuneTargetBlade({3, 1}, {0, 4}), 3);
    EXPECT_EQ(armor_sim::selectRuneTargetBlade({}, {4, 2}), 4);
    EXPECT_EQ(armor_sim::selectRuneTargetBlade({}, {}), 0);
    EXPECT_EQ(armor_sim::selectRuneTargetBlade({8}, {}), 4);
    EXPECT_EQ(armor_sim::selectRuneTargetBlade({-2}, {}), 0);
}

TEST(RuneSimulationTest, RuneDebugMessagesCarryPlaneYawPitch) {
    armor_simulation::msg::RuneGroundTruth truth;
    truth.normal_yaw = 1.2;
    truth.normal_pitch = -0.3;
    EXPECT_DOUBLE_EQ(truth.normal_yaw, 1.2);
    EXPECT_DOUBLE_EQ(truth.normal_pitch, -0.3);

    auto_aim_interfaces::msg::RuneInfo info;
    info.normal_yaw = 1.2;
    info.normal_pitch = -0.3;
    EXPECT_DOUBLE_EQ(info.normal_yaw, 1.2);
    EXPECT_DOUBLE_EQ(info.normal_pitch, -0.3);
}

TEST(OutpostGeometryTest, ThreePlateLayoutUsesConfiguredOffsets) {
    armor_sim::OutpostGeometryConfig config;
    config.radius = 0.2765;
    config.dz_1 = 0.2;
    config.dz_2 = -0.1;

    const auto plates = armor_sim::computeOutpostPlates(
        Eigen::Vector3d(1.0, 2.0, 0.5), 0.3,
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()).toRotationMatrix(),
        config);

    ASSERT_EQ(plates.size(), 3u);
    EXPECT_EQ(plates[0].index, 0);
    EXPECT_EQ(plates[1].index, 1);
    EXPECT_EQ(plates[2].index, 2);
    EXPECT_NEAR(plates[0].position.z(), 0.5, 1e-12);
    EXPECT_NEAR(plates[1].position.z(), 0.7, 1e-12);
    EXPECT_NEAR(plates[2].position.z(), 0.4, 1e-12);
    EXPECT_NEAR((plates[0].position.head<2>() - Eigen::Vector2d(1.0, 2.0)).norm(),
                config.radius, 1e-12);
}

TEST(OutpostMotionTest, FixedAngularVelocityKeepsConfiguredCenter) {
    armor_sim::OutpostMotionConfig config;
    config.center = Eigen::Vector3d(1.2, -4.3, 0.6);
    config.initial_yaw = 0.35;
    config.angular_velocity = 0.8 * M_PI;

    const auto state = armor_sim::computeOutpostMotionState(config, 0.5);
    const auto clamped_state = armor_sim::computeOutpostMotionState(config, -1.0);

    EXPECT_TRUE(state.center.isApprox(config.center, 1e-12));
    EXPECT_NEAR(state.yaw, config.initial_yaw + config.angular_velocity * 0.5, 1e-12);
    EXPECT_NEAR(state.angular_velocity, config.angular_velocity, 1e-12);

    EXPECT_TRUE(clamped_state.center.isApprox(config.center, 1e-12));
    EXPECT_NEAR(clamped_state.yaw, config.initial_yaw, 1e-12);
    EXPECT_NEAR(clamped_state.angular_velocity, config.angular_velocity, 1e-12);
}

TEST(OutpostModelTest, DedicatedConfigBuildsMotionFromOutpostParameters) {
    armor_sim::OutpostModelConfig config;
    config.center = Eigen::Vector3d(0.8, -3.7, 1.1);
    config.body_roll = 0.2;
    config.body_pitch = -0.1;
    config.angular_velocity = -1.7;

    const auto motion_config = armor_sim::outpostMotionConfig(config);
    const auto body_rotation = armor_sim::outpostBodyRotation(config);
    const Eigen::Matrix3d expected_rotation =
        (Eigen::AngleAxisd(config.body_roll, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(config.body_pitch, Eigen::Vector3d::UnitY()))
            .toRotationMatrix();

    EXPECT_TRUE(motion_config.center.isApprox(config.center, 1e-12));
    EXPECT_NEAR(motion_config.initial_yaw, 0.0, 1e-12);
    EXPECT_NEAR(motion_config.angular_velocity, -1.7, 1e-12);
    EXPECT_TRUE(body_rotation.isApprox(expected_rotation, 1e-12));
}

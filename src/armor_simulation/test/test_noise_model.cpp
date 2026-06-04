#include <gtest/gtest.h>

#include "armor_simulation/camera_model.hpp"
#include "armor_simulation/armor_geometry.hpp"
#include "armor_simulation/detection_noise.hpp"
#include "armor_simulation/pnp_pose_utils.hpp"

#include <angles/angles.h>

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

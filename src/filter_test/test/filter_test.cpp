#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "filter_test/filter.hpp"
#include "filter_test/filters/cv_model.hpp"
#include "filter_test/filters/singer_model.hpp"

TEST(filter, filter_test)
{
    ArmorFilter af;
    auto armors_msg = std::make_shared<auto_aim_interfaces::msg::Armors>();
    auto_aim_interfaces::msg::Armor armor;
    tf2::Quaternion q;
    q.setRPY(0, 0, 1.5708);  // 绕 Z 轴旋转 90 度（弧度）
    armor.pose.orientation.x = q.x();
    armor.pose.orientation.y = q.y();
    armor.pose.orientation.z = q.z();
    armor.pose.orientation.w = q.w();
    armor.pose.position.x = 1.0;
    armor.pose.position.y = 2.0;
    armor.pose.position.z = 0.5;
    armors_msg->armors.push_back(armor);

    armors_msg->header.stamp = rclcpp::Clock().now();
    af.init(armors_msg);
    armors_msg->header.stamp = rclcpp::Clock().now() + rclcpp::Duration::from_seconds(0.01);
    af.update(armors_msg);

    // ASSERT_EQ(4, 2 + 2);
}

TEST(filter, CvOutpostMeasureUsesThreePlateGeometry)
{
    std::vector<double> x(11);
    x[0] = 1.0;
    x[1] = 0.0;
    x[2] = 2.0;
    x[3] = 0.0;
    x[4] = 0.5;
    x[5] = 0.0;
    x[6] = 0.1;
    x[7] = 0.0;
    x[8] = 0.2765;
    x[9] = 0.2;
    x[10] = -0.1;

    cv_model::MeasureOutpost measure({0, 1, 2});
    std::vector<double> z(12, 0.0);
    measure(x, z);

    ASSERT_EQ(measure.input_size, 11);
    ASSERT_EQ(measure.output_size, 12);
    for (int i = 0; i < 3; ++i) {
        const double ay = x[6] + static_cast<double>(i) * 2.0 * M_PI / 3.0;
        const double dz = i == 0 ? 0.0 : (i == 1 ? x[9] : x[10]);
        const double ax = x[0] - x[8] * std::cos(ay);
        const double ay_pos = x[2] - x[8] * std::sin(ay);
        const double az = x[4] + dz;
        EXPECT_NEAR(z[i * 4 + 0], std::atan2(ay_pos, ax), 1e-12);
        EXPECT_NEAR(
            z[i * 4 + 1],
            std::atan2(az, std::sqrt(ax * ax + ay_pos * ay_pos)), 1e-12);
        EXPECT_NEAR(
            z[i * 4 + 2],
            std::sqrt(ax * ax + ay_pos * ay_pos + az * az), 1e-12);
        EXPECT_NEAR(z[i * 4 + 3], ay, 1e-12);
    }
}

TEST(filter, StandardInitUsesConfiguredRadius)
{
    constexpr double custom_init_r = 0.37;
    ArmorFilter af(
        true, true,
        0.1, 0.1, 0.5, 10.0, 0.1,
        0.05, 0.05, 0.1, 10.0, 0.01,
        2.0,
        0.01, 2.0, 0.0,
        0.2765,
        custom_init_r);

    auto armors_msg = std::make_shared<auto_aim_interfaces::msg::Armors>();
    auto_aim_interfaces::msg::Armor armor;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    armor.pose.orientation = tf2::toMsg(q);
    armor.pose.position.x = 1.0;
    armor.pose.position.y = 2.0;
    armor.pose.position.z = 0.5;
    armor.number = "4";
    armors_msg->armors.push_back(armor);
    armors_msg->header.stamp = rclcpp::Clock().now();

    af.init(armors_msg);

    const auto result = af.get_last_result();
    EXPECT_NEAR(result[0], 1.0 + custom_init_r, 1e-12);
    EXPECT_NEAR(result[1], 2.0, 1e-12);
    EXPECT_NEAR(result[2], 0.5, 1e-12);
    EXPECT_NEAR(result[8], custom_init_r, 1e-12);
    EXPECT_NEAR(result[9], custom_init_r, 1e-12);
}

TEST(filter, SingerOutpostMeasureUsesThreePlateGeometry)
{
    std::vector<double> x(15);
    x[0] = 1.0;
    x[1] = 0.0;
    x[2] = 0.0;
    x[3] = 2.0;
    x[4] = 0.0;
    x[5] = 0.0;
    x[6] = 0.5;
    x[7] = 0.0;
    x[8] = 0.0;
    x[9] = -0.2;
    x[10] = 0.0;
    x[11] = 0.0;
    x[12] = 0.2765;
    x[13] = 0.2;
    x[14] = -0.1;

    singer_model::MeasureOutpost measure({0, 1, 2});
    std::vector<double> z(12, 0.0);
    measure(x, z);

    ASSERT_EQ(measure.input_size, 15);
    ASSERT_EQ(measure.output_size, 12);
    for (int i = 0; i < 3; ++i) {
        const double ay = x[9] + static_cast<double>(i) * 2.0 * M_PI / 3.0;
        const double dz = i == 0 ? 0.0 : (i == 1 ? x[13] : x[14]);
        const double ax = x[0] - x[12] * std::cos(ay);
        const double ay_pos = x[3] - x[12] * std::sin(ay);
        const double az = x[6] + dz;
        EXPECT_NEAR(z[i * 4 + 0], std::atan2(ay_pos, ax), 1e-12);
        EXPECT_NEAR(
            z[i * 4 + 1],
            std::atan2(az, std::sqrt(ax * ax + ay_pos * ay_pos)), 1e-12);
        EXPECT_NEAR(
            z[i * 4 + 2],
            std::sqrt(ax * ax + ay_pos * ay_pos + az * az), 1e-12);
        EXPECT_NEAR(z[i * 4 + 3], ay, 1e-12);
    }
}

TEST(filter, CvMeasureRUsesFiniteYawNoiseAtZeroYawDeviation)
{
    Eigen::VectorXd z(4);
    z << 0.0, 0.0, 5.0, 0.0;
    std::vector<double> abs_yaws{0.0};

    const auto R = cv_model::measure_r(z, 0.01, 0.01, 0.05, abs_yaws, false);

    EXPECT_TRUE(std::isfinite(R(3, 3)));
    EXPECT_NEAR(R(3, 3), 0.05, 1e-12);
}

TEST(filter, OutpostPredictQUsesDzNoiseForBothHeightOffsets)
{
    constexpr double dt = 0.1;
    constexpr double s2qxy = 0.003;
    constexpr double s2qz = 0.002;
    constexpr double s2qyaw = 0.12;
    constexpr double s2qr = 0.03;
    constexpr double s2qdz = 0.015;
    constexpr double tau = 2.0;

    const auto cv_q =
        cv_model::predict_outpost_q(dt, s2qxy, s2qz, s2qyaw, s2qr, s2qdz);
    const double cv_radius_q = s2qr * std::pow(dt, 4) / 4.0;
    const double cv_dz_q = s2qdz * std::pow(dt, 4) / 4.0;
    EXPECT_NEAR(cv_q(8, 8), cv_radius_q, 1e-15);
    EXPECT_NEAR(cv_q(9, 9), cv_dz_q, 1e-15);
    EXPECT_NEAR(cv_q(10, 10), cv_dz_q, 1e-15);

    const auto singer_q =
        singer_model::predict_outpost_q(
            dt, s2qxy, s2qz, s2qyaw, s2qr, s2qdz, tau);
    const double singer_radius_q = s2qr * std::pow(dt, 4) / 4.0;
    const double singer_dz_q = s2qdz * std::pow(dt, 4) / 4.0;
    EXPECT_NEAR(singer_q(12, 12), singer_radius_q, 1e-15);
    EXPECT_NEAR(singer_q(13, 13), singer_dz_q, 1e-15);
    EXPECT_NEAR(singer_q(14, 14), singer_dz_q, 1e-15);
}

TEST(filter, OutpostInitUsesObservedDzOffsets)
{
    ArmorFilter af(true, true,
        0.5, 1.0, 0.5, 10.0, 1.0,
        0.5, 1.0, 0.5, 10.0, 1.0,
        1.0, 0.001, 2.0, 0.0,
        0.2765);
    auto armors_msg = std::make_shared<auto_aim_interfaces::msg::Armors>();
    const double center_yaw = 0.0;
    const double radius = 0.2765;
    const Eigen::Vector3d center(1.0, 2.0, 0.5);
    constexpr double dz1 = 0.15;
    constexpr double dz2 = -0.06;

    auto make_armor = [&](int index, double dz) {
        auto_aim_interfaces::msg::Armor armor;
        const double armor_yaw = center_yaw + index * 2.0 * M_PI / 3.0;
        armor.number = "outpost";
        armor.priority = 2 - index;
        armor.pose.position.x = center.x() - radius * std::cos(armor_yaw);
        armor.pose.position.y = center.y() - radius * std::sin(armor_yaw);
        armor.pose.position.z = center.z() + dz;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, armor_yaw);
        armor.pose.orientation = tf2::toMsg(q);
        return armor;
    };

    armors_msg->armors.push_back(make_armor(0, 0.0));
    armors_msg->armors.push_back(make_armor(2, dz2));
    armors_msg->armors.push_back(make_armor(1, dz1));
    armors_msg->header.stamp = rclcpp::Clock().now();

    af.init(armors_msg);
    const auto result = af.get_last_result();

    EXPECT_NEAR(result[0], center.x(), 1e-12);
    EXPECT_NEAR(result[1], center.y(), 1e-12);
    EXPECT_NEAR(result[2], center.z(), 1e-12);
    EXPECT_NEAR(result[6], center_yaw, 1e-12);
    EXPECT_NEAR(result[8], radius, 1e-12);
    EXPECT_NEAR(result[9], dz1, 1e-12);
    EXPECT_NEAR(result[10], dz2, 1e-12);
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

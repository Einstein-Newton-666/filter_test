#include "filter_test/graph_optimizer/observation_backend.hpp"

#include "filter_test/graph_optimizer/factors.hpp"
#include "filter_test/graph_optimizer/motion_models.hpp"
#include "filter_test/graph_optimizer/tracker_math.hpp"
#include "filter_test/common.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace filter_test::graph_optimizer {
namespace {

std::array<Eigen::Vector3d, 4> smallArmorCornersLocal() {
    // autoaim 装甲板局部系: X=法向, Y=宽度, Z=高度; 顺序 LB,LT,RT,RB。
    return {{
        {0.0, -0.0675, -0.0625},
        {0.0, -0.0675,  0.0625},
        {0.0,  0.0675,  0.0625},
        {0.0,  0.0675, -0.0625},
    }};
}

gtsam::Pose3 armorPoseCameraFromObservation(
    const auto_aim_interfaces::msg::Armor& armor,
    const Eigen::Isometry3d& T_camera_to_odom) {
    const Eigen::Isometry3d T_odom_to_camera = T_camera_to_odom.inverse();
    Eigen::Vector3d armor_pos_odom(
        armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);
    Eigen::Vector3d armor_pos_camera = T_odom_to_camera * armor_pos_odom;

    tf2::Quaternion q_armor_odom;
    tf2::fromMsg(armor.pose.orientation, q_armor_odom);
    Eigen::Quaterniond eq_armor_odom(
        q_armor_odom.w(), q_armor_odom.x(), q_armor_odom.y(), q_armor_odom.z());
    Eigen::Quaterniond eq_armor_camera(
        T_odom_to_camera.rotation() * eq_armor_odom.toRotationMatrix());

    return {
        gtsam::Rot3::Quaternion(
            eq_armor_camera.w(), eq_armor_camera.x(),
            eq_armor_camera.y(), eq_armor_camera.z()),
        gtsam::Point3(
            armor_pos_camera.x(), armor_pos_camera.y(), armor_pos_camera.z())};
}

Eigen::Vector4d ypdObservation(const auto_aim_interfaces::msg::Armor& armor,
                               double r_pose, double r_distance, double r_yaw,
                               Eigen::MatrixXd& R_out) {
    const double x = armor.pose.position.x;
    const double y = armor.pose.position.y;
    const double z = armor.pose.position.z;
    const double yaw_obs = std::atan2(y, x);
    const double xy_norm = std::sqrt(x * x + y * y);
    const double distance = std::sqrt(x * x + y * y + z * z);
    const double pitch = std::atan2(z, xy_norm);

    R_out = Eigen::MatrixXd::Zero(4, 4);
    R_out(0, 0) = r_pose;
    R_out(1, 1) = r_pose;
    R_out(2, 2) = r_distance * distance * distance;
    R_out(3, 3) = r_yaw;

    return {yaw_obs, pitch, distance, armor.yaw};
}

class PixelObservationBackend final : public ObservationBackend {
public:
    const char* name() const override { return "pixel"; }

private:
    AddFrameFactorsResult addFrameFactorsImpl(
        auto_graph::GraphOptimizer& optimizer,
        const TrackerFrameContext& context) override {
        const uint64_t fid = context.frame_id;
        auto k_pos_vel = optimizer.key("pos_vel", fid);
        auto k_yaw_vyaw = optimizer.key("yaw_vyaw", fid);
        auto k_radius = optimizer.key("radius", 0);
        auto k_dz = optimizer.key("dz", 0);

        optimizer.addCustomFactor<VelSmoothFactor>(
            optimizer.key("pos_vel", fid - 1), k_pos_vel,
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(
                context.config.vel_sigma,
                context.config.vel_sigma,
                context.config.vel_sigma)));

        optimizer.addCustomFactor<VyawSmoothFactor>(
            optimizer.key("yaw_vyaw", fid - 1), k_yaw_vyaw,
            gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector1(context.config.vyaw_sigma)));

        const auto corners_local = smallArmorCornersLocal();
        auto pixel_noise = gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector2(context.config.pixel_sigma, context.config.pixel_sigma));
        gtsam::Vector6 prior_sig;
        prior_sig << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        auto pose_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(prior_sig);
        gtsam::Vector4 geo_sig(
            context.config.geo_noise.tangential,
            context.config.geo_noise.radial,
            context.config.geo_noise.height,
            context.config.geo_noise.yaw);
        auto geo_noise = gtsam::noiseModel::Diagonal::Sigmas(geo_sig);

        for (std::size_t ai = 0; ai < context.armors_msg.armors.size(); ++ai) {
            const auto& armor = context.armors_msg.armors[ai];
            const int idx = context.matched_indices[ai];
            if (idx < 0) {
                continue;
            }
            gtsam::Key akey = optimizer.armorPoseKey(fid, idx);
            const gtsam::Pose3 armor_pose_camera =
                armorPoseCameraFromObservation(armor, context.T_camera_to_odom);

            optimizer.insertArmorPose(akey, armor_pose_camera);
            for (int ci = 0; ci < 4; ++ci) {
                if (static_cast<std::size_t>(ci) >= armor.detected_points.size()) {
                    continue;
                }
                Eigen::Vector2d px_obs(
                    armor.detected_points[ci].x, armor.detected_points[ci].y);
                optimizer.addCustomFactor<ArmorReprojFactor>(
                    akey, pixel_noise, corners_local[ci],
                    context.config.camera_matrix,
                    context.config.distortion,
                    px_obs);
            }

            optimizer.addPose3Prior(akey, armor_pose_camera, pose_prior_noise);
            optimizer.addCustomFactor<ArmorCenterFactor>(
                akey, k_pos_vel, k_yaw_vyaw, k_radius, k_dz, geo_noise, idx,
                context.T_camera_to_odom, kRadiusMin, kRadiusMax);
        }
        return {};
    }
};

class YpdObservationBackend final : public ObservationBackend {
public:
    const char* name() const override { return "ypd"; }

private:
    AddFrameFactorsResult addFrameFactorsImpl(
        auto_graph::GraphOptimizer& optimizer,
        const TrackerFrameContext& context) override {
        const auto& armors = context.armors_msg.armors;
        if (armors.size() >= 2) {
            addDoubleArmorObservation(optimizer, context);
        } else if (!armors.empty()) {
            addSingleArmorObservation(optimizer, context);
        }
        return {};
    }

    static void addDoubleArmorObservation(auto_graph::GraphOptimizer& optimizer,
                                          const TrackerFrameContext& context) {
        constexpr std::array<std::pair<int, int>, 4> adjacent_pairs = {{
            {0, 1}, {1, 2}, {2, 3}, {3, 0}
        }};
        const auto& armors = context.armors_msg.armors;
        const double yaw1 = armors[0].yaw;
        const double yaw2 = armors[1].yaw;
        auto best_pair = adjacent_pairs[0];
        double min_total_diff = std::numeric_limits<double>::max();

        for (const auto& pair : adjacent_pairs) {
            double diff1 =
                std::abs(auto_graph::shortestAngularDistance(pair.first * M_PI_2, yaw1)) +
                std::abs(auto_graph::shortestAngularDistance(pair.second * M_PI_2, yaw2));
            double diff2 =
                std::abs(auto_graph::shortestAngularDistance(pair.second * M_PI_2, yaw1)) +
                std::abs(auto_graph::shortestAngularDistance(pair.first * M_PI_2, yaw2));
            double total_diff = std::min(diff1, diff2);
            if (total_diff < min_total_diff) {
                min_total_diff = total_diff;
                best_pair = diff1 < diff2
                    ? std::make_pair(pair.first, pair.second)
                    : std::make_pair(pair.second, pair.first);
            }
        }

        Eigen::VectorXd z = Eigen::VectorXd::Zero(8);
        Eigen::MatrixXd R_mat = Eigen::MatrixXd::Zero(8, 8);
        for (int i = 0; i < 2; ++i) {
            Eigen::MatrixXd R_single;
            Eigen::Vector4d z_single = ypdObservation(
                armors[i], context.config.r_pose, context.config.r_distance,
                context.config.r_yaw, R_single);
            const int off = i * 4;
            z.segment<4>(off) = z_single;
            R_mat.block<4, 4>(off, off) = R_single;
        }

        for (int i = 0; i < 2; ++i) {
            const int off = i * 4;
            const int idx = (i == 0) ? best_pair.first : best_pair.second;
            const double pred_yaw =
                std::atan2(context.predicted_state[2], context.predicted_state[0]) +
                idx * M_PI_2;
            z[off] = get_closest_angle(z[off], pred_yaw);
            z[off + 3] = get_closest_angle(
                z[off + 3], context.predicted_state[6] + M_PI_2 * idx);
        }

        optimizer.update<ArmorCVMeasureYPDDouble>(
            ArmorCVMeasureYPDDouble(best_pair.first, best_pair.second), z, R_mat);
    }

    static void addSingleArmorObservation(auto_graph::GraphOptimizer& optimizer,
                                          const TrackerFrameContext& context) {
        const auto& armor = context.armors_msg.armors.front();
        Eigen::MatrixXd R_mat;
        Eigen::Vector4d z_single = ypdObservation(
            armor, context.config.r_pose, context.config.r_distance,
            context.config.r_yaw, R_mat);
        Eigen::VectorXd z = z_single;
        // 兼容旧 graph_optimizer_test: 单板 fallback 虽然会更新 last index,
        // 但观测模型仍按 index 0 约束。
        optimizer.update<ArmorCVMeasureYPD>(z, R_mat);
    }
};

}  // namespace

std::unique_ptr<ObservationBackend> createObservationBackend(const TrackerConfig& config) {
    if (config.use_2d_observation) {
        return std::make_unique<PixelObservationBackend>();
    }
    return std::make_unique<YpdObservationBackend>();
}

}  // namespace filter_test::graph_optimizer

#pragma once

#include "filter_test/auto_graph_optimizer/utils/helpers.hpp"
#include "filter_test/graph_optimizer/tracker_types.hpp"

#include <auto_aim_interfaces/msg/armor.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

namespace filter_test::graph_optimizer {

inline constexpr double kRadiusMin = 0.10;
inline constexpr double kRadiusMax = 0.60;

inline double radiusFromState(double radius_u) {
    return auto_graph::logisticFunction(radius_u, kRadiusMin, kRadiusMax);
}

inline double radiusToState(double radius) {
    return auto_graph::logisticInverse(radius, kRadiusMin, kRadiusMax);
}

inline Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R) {
    double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
    double roll = std::atan2(R(2, 1), R(2, 2));
    double yaw = std::atan2(R(1, 0), R(0, 0));
    return {roll, pitch, yaw};
}

inline TrackerState stateFromVector(const Eigen::VectorXd& x) {
    TrackerState state;
    if (x.size() < 11) return state;
    state.center = {x[0], x[2], x[4]};
    state.velocity = {x[1], x[3], x[5]};
    state.yaw = x[6];
    state.vyaw = x[7];
    state.radius_1 = radiusFromState(x[8]);
    state.radius_2 = radiusFromState(x[9]);
    state.dz = x[10];
    return state;
}

inline std::vector<PredictedArmor> predictedArmorsFromState(const TrackerState& state) {
    std::vector<PredictedArmor> armors;
    armors.reserve(4);
    for (int i = 0; i < 4; ++i) {
        const double armor_yaw = state.yaw + i * M_PI_2;
        const double radius = (i % 2 == 0) ? state.radius_1 : state.radius_2;
        const double dz = (i % 2 == 0) ? 0.0 : state.dz;

        PredictedArmor armor;
        armor.index = i;
        armor.yaw = armor_yaw;
        armor.radius = radius;
        armor.dz = dz;
        armor.position.x() = state.center.x() - radius * std::cos(armor_yaw);
        armor.position.y() = state.center.y() - radius * std::sin(armor_yaw);
        armor.position.z() = state.center.z() + dz;
        armors.push_back(armor);
    }
    return armors;
}

inline int matchArmorIndex(const TrackerState& state,
                           const auto_aim_interfaces::msg::Armor& armor,
                           int last_armor_index) {
    double best_cost = std::numeric_limits<double>::max();
    int best_index = last_armor_index >= 0 ? last_armor_index : 0;

    for (int i = 0; i < 4; ++i) {
        const double pred_yaw = state.yaw + i * M_PI_2;
        const double yaw_diff =
            std::abs(auto_graph::shortestAngularDistance(pred_yaw, armor.yaw));

        const double radius = (i % 2 == 0) ? state.radius_1 : state.radius_2;
        const double dz = (i % 2 == 0) ? 0.0 : state.dz;
        const double pred_x = state.center.x() - std::cos(pred_yaw) * radius;
        const double pred_y = state.center.y() - std::sin(pred_yaw) * radius;
        const double pred_z = state.center.z() + dz;

        const double dx = armor.pose.position.x - pred_x;
        const double dy = armor.pose.position.y - pred_y;
        const double dz_err = armor.pose.position.z - pred_z;
        const double pos_err = std::sqrt(dx * dx + dy * dy + dz_err * dz_err);

        double cost = yaw_diff + 3.0 * pos_err;
        if (i == last_armor_index) cost *= 0.85;
        if (cost < best_cost) {
            best_cost = cost;
            best_index = i;
        }
    }

    return best_index;
}

inline std::vector<int> matchArmorIndicesUnique(
    const TrackerState& state,
    const auto_aim_interfaces::msg::Armors& msg,
    int last_armor_index) {
    std::vector<int> matched;
    matched.reserve(msg.armors.size());
    std::set<int> used_indices;

    for (const auto& armor : msg.armors) {
        double best_cost = std::numeric_limits<double>::max();
        int best_index = -1;

        for (int i = 0; i < 4; ++i) {
            if (used_indices.count(i) != 0) continue;

            const double pred_yaw = state.yaw + i * M_PI_2;
            const double yaw_diff =
                std::abs(auto_graph::shortestAngularDistance(pred_yaw, armor.yaw));

            const double radius = (i % 2 == 0) ? state.radius_1 : state.radius_2;
            const double dz = (i % 2 == 0) ? 0.0 : state.dz;
            const double pred_x = state.center.x() - std::cos(pred_yaw) * radius;
            const double pred_y = state.center.y() - std::sin(pred_yaw) * radius;
            const double pred_z = state.center.z() + dz;

            const double dx = armor.pose.position.x - pred_x;
            const double dy = armor.pose.position.y - pred_y;
            const double dz_err = armor.pose.position.z - pred_z;
            const double pos_err = std::sqrt(dx * dx + dy * dy + dz_err * dz_err);

            double cost = yaw_diff + 3.0 * pos_err;
            if (i == last_armor_index) cost *= 0.85;
            if (cost < best_cost) {
                best_cost = cost;
                best_index = i;
            }
        }

        // 车辆最多只有 4 个物理装甲板槽位。超出的检测保留为未匹配,
        // 避免像素后端把多块观测挂到同一个 Pose3 key 上。
        if (best_index >= 0) {
            used_indices.insert(best_index);
        }
        matched.push_back(best_index);
    }

    return matched;
}

}  // namespace filter_test::graph_optimizer

#pragma once

#include "filter_test/graph_optimizer/armor_model.hpp"

#include <auto_aim_interfaces/msg/armors.hpp>
#include <filter_test/msg/result.hpp>

#include <algorithm>
#include <limits>

namespace filter_test {

// filter_test::msg::Result 本身不携带普通/前哨站类型，因此用同步到的
// Armors 消息判断当前模型维度。
inline bool containsOutpostArmor(const auto_aim_interfaces::msg::Armors& armors) {
    return std::any_of(
        armors.armors.begin(), armors.armors.end(),
        [](const auto_aim_interfaces::msg::Armor& armor) {
            return armor.number == "outpost";
        });
}

inline graph_optimizer::TrackerState trackerStateFromFilterResult(
    const filter_test::msg::Result& result,
    const auto_aim_interfaces::msg::Armors& armors) {
    graph_optimizer::TrackerState state;
    state.center = Eigen::Vector3d(
        result.position.x, result.position.y, result.position.z);
    state.velocity = Eigen::Vector3d(
        result.velocity.x, result.velocity.y, result.velocity.z);
    state.yaw = result.yaw;
    state.vyaw = result.v_yaw;
    state.radius_1 = result.radius_1;
    state.radius_2 = result.radius_2;
    state.dz = result.dz;
    state.armor_count = containsOutpostArmor(armors) ? 3 : 4;

    if (state.armor_count == 3) {
        // 前哨站滤波器输出的 position 表示基准中心；图优化内部用
        // outpost_base_xy/z 保存这个静态基准，dz2 对应第三块装甲板高度差。
        state.outpost_dz_2 = result.dz2;
        state.outpost_base_xy = state.center.head<2>();
        state.outpost_base_z = state.center.z();
    } else {
        state.outpost_dz_2 = 0.0;
        state.outpost_base_xy = Eigen::Vector2d::Constant(
            std::numeric_limits<double>::quiet_NaN());
        state.outpost_base_z = std::numeric_limits<double>::quiet_NaN();
    }
    return state;
}

}  // namespace filter_test

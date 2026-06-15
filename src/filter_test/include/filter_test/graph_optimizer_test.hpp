#pragma once

#include "filter_test/graph_optimizer/armor_tracker.hpp"
#include "filter_test/visualization_marker_utils.hpp"

#include <auto_aim_interfaces/msg/enemy_info.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/tracker_target.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

namespace filter_test {

class GraphOptimizerTest : public rclcpp::Node {
public:
    explicit GraphOptimizerTest(const rclcpp::NodeOptions& options);

private:
    void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg);
    graph_optimizer::TrackerConfig loadTrackerConfig();
    bool lookupCameraToOdom(const rclcpp::Time& stamp, Eigen::Isometry3d& T_camera_to_odom);
    void logOutpostHeightDebug(
        const graph_optimizer::TrackerUpdateResult& result,
        const auto_aim_interfaces::msg::Armors& observed) const;
    void publishResult(const graph_optimizer::TrackerUpdateResult& result);
    void publishTrackerTarget(const graph_optimizer::TrackerUpdateResult& result);
    void publishMarkers(const graph_optimizer::TrackerUpdateResult& result,
                        const auto_aim_interfaces::msg::Armors& observed);

    std::unique_ptr<graph_optimizer::ArmorGraphTracker> tracker_;
    graph_optimizer::TrackerConfig tracker_config_;
    bool debug_outpost_height_ = false;

    graph_optimizer::FrameTimeTracker frame_time_;

    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr result_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::TrackerTarget>::SharedPtr tracker_target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

inline constexpr double kGraphOptimizerRadiusMin = graph_optimizer::kRadiusMin;
inline constexpr double kGraphOptimizerRadiusMax = graph_optimizer::kRadiusMax;

inline double graphOptimizerRadiusFromState(double radius_u) {
    return graph_optimizer::radiusFromState(radius_u);
}

inline double graphOptimizerRadiusToState(double radius) {
    return graph_optimizer::radiusToState(radius);
}

inline Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R) {
    return graph_optimizer::rotationMatrixToRPY(R);
}

inline void fillTrackerTargetEnemyFromState(
    const graph_optimizer::TrackerState& state,
    auto_aim_interfaces::msg::EnemyInfo& enemy) {
    enemy.tracking = true;
    enemy.position.x = state.center.x();
    enemy.position.y = state.center.y();
    enemy.position.z = state.center.z();
    enemy.velocity.x = state.velocity.x();
    enemy.velocity.y = state.velocity.y();
    enemy.velocity.z = state.velocity.z();
    enemy.orientation_yaw = state.yaw;
    enemy.v_yaw = state.vyaw;
    enemy.radius_1 = state.radius_1;
    if (state.armor_count == 3) {
        // 前哨站对齐 autoaim：radius_1/radius_2 为半径，dz/dz2 为两块装甲板高度偏移。
        enemy.radius_2 = state.radius_2;
        enemy.dz = state.dz;
        enemy.dz2 = state.outpost_dz_2;
    } else {
        enemy.radius_2 = state.radius_2;
        enemy.dz = state.dz;
        enemy.dz2 = 0.0;
    }
}

}  // namespace filter_test

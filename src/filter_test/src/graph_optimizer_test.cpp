#include "filter_test/graph_optimizer_test.hpp"
#include "filter_test/ros_utils/graph_optimizer_node_utils.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace filter_test {
GraphOptimizerTest::GraphOptimizerTest(const rclcpp::NodeOptions& options)
    : Node("graph_optimizer_test", options) {
    declareTrackerParameters(*this);
    tracker_config_ = loadTrackerConfig();
    debug_outpost_height_ = get_parameter("outpost.debug_height").as_bool();
    tracker_ = std::make_unique<graph_optimizer::ArmorGraphTracker>(tracker_config_);

    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    armors_sub_ = create_subscription<auto_aim_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
            armorsCallback(msg);
        });
    result_pub_ = create_publisher<auto_aim_interfaces::msg::Armors>(
        "/graph_optimizer/armors", rclcpp::SensorDataQoS());
    tracker_target_pub_ = create_publisher<auto_aim_interfaces::msg::TrackerTarget>(
        "/tracker/target", rclcpp::SensorDataQoS());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/graph_optimizer/marker", 10);

    RCLCPP_INFO(get_logger(), "Graph optimizer test initialized");
    RCLCPP_INFO(get_logger(), "  s2qxy: %.3f, s2qz: %.3f, s2qyaw: %.3f",
                tracker_config_.s2qxy, tracker_config_.s2qz, tracker_config_.s2qyaw);
}

graph_optimizer::TrackerConfig GraphOptimizerTest::loadTrackerConfig() {
    return loadTrackerConfigFromParameters(*this);
}

void GraphOptimizerTest::armorsCallback(
    const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
    if (msg->armors.empty()) return;

    const double dt = frame_time_.computeDt(msg->header.stamp);
    Eigen::Isometry3d T_camera_to_odom{Eigen::Isometry3d::Identity()};
    if (tracker_->initialized() &&
        !lookupCameraToOdom(msg->header.stamp, T_camera_to_odom)) {
        return;
    }

    graph_optimizer::TrackerFrameInput input{*msg, dt, T_camera_to_odom};
    auto result = tracker_->update(input);
    if (!result.reset_reason.empty()) {
        RCLCPP_WARN(get_logger(), "Graph optimizer reset: %s",
                    result.reset_reason.c_str());
    }
    if (result.accepted_frame) {
        frame_time_.commit(msg->header.stamp);
    }
    if (result.solve_failed) {
        RCLCPP_WARN(get_logger(), "Graph optimizer solve failed: %s",
                    result.solve_error.c_str());
        return;
    }
    if (!result.accepted_frame || !result.solved) return;

    logOutpostHeightDebug(result, *msg);
    publishResult(result);
    publishMarkers(result, *msg);
    publishTrackerTarget(result);

    static int frame_count = 0;
    frame_count++;
    if (frame_count % 10 == 0) {
        const auto& s = result.state;
        RCLCPP_INFO_STREAM(
            get_logger(),
            "Frame " << frame_count
                     << ": x=" << s.center.x()
                     << ", y=" << s.center.y()
                     << ", z=" << s.center.z()
                     << ", yaw=" << s.yaw
                     << ", r1=" << s.radius_1
                     << ", r2=" << s.radius_2
                     << ", dz1=" << s.dz
                     << ", dz2=" << s.outpost_dz_2);
    }
}

void GraphOptimizerTest::logOutpostHeightDebug(
    const graph_optimizer::TrackerUpdateResult& result,
    const auto_aim_interfaces::msg::Armors& observed) const {
    if (!debug_outpost_height_ || result.state.armor_count != 3) return;

    const auto& s = result.state;
    std::ostringstream line;
    line << "outpost_height frame=" << result.frame_id
         << " center_z=" << s.center.z()
         << " dz1=" << s.dz
         << " dz2=" << s.outpost_dz_2
         << " slots=[0:" << s.center.z()
         << ",1:" << s.center.z() + s.dz
         << ",2:" << s.center.z() + s.outpost_dz_2
         << "] obs={";

    bool first = true;
    for (std::size_t i = 0; i < observed.armors.size(); ++i) {
        const auto& armor = observed.armors[i];
        if (armor.number != "outpost") continue;

        const int matched_index =
            i < result.matched_indices.size() ? result.matched_indices[i] : -1;
        const double match_cost =
            i < result.match_costs.size() ? result.match_costs[i] :
            std::numeric_limits<double>::quiet_NaN();
        const double slot_z =
            matched_index == 0 ? s.center.z() :
            matched_index == 1 ? s.center.z() + s.dz :
            matched_index == 2 ? s.center.z() + s.outpost_dz_2 :
            std::numeric_limits<double>::quiet_NaN();
        const double z_residual =
            std::isfinite(slot_z) ? slot_z - armor.pose.position.z :
            std::numeric_limits<double>::quiet_NaN();

        if (!first) line << "; ";
        first = false;
        line << "#" << i
             << ":slot=" << matched_index
             << ",z=" << armor.pose.position.z
             << ",yaw=" << armor.yaw
             << ",cost=" << match_cost
             << ",slot_z=" << slot_z
             << ",z_res=" << z_residual;
    }
    line << "}";

    RCLCPP_INFO_STREAM(get_logger(), line.str());
}

bool GraphOptimizerTest::lookupCameraToOdom(
    const rclcpp::Time& stamp, Eigen::Isometry3d& T_camera_to_odom) {
    try {
        auto tf = tf2_buffer_->lookupTransform(
            "odom", "camera_optical_frame", stamp,
            rclcpp::Duration::from_seconds(0.1));
        Eigen::Quaterniond q(
            tf.transform.rotation.w, tf.transform.rotation.x,
            tf.transform.rotation.y, tf.transform.rotation.z);
        Eigen::Vector3d t(
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z);
        T_camera_to_odom = Eigen::Isometry3d::Identity();
        T_camera_to_odom.rotate(q.toRotationMatrix());
        T_camera_to_odom.pretranslate(t);
        return true;
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_SKIPFIRST(get_logger(), "TF lookup failed: %s", e.what());
        return false;
    }
}

void GraphOptimizerTest::publishResult(
    const graph_optimizer::TrackerUpdateResult& result) {
    auto_aim_interfaces::msg::Armors msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";

    auto_aim_interfaces::msg::Armor armor;
    armor.pose.position.x = result.state.center.x();
    armor.pose.position.y = result.state.center.y();
    armor.pose.position.z = result.state.center.z();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, result.state.yaw);
    armor.pose.orientation = tf2::toMsg(q);
    armor.yaw = result.state.yaw;
    armor.type = "small";
    armor.number = result.state.armor_count == 3 ? "outpost" : "4";

    msg.armors.push_back(armor);
    result_pub_->publish(msg);
}

void GraphOptimizerTest::publishTrackerTarget(
    const graph_optimizer::TrackerUpdateResult& result) {
    auto_aim_interfaces::msg::TrackerTarget target;
    target.header.stamp = now();
    target.header.frame_id = "odom";

    fillTrackerTargetEnemyFromState(result.state, target.enemy);

    target.armors_num = result.state.armor_count;
    tracker_target_pub_->publish(target);
}

void GraphOptimizerTest::publishMarkers(
    const graph_optimizer::TrackerUpdateResult& result,
    const auto_aim_interfaces::msg::Armors& observed) {
    auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
    const auto ts = now();

    visualization_msgs::msg::Marker pm;
    pm.header.stamp = ts;
    pm.header.frame_id = "odom";
    pm.ns = "position";
    pm.id = 0;
    pm.type = visualization_msgs::msg::Marker::SPHERE;
    pm.action = visualization_msgs::msg::Marker::ADD;
    pm.lifetime = rclcpp::Duration::from_seconds(0.1);
    pm.pose.position.x = result.state.center.x();
    pm.pose.position.y = result.state.center.y();
    pm.pose.position.z = result.state.armor_count == 3
        ? result.state.center.z()
        : result.state.center.z() + result.state.dz / 2.0;
    pm.scale.x = pm.scale.y = pm.scale.z = 0.1;
    pm.color.a = 1.0;
    pm.color.g = 1.0;
    markers->markers.push_back(pm);

    for (auto* ns : {"armors_pred", "armors_obs"}) {
        for (int i = 0; i < 5; ++i) {
            visualization_msgs::msg::Marker del;
            del.header.stamp = ts;
            del.header.frame_id = "odom";
            del.ns = ns;
            del.id = i;
            del.action = visualization_msgs::msg::Marker::DELETE;
            markers->markers.push_back(del);
        }
    }

    for (const auto& pred : result.predicted_armors) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = ts;
        marker.header.frame_id = "odom";
        marker.ns = "armors_pred";
        marker.id = pred.index;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker.pose.position.x = pred.position.x();
        marker.pose.position.y = pred.position.y();
        marker.pose.position.z = pred.position.z();
        tf2::Quaternion q;
        const double pitch = result.state.armor_count == 3
            ? tracker_config_.outpost_armor_pitch
            : tracker_config_.standard_armor_pitch;
        q.setRPY(0.0, pitch, pred.yaw);
        marker.pose.orientation = tf2::toMsg(q);
        marker.scale.x = 0.005;
        marker.scale.y = 0.135;
        marker.scale.z = 0.125;
        marker.color.a = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        markers->markers.push_back(marker);
    }

    for (std::size_t i = 0; i < observed.armors.size(); ++i) {
        const auto& obs = observed.armors[i];
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = ts;
        marker.header.frame_id = "odom";
        marker.ns = "armors_obs";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker.pose.position = obs.pose.position;
        marker.pose.orientation = tf2::toMsg(
            observedArmorMarkerQuaternion(obs.yaw, obs.number == "outpost"));
        marker.scale.x = 0.005;
        marker.scale.y = 0.135;
        marker.scale.z = 0.125;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        markers->markers.push_back(marker);
    }

    marker_pub_->publish(std::move(markers));
}

}  // namespace filter_test

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<filter_test::GraphOptimizerTest>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

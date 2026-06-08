#include "filter_test/graph_optimizer_test.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>

namespace filter_test {
namespace {

void declareTrackerParameters(rclcpp::Node& node) {
    node.declare_parameter("use_2d_observation", false);
    node.declare_parameter("s2qxy", 0.1);
    node.declare_parameter("s2qz", 0.1);
    node.declare_parameter("s2qyaw", 0.1);
    node.declare_parameter("vel_sigma", 0.01);
    node.declare_parameter("vyaw_sigma", 0.05);
    node.declare_parameter("match_max_cost", graph_optimizer::kDefaultArmorMatchMaxCost);
    node.declare_parameter("pixel_noise_std", 2.0);
    node.declare_parameter("geo_tangential", 0.01);
    node.declare_parameter("geo_radial", 0.03);
    node.declare_parameter("geo_height", 0.01);
    node.declare_parameter("geo_yaw", 0.005);

    node.declare_parameter("camera_fx", 2411.0);
    node.declare_parameter("camera_fy", 2411.0);
    node.declare_parameter("camera_cx", 720.0);
    node.declare_parameter("camera_cy", 640.0);
    node.declare_parameter("distortion_k1", -0.093);
    node.declare_parameter("distortion_k2", 0.154);
    node.declare_parameter("distortion_p1", 0.0001);
    node.declare_parameter("distortion_p2", -0.0006);

    node.declare_parameter("verbose", true);
    node.declare_parameter("cold_start_frames", 3);
    node.declare_parameter("smoother_lag", 0.0);
    node.declare_parameter("smoother_type", "incremental");
    node.declare_parameter("relinearize_threshold", 0.001);
    node.declare_parameter("extra_iterations", 2);
}

}  // namespace

GraphOptimizerTest::GraphOptimizerTest(const rclcpp::NodeOptions& options)
    : Node("graph_optimizer_test", options) {
    declareTrackerParameters(*this);
    tracker_config_ = loadTrackerConfig();
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
    const bool legacy_use_2d_observation =
        get_parameter("use_2d_observation").as_bool();
    RCLCPP_INFO(get_logger(), "  typed pixel graph enabled; use_2d_observation parameter kept for compatibility: %s",
                legacy_use_2d_observation ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  s2qxy: %.3f, s2qz: %.3f, s2qyaw: %.3f",
                tracker_config_.s2qxy, tracker_config_.s2qz, tracker_config_.s2qyaw);
}

graph_optimizer::TrackerConfig GraphOptimizerTest::loadTrackerConfig() {
    graph_optimizer::TrackerConfig config;
    config.s2qxy = get_parameter("s2qxy").as_double();
    config.s2qz = get_parameter("s2qz").as_double();
    config.s2qyaw = get_parameter("s2qyaw").as_double();
    config.vel_sigma = get_parameter("vel_sigma").as_double();
    config.vyaw_sigma = get_parameter("vyaw_sigma").as_double();
    config.match_max_cost = get_parameter("match_max_cost").as_double();
    config.pixel_sigma = get_parameter("pixel_noise_std").as_double();
    config.geo_noise.tangential = get_parameter("geo_tangential").as_double();
    config.geo_noise.radial = get_parameter("geo_radial").as_double();
    config.geo_noise.height = get_parameter("geo_height").as_double();
    config.geo_noise.yaw = get_parameter("geo_yaw").as_double();

    const double fx = get_parameter("camera_fx").as_double();
    const double fy = get_parameter("camera_fy").as_double();
    const double cx = get_parameter("camera_cx").as_double();
    const double cy = get_parameter("camera_cy").as_double();
    config.camera_matrix << fx, 0.0, cx,
                            0.0, fy, cy,
                            0.0, 0.0, 1.0;
    config.distortion[0] = get_parameter("distortion_k1").as_double();
    config.distortion[1] = get_parameter("distortion_k2").as_double();
    config.distortion[2] = get_parameter("distortion_p1").as_double();
    config.distortion[3] = get_parameter("distortion_p2").as_double();
    config.distortion[4] = 0.0;

    config.optimizer.cold_start_frames = get_parameter("cold_start_frames").as_int();
    config.optimizer.verbose = get_parameter("verbose").as_bool();
    config.optimizer.smoother_lag = get_parameter("smoother_lag").as_double();
    const std::string smoother_type = get_parameter("smoother_type").as_string();
    config.optimizer.smoother_type = smoother_type == "batch"
        ? auto_graph::SmootherType::Batch
        : auto_graph::SmootherType::Incremental;
    config.optimizer.relinearize_threshold =
        get_parameter("relinearize_threshold").as_double();
    // Keep the ROS parameter name for config compatibility; internally this is
    // the total number of update calls per solve, including the first update.
    config.optimizer.update_iterations = get_parameter("extra_iterations").as_int();
    return config;
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
    if (result.accepted_frame) {
        frame_time_.commit(msg->header.stamp);
    }
    if (result.solve_failed) {
        RCLCPP_WARN(get_logger(), "Graph optimizer solve failed: %s",
                    result.solve_error.c_str());
        return;
    }
    if (!result.accepted_frame || !result.solved) return;

    publishResult(result);
    publishMarkers(result, *msg);
    publishTrackerTarget(result);

    static int frame_count = 0;
    frame_count++;
    if (frame_count % 10 == 0) {
        const auto& s = result.state;
        RCLCPP_INFO(get_logger(),
                    "Frame %d: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f, r1=%.3f, r2=%.3f, dz=%.3f",
                    frame_count, s.center.x(), s.center.y(), s.center.z(), s.yaw,
                    s.radius_1, s.radius_2, s.dz);
    }
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
    armor.number = "4";

    msg.armors.push_back(armor);
    result_pub_->publish(msg);
}

void GraphOptimizerTest::publishTrackerTarget(
    const graph_optimizer::TrackerUpdateResult& result) {
    auto_aim_interfaces::msg::TrackerTarget target;
    target.header.stamp = now();
    target.header.frame_id = "odom";

    target.enemy.tracking = true;
    target.enemy.position.x = result.state.center.x();
    target.enemy.position.y = result.state.center.y();
    target.enemy.position.z = result.state.center.z();
    target.enemy.velocity.x = result.state.velocity.x();
    target.enemy.velocity.y = result.state.velocity.y();
    target.enemy.velocity.z = result.state.velocity.z();
    target.enemy.orientation_yaw = result.state.yaw;
    target.enemy.v_yaw = result.state.vyaw;
    target.enemy.radius_1 = result.state.radius_1;
    target.enemy.radius_2 = result.state.radius_2;
    target.enemy.dz = result.state.dz;

    target.armors_num = 4;
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
    pm.pose.position.z = result.state.center.z() + result.state.dz / 2.0;
    pm.scale.x = pm.scale.y = pm.scale.z = 0.1;
    pm.color.a = 1.0;
    pm.color.g = 1.0;
    markers->markers.push_back(pm);

    for (auto* ns : {"armors_pred", "armors_obs"}) {
        for (int i = 0; i < 4; ++i) {
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
        q.setRPY(0.0, 15.0 * M_PI / 180.0, pred.yaw);
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
        marker.pose.orientation = tf2::toMsg(observedArmorMarkerQuaternion(obs.yaw));
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

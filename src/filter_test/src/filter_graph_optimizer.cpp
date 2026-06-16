#include "filter_test/ros_utils/filter_frontend_adapter.hpp"
#include "filter_test/graph_optimizer/armor_tracker.hpp"
#include "filter_test/ros_utils/graph_optimizer_node_utils.hpp"
#include "filter_test/graph_optimizer_test.hpp"
#include "filter_test/visualization_marker_utils.hpp"

#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/tracker_target.hpp>
#include <filter_test/msg/result.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>

namespace filter_test {
namespace {

using ArmorsMsg = auto_aim_interfaces::msg::Armors;
using ResultMsg = filter_test::msg::Result;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    ArmorsMsg, ResultMsg>;

}  // namespace

// 该节点把传统滤波器作为前端：滤波器继续处理原始装甲板并发布 /track_result，
// 本节点只把同一时刻的 /detector/armors + /track_result 送入图优化后端。
class FilterGraphOptimizer : public rclcpp::Node {
public:
    explicit FilterGraphOptimizer(const rclcpp::NodeOptions& options)
        : Node("filter_graph_optimizer", options),
          tf2_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
          tf2_listener_(std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_)) {
        declareTrackerParameters(*this);
        declare_parameter("frontend.sync_queue_size", 10);
        declare_parameter("frontend.sync_slop", 0.02);

        tracker_config_ = loadTrackerConfigFromParameters(*this);
        tracker_ = std::make_unique<graph_optimizer::ArmorGraphTracker>(
            tracker_config_);
        debug_outpost_height_ = get_parameter("outpost.debug_height").as_bool();

        armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
        result_sub_.subscribe(this, "/track_result", rmw_qos_profile_sensor_data);

        const int queue_size = static_cast<int>(
            std::max<int64_t>(
                1, get_parameter("frontend.sync_queue_size").as_int()));
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(static_cast<uint32_t>(queue_size)),
            armors_sub_, result_sub_);
        // 只接受接近同一帧的前端输出，避免滤波结果跨帧后反而把图初值拉偏。
        sync_->setMaxIntervalDuration(
            rclcpp::Duration::from_seconds(
                get_parameter("frontend.sync_slop").as_double()));
        sync_->registerCallback(
            std::bind(
                &FilterGraphOptimizer::syncedCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        result_pub_ = create_publisher<auto_aim_interfaces::msg::Armors>(
            "/graph_optimizer/armors", rclcpp::SensorDataQoS());
        tracker_target_pub_ =
            create_publisher<auto_aim_interfaces::msg::TrackerTarget>(
                "/tracker/target", rclcpp::SensorDataQoS());
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/graph_optimizer/marker", 10);

        RCLCPP_INFO(get_logger(), "Filter frontend + graph optimizer initialized");
    }

private:
    void syncedCallback(
        const ArmorsMsg::ConstSharedPtr& armors,
        const ResultMsg::ConstSharedPtr& filter_result) {
        if (armors->armors.empty()) return;

        const double dt = frame_time_.computeDt(armors->header.stamp);
        Eigen::Isometry3d T_camera_to_odom{Eigen::Isometry3d::Identity()};
        if (tracker_->initialized() &&
            !lookupCameraToOdom(armors->header.stamp, T_camera_to_odom)) {
            return;
        }

        const auto frontend_state =
            trackerStateFromFilterResult(*filter_result, *armors);
        // frontend_state 提供当前帧初值和可选弱 prior；armors 中的像素/PnP
        // 观测仍由图优化照常建因子，后端不会退化成简单转发滤波器结果。
        graph_optimizer::TrackerFrameInput input{
            *armors, dt, T_camera_to_odom, frontend_state};
        auto result = tracker_->update(input);
        if (!result.reset_reason.empty()) {
            RCLCPP_WARN(
                get_logger(), "Filter graph optimizer reset: %s",
                result.reset_reason.c_str());
        }
        if (result.accepted_frame) {
            frame_time_.commit(armors->header.stamp);
        }
        if (result.solve_failed) {
            RCLCPP_WARN(
                get_logger(), "Filter graph optimizer solve failed: %s",
                result.solve_error.c_str());
            return;
        }
        if (!result.accepted_frame || !result.solved) return;

        logOutpostHeightDebug(result, *armors);
        publishResult(result);
        publishMarkers(result, *armors);
        publishTrackerTarget(result);
    }

    bool lookupCameraToOdom(
        const rclcpp::Time& stamp,
        Eigen::Isometry3d& T_camera_to_odom) {
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

    void logOutpostHeightDebug(
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
                 << ",slot_z=" << slot_z
                 << ",z_res=" << z_residual;
        }
        line << "}";

        RCLCPP_INFO_STREAM(get_logger(), line.str());
    }

    void publishResult(const graph_optimizer::TrackerUpdateResult& result) {
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

    void publishTrackerTarget(
        const graph_optimizer::TrackerUpdateResult& result) {
        auto_aim_interfaces::msg::TrackerTarget target;
        target.header.stamp = now();
        target.header.frame_id = "odom";

        fillTrackerTargetEnemyFromState(result.state, target.enemy);

        target.armors_num = result.state.armor_count;
        tracker_target_pub_->publish(target);
    }

    void publishMarkers(
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

    graph_optimizer::TrackerConfig tracker_config_;
    std::unique_ptr<graph_optimizer::ArmorGraphTracker> tracker_;
    bool debug_outpost_height_ = false;
    graph_optimizer::FrameTimeTracker frame_time_;

    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<ArmorsMsg> armors_sub_;
    message_filters::Subscriber<ResultMsg> result_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr result_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::TrackerTarget>::SharedPtr
        tracker_target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace filter_test

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<filter_test::FilterGraphOptimizer>(
        rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

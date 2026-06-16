// jlu tracker adapter — converts ROS2 Armor messages to jlu::RobotTarget
#include "filter_test/jlu_tracker/target.hpp"
#include "filter_test/jlu_tracker/configs.hpp"
#include "filter_test/ros_utils/camera_info_utils.hpp"
#include "filter_test/visualization_marker_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>
#include <auto_aim_interfaces/msg/tracker_target.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <memory>
#include <vector>
#include <string>

namespace filter_test {

class JluTrackerNode : public rclcpp::Node {
 public:
  JluTrackerNode(const rclcpp::NodeOptions& options)
      : Node("jlu_tracker", options) {
    // Config
    jlu::RobotConfig config;
    config.first_update_batch_size =
        declare_parameter("cold_start_frames", 5);
    config_.cold_start_frames = config.first_update_batch_size;

    // Camera
    // jlu::RobotTarget 接口沿用 cv::Mat K/D；读取仍统一走 CameraInfo，
    // 不再维护一套独立的手写 fx/fy/cx/cy 参数。
    auto calibration = loadCameraCalibrationFromInfo(*this);
    camera_matrix_ = calibration.cv_camera_matrix;
    distortion_ = calibration.cv_distortion;

    // TF2
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    // Create tracker
    target_ = std::make_unique<jlu::RobotTarget>(
        config, jlu::ArmorType::Small, camera_matrix_, distortion_);

    // Sub
    armors_sub_ = create_subscription<auto_aim_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS(),
        [this](auto_aim_interfaces::msg::Armors::SharedPtr msg) { armorsCallback(msg); });

    // Pub
    result_pub_ = create_publisher<auto_aim_interfaces::msg::Armors>(
        "/graph_optimizer/armors", rclcpp::SensorDataQoS());
    tracker_target_pub_ = create_publisher<auto_aim_interfaces::msg::TrackerTarget>(
        "/tracker/target", rclcpp::SensorDataQoS());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/graph_optimizer/marker", 10);

    RCLCPP_INFO(get_logger(), "jlu tracker adapter initialized, cold_start=%d",
                config_.cold_start_frames);
  }

 private:
  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
    if (msg->armors.empty()) return;

    // Convert ROS2 Armor → jlu::ArmorPositionRollPitchYawPoints
    std::vector<jlu::ArmorPositionRollPitchYawPoints> armors;
    for (const auto& a : msg->armors) {
      armors.emplace_back(a);
    }

    // TF lookup
    Eigen::Isometry3d T_camera_to_odom{Eigen::Isometry3d::Identity()};
    bool tf_ok = false;
    try {
      auto tf = tf2_buffer_->lookupTransform(
          "odom", "camera_optical_frame", msg->header.stamp,
          rclcpp::Duration::from_seconds(0.1));
      Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                           tf.transform.rotation.y, tf.transform.rotation.z);
      Eigen::Matrix3d R = q.toRotationMatrix();
      Eigen::Vector3d t(tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z);
      T_camera_to_odom.rotate(R);
      T_camera_to_odom.pretranslate(t);
      tf_ok = true;
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_SKIPFIRST(get_logger(), "TF fail: %s", e.what());
    }
    if (!tf_ok) return;

    // Convert armor position + orientation from odom to camera
    // (jlu tracker expects camera frame; simulation publishes odom frame)
    auto T_odom_to_camera = T_camera_to_odom.inverse();
    Eigen::Matrix3d R_o2c = T_odom_to_camera.rotation();
    for (auto& a : armors) {
      a.position = T_odom_to_camera * a.position;
      // Also convert orientation: a.getRotation() is odom-frame → camera-frame RPY
      Eigen::Matrix3d R_cam = R_o2c * a.getRotation().toRotationMatrix();
      a.pitch = std::asin(std::max(-1.0, std::min(1.0, -R_cam(2, 0))));
      a.roll  = std::atan2(R_cam(2, 1), R_cam(2, 2));
      a.yaw   = gtsam::Rot2::fromAngle(std::atan2(R_cam(1, 0), R_cam(0, 0)));
    }

    // Track
    double stamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    auto state = target_->track(armors, stamp_sec, T_camera_to_odom);

    // Diagnostic
    static int frame_cnt = 0;
    frame_cnt++;
    if (frame_cnt % 100 == 1) {
      auto [ts, trk] = target_->getTargetTrackState();
      RCLCPP_INFO(get_logger(), "f=%d x=%.2f y=%.2f z=%.2f yaw=%.2f ra=%.3f rb=%.3f dz=%.3f k=%lu s=%d",
                  frame_cnt, ts.center_position.x(), ts.center_position.y(),
                  ts.center_position.z(), ts.center_yaw,
                  ts.radius_a, ts.radius_b, ts.dz, trk.k, (int)trk.state);
    }

    // Save observed armors for markers
    last_armors_ = msg;

    // Publish result
    publishResult();
    publishMarkers();
  }

  void publishResult() {
    auto [target_state, track_state] = target_->getTargetTrackState();
    if (track_state.state == jlu::TrackState::State::LOST) return;

    auto result = std::make_unique<auto_aim_interfaces::msg::Armors>();
    result->header.stamp = now();
    result->header.frame_id = "odom";

    // Publish 4 armor positions from target state
    for (int i = 0; i < 4; i++) {
      auto idx = static_cast<jlu::ArmorIndex>(i);
      auto [r, dz] = (idx == jlu::ArmorIndex::_0 || idx == jlu::ArmorIndex::_2)
                         ? std::make_pair(target_state.radius_a, 0.0)
                         : std::make_pair(target_state.radius_b, target_state.dz);
      jlu::ArmorPositionYaw ap(target_state.center_position,
                                target_state.center_yaw, r, dz, 4, idx);
      auto_aim_interfaces::msg::Armor armor;
      armor.pose.position.x = ap.position.x();
      armor.pose.position.y = ap.position.y();
      armor.pose.position.z = ap.position.z();
      armor.yaw = ap.yaw.theta();
      result->armors.push_back(armor);
    }
    result_pub_->publish(std::move(result));
  }

  void publishMarkers() {
    auto [target_state, track_state] = target_->getTargetTrackState();
    if (track_state.state == jlu::TrackState::State::LOST) return;

    auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
    auto ts = now();
    double xc = target_state.center_position.x();
    double yc = target_state.center_position.y();
    double za = target_state.center_position.z();
    double yaw = target_state.center_yaw;
    double r1 = target_state.radius_a, r2 = target_state.radius_b;
    double dz = target_state.dz;

    // ── 位置标记 (绿色球) ──
    visualization_msgs::msg::Marker pm;
    pm.header.stamp = ts; pm.header.frame_id = "odom";
    pm.ns = "position"; pm.id = 0;
    pm.type = visualization_msgs::msg::Marker::SPHERE;
    pm.action = visualization_msgs::msg::Marker::ADD;
    pm.lifetime = rclcpp::Duration::from_seconds(0.1);
    pm.pose.position.x = xc; pm.pose.position.y = yc;
    pm.pose.position.z = za + dz / 2;
    pm.scale.x = pm.scale.y = pm.scale.z = 0.1;
    pm.color.a = 1.0; pm.color.r = 0.0; pm.color.g = 1.0; pm.color.b = 0.0;
    markers->markers.push_back(pm);

    // ── DELETE 旧 marker (armors_pred + armors_obs) ──
    for (auto* ns : {"armors_pred", "armors_obs"}) {
      for (int i = 0; i < 4; i++) {
        visualization_msgs::msg::Marker del;
        del.header.stamp = ts; del.header.frame_id = "odom";
        del.ns = ns; del.id = i;
        del.action = visualization_msgs::msg::Marker::DELETE;
        markers->markers.push_back(del);
      }
    }

    // ── 追踪器预测的 4 块装甲板 (蓝色) ──
    for (int i = 0; i < 4; i++) {
      double ay = yaw + i * M_PI_2;
      double r = (i % 2 == 0) ? r1 : r2;
      double dzi = (i % 2 == 0) ? 0.0 : dz;

      visualization_msgs::msg::Marker am;
      am.header.stamp = ts; am.header.frame_id = "odom";
      am.ns = "armors_pred"; am.id = i;
      am.type = visualization_msgs::msg::Marker::CUBE;
      am.action = visualization_msgs::msg::Marker::ADD;
      am.lifetime = rclcpp::Duration::from_seconds(0.1);
      am.pose.position.x = xc - r * std::cos(ay);
      am.pose.position.y = yc - r * std::sin(ay);
      am.pose.position.z = za + dzi;
      tf2::Quaternion q; q.setRPY(0, 15.0 * M_PI / 180.0, ay);
      am.pose.orientation = tf2::toMsg(q);
      am.scale.x = 0.005; am.scale.y = 0.135; am.scale.z = 0.125;
      am.color.a = 1.0; am.color.r = 0.0; am.color.g = 0.5; am.color.b = 1.0;
      markers->markers.push_back(am);
    }

    // ── 检测器观测装甲板 (红色) ──
    if (last_armors_) {
      for (size_t i = 0; i < last_armors_->armors.size(); i++) {
        const auto& obs = last_armors_->armors[i];
        visualization_msgs::msg::Marker am;
        am.header.stamp = ts; am.header.frame_id = "odom";
        am.ns = "armors_obs"; am.id = static_cast<int>(i);
        am.type = visualization_msgs::msg::Marker::CUBE;
        am.action = visualization_msgs::msg::Marker::ADD;
        am.lifetime = rclcpp::Duration::from_seconds(0.1);
        am.pose.position.x = obs.pose.position.x;
        am.pose.position.y = obs.pose.position.y;
        am.pose.position.z = obs.pose.position.z;
        am.pose.orientation = tf2::toMsg(
            observedArmorMarkerQuaternion(obs.yaw, obs.number == "outpost"));
        am.scale.x = 0.005; am.scale.y = 0.135; am.scale.z = 0.125;
        am.color.a = 1.0; am.color.r = 1.0; am.color.g = 0.0; am.color.b = 0.0;
        markers->markers.push_back(am);
      }
    }

    marker_pub_->publish(std::move(markers));
  }

  struct Config { int cold_start_frames = 3; } config_;
  cv::Mat camera_matrix_, distortion_;
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::unique_ptr<jlu::RobotTarget> target_;
  auto_aim_interfaces::msg::Armors::SharedPtr last_armors_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr result_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerTarget>::SharedPtr tracker_target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace filter_test

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<filter_test::JluTrackerNode>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

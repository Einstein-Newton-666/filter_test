#include "filter_test/graph_optimizer/armor_tracker.hpp"
#include "filter_test/graph_optimizer/rune_model.hpp"
#include "filter_test/ros_utils/camera_info_utils.hpp"

#include <auto_aim_interfaces/msg/rune_info.hpp>
#include <auto_aim_interfaces/msg/rune_targets.hpp>
#include <auto_aim_interfaces/msg/target_pose.hpp>
#include <auto_aim_interfaces/msg/tracker_target.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace filter_test {
namespace {
constexpr double kRuneBladeMarkerThinAxis = 0.01;
constexpr double kRuneBladeMarkerWideAxis = 0.15;


void setPoint(geometry_msgs::msg::Point& msg, const Eigen::Vector3d& point) {
    msg.x = point.x();
    msg.y = point.y();
    msg.z = point.z();
}

auto_graph::SmootherType parseSmootherType(const std::string& value) {
    return value == "batch" ? auto_graph::SmootherType::Batch
                            : auto_graph::SmootherType::Incremental;
}

geometry_msgs::msg::Quaternion planeOrientation(double normal_yaw, double normal_pitch) {
    tf2::Quaternion q;
    q.setRPY(0.0, normal_pitch, normal_yaw);
    return tf2::toMsg(q);
}

geometry_msgs::msg::Quaternion eigenQuaternionToMsg(const Eigen::Quaterniond& q) {
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}

}  // namespace

class RuneGraphOptimizerNode : public rclcpp::Node {
public:
    explicit RuneGraphOptimizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("rune_graph_optimizer", options),
          graph_(loadConfig()) {
        tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

        const auto targets_topic =
            declare_parameter<std::string>("targets_topic", "/rune_detector/rune_targets");
        targets_sub_ = create_subscription<auto_aim_interfaces::msg::RuneTargets>(
            targets_topic, rclcpp::SensorDataQoS(),
            [this](const auto_aim_interfaces::msg::RuneTargets::SharedPtr msg) {
                targetsCallback(msg);
            });
        info_pub_ = create_publisher<auto_aim_interfaces::msg::RuneInfo>(
            "/rune_graph_optimizer/rune_info", 10);
        legacy_info_pub_ = create_publisher<auto_aim_interfaces::msg::RuneInfo>(
            "/rune_graph_optimizer/info", 10);
        target_pose_pub_ = create_publisher<auto_aim_interfaces::msg::TargetPose>(
            "/rune_graph_optimizer/target_pose", rclcpp::SensorDataQoS());
        tracker_target_pub_ = create_publisher<auto_aim_interfaces::msg::TrackerTarget>(
            "/rune/tracker_target", rclcpp::SensorDataQoS());
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/rune_graph_optimizer/marker", 10);

        RCLCPP_INFO(get_logger(), "Rune graph optimizer initialized");
    }

private:
    struct RunePublishPrediction {
        double optimized_roll = 0.0;
        double predicted_roll = 0.0;
        double angular_velocity = 0.0;
        Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
        std::array<double, 5> curve_params{0.0, 0.0, 0.0, 0.0, 0.0};
    };

    graph_optimizer::RuneTrackerConfig loadConfig() {
        graph_optimizer::RuneTrackerConfig config;
        // 参数分三类：
        // 1. 几何先验 initial_center/normal_yaw/normal_pitch；
        // 2. 图优化噪声 pixel/center/roll/vroll/pose_prior；
        // 3. GraphOptimizer 的 smoother/iSAM2 生命周期参数。
        declare_parameter("initial_center.x", config.initial_center.x());
        declare_parameter("initial_center.y", config.initial_center.y());
        declare_parameter("initial_center.z", config.initial_center.z());
        declare_parameter("normal_yaw", config.normal_yaw);
        declare_parameter("normal_pitch", config.normal_pitch);
        declare_parameter("pixel_noise_std", config.pixel_sigma);
        declare_parameter("center_sigma", config.center_sigma);
        declare_parameter("roll_sigma", config.roll_sigma);
        declare_parameter("vroll_sigma", config.vroll_sigma);
        declare_parameter("pose_prior_sigma", config.pose_prior_sigma);
        declare_parameter("pnp_pose_prior_sigma", config.pnp_pose_prior_sigma);
        declare_parameter("use_direct_reproj_factor", config.use_direct_reproj_factor);
        declare_parameter("direct_reproj_sigma", config.direct_reproj_sigma);
        declare_parameter("normal_yaw_sigma", config.normal_yaw_sigma);
        declare_parameter("match_max_roll_diff", config.match_max_roll_diff);
        declare_parameter("match_max_center_distance", config.match_max_center_distance);
        declare_parameter("use_direct_reproj_match_fallback", config.use_direct_reproj_match_fallback);
        declare_parameter("match_max_direct_reproj_error", config.match_max_direct_reproj_error);
        declare_parameter("predict_dt", config.predict_dt);
        declare_parameter("big_fitter.min_data_size", 100);
        declare_parameter("big_fitter.max_data_size", 600);
        declare_parameter("big_fitter.max_iterations", 30);
        declare_parameter("camera_name", "narrow_stereo");
        declare_parameter(
            "camera_info_url",
            "package://armor_simulation/config/camera_info.yaml");
        declare_parameter("verbose", false);
        declare_parameter("cold_start_frames", 3);
        declare_parameter("smoother_lag", 10.0);
        declare_parameter("smoother_type", std::string("incremental"));
        declare_parameter("relinearize_threshold", 0.001);
        declare_parameter("extra_iterations", 2);

        config.initial_center.x() = get_parameter("initial_center.x").as_double();
        config.initial_center.y() = get_parameter("initial_center.y").as_double();
        config.initial_center.z() = get_parameter("initial_center.z").as_double();
        config.normal_yaw = get_parameter("normal_yaw").as_double();
        config.normal_pitch = get_parameter("normal_pitch").as_double();
        normal_pitch_ = config.normal_pitch;
        config.pixel_sigma = get_parameter("pixel_noise_std").as_double();
        config.center_sigma = get_parameter("center_sigma").as_double();
        config.roll_sigma = get_parameter("roll_sigma").as_double();
        config.vroll_sigma = get_parameter("vroll_sigma").as_double();
        config.pose_prior_sigma = get_parameter("pose_prior_sigma").as_double();
        config.pnp_pose_prior_sigma = get_parameter("pnp_pose_prior_sigma").as_double();
        config.use_direct_reproj_factor = get_parameter("use_direct_reproj_factor").as_bool();
        config.direct_reproj_sigma = get_parameter("direct_reproj_sigma").as_double();
        config.normal_yaw_sigma = get_parameter("normal_yaw_sigma").as_double();
        config.match_max_roll_diff = get_parameter("match_max_roll_diff").as_double();
        config.match_max_center_distance = get_parameter("match_max_center_distance").as_double();
        config.use_direct_reproj_match_fallback =
            get_parameter("use_direct_reproj_match_fallback").as_bool();
        config.match_max_direct_reproj_error =
            get_parameter("match_max_direct_reproj_error").as_double();
        config.predict_dt = get_parameter("predict_dt").as_double();

        // Rune 图优化的 PnP prior、五点像素重投影和 direct roll loss
        // 必须与 rune_simulation_node 使用同一 CameraInfo。
        const auto calibration = loadCameraCalibrationFromInfo(*this);
        config.camera_matrix = calibration.camera_matrix;
        config.distortion = calibration.distortion;

        config.optimizer.verbose = get_parameter("verbose").as_bool();
        config.optimizer.cold_start_frames = get_parameter("cold_start_frames").as_int();
        config.optimizer.smoother_lag = get_parameter("smoother_lag").as_double();
        config.optimizer.smoother_type =
            parseSmootherType(get_parameter("smoother_type").as_string());
        config.optimizer.relinearize_threshold =
            get_parameter("relinearize_threshold").as_double();
        config.optimizer.update_iterations = get_parameter("extra_iterations").as_int();
        predict_dt_ = config.predict_dt;
        graph_optimizer::RuneBigCurveFitterConfig fitter_config;
        fitter_config.min_data_size = get_parameter("big_fitter.min_data_size").as_int();
        fitter_config.max_data_size = get_parameter("big_fitter.max_data_size").as_int();
        fitter_config.max_iterations = get_parameter("big_fitter.max_iterations").as_int();
        curve_fitter_ = graph_optimizer::RuneBigCurveFitter(fitter_config);
        return config;
    }

    void targetsCallback(const auto_aim_interfaces::msg::RuneTargets::SharedPtr msg) {
        Eigen::Isometry3d T_camera_to_odom = Eigen::Isometry3d::Identity();
        // RuneTargets 只有像素观测，图优化需要相机位姿把辅助 Pose3
        // 从 camera 系连接回 odom 下的 center/roll 状态。
        if (!lookupCameraToOdom(msg->header.stamp, T_camera_to_odom)) {
            return;
        }

        const double dt = frame_time_.computeDt(msg->header.stamp);
        auto output = graph_.update(*msg, dt, T_camera_to_odom);
        frame_time_.commit(msg->header.stamp);

        if (output.solve_result.failed) {
            RCLCPP_WARN(get_logger(), "Rune graph solve failed: %s",
                        output.solve_result.error_message.c_str());
            return;
        }

        const auto prediction = makePublishPrediction(*msg, output);
        publishInfo(*msg, output, prediction);
        publishTargetPose(*msg, output, prediction);
        publishTrackerTarget(*msg, output, prediction);
        publishMarkers(*msg, output, prediction);
    }

    bool lookupCameraToOdom(
        const builtin_interfaces::msg::Time& stamp,
        Eigen::Isometry3d& T_camera_to_odom) {
        try {
            auto tf = tf2_buffer_->lookupTransform(
                "odom", "camera_optical_frame", rclcpp::Time(stamp),
                rclcpp::Duration::from_seconds(0.1));
            Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                                 tf.transform.rotation.y, tf.transform.rotation.z);
            Eigen::Vector3d t(tf.transform.translation.x,
                              tf.transform.translation.y,
                              tf.transform.translation.z);
            // TF 查询的是 camera_optical_frame -> odom。Eigen::Isometry3d
            // 直接保存这个方向，传入 RuneCvGraph 后用于 camera pose 转 odom。
            T_camera_to_odom = Eigen::Isometry3d::Identity();
            T_camera_to_odom.rotate(q.toRotationMatrix());
            T_camera_to_odom.pretranslate(t);
            return true;
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_SKIPFIRST(get_logger(), "TF lookup failed: %s", e.what());
            return false;
        }
    }

    void publishInfo(
        const auto_aim_interfaces::msg::RuneTargets& msg,
        const graph_optimizer::RuneGraphOutput& output,
        const RunePublishPrediction& prediction) {
        auto info = auto_aim_interfaces::msg::RuneInfo();
        info.header = msg.header;
        info.mode = "graph";
        info.tracking = output.observed_count > 0 && !output.solve_result.failed;
        info.observed_count = output.observed_count;
        info.normal_yaw = output.state.normal_yaw;
        info.normal_pitch = normal_pitch_;
        info.optimized_roll = prediction.optimized_roll;
        info.predicted_roll = prediction.predicted_roll;
        info.angular_velocity = prediction.angular_velocity;
        info.curve_amplitude = prediction.curve_params[0];
        info.curve_frequency = prediction.curve_params[1];
        info.curve_phase = prediction.curve_params[2];
        info.curve_bias = prediction.curve_params[3];
        setPoint(info.target_position, prediction.target_position);
        legacy_info_pub_->publish(info);
        info_pub_->publish(info);
    }

    double relativeTime(const builtin_interfaces::msg::Time& stamp) {
        const rclcpp::Time current(stamp);
        if (!start_time_initialized_) {
            start_time_ = current;
            start_time_initialized_ = true;
        }
        return std::max(0.0, (current - start_time_).seconds());
    }

    double configPredictDt() const {
        return predict_dt_;
    }

    double unwrapRoll(double roll) {
        if (!roll_unwrap_initialized_) {
            continuous_roll_ = roll;
            roll_unwrap_initialized_ = true;
        } else {
            continuous_roll_ += auto_graph::shortestAngularDistance(continuous_roll_, roll);
        }
        return continuous_roll_;
    }

    RunePublishPrediction makePublishPrediction(
        const auto_aim_interfaces::msg::RuneTargets& msg,
        const graph_optimizer::RuneGraphOutput& output) {
        RunePublishPrediction prediction;
        const double t = relativeTime(msg.header.stamp);
        prediction.optimized_roll = unwrapRoll(output.state.roll);
        if (output.observed_count > 0) {
            if (!curve_time_initialized_) {
                curve_fitter_.clear();
                curve_fitter_.setC(prediction.optimized_roll);
                curve_time_initialized_ = true;
            }
            curve_fitter_.addSample(t, prediction.optimized_roll);
            curve_fitter_.fit();
        }

        const double predict_time = t + configPredictDt();
        const bool has_curve = curve_fitter_.hasFit();
        prediction.predicted_roll = has_curve
            ? curve_fitter_.angleAt(predict_time)
            : prediction.optimized_roll + output.state.vroll * configPredictDt();
        prediction.angular_velocity = has_curve
            ? curve_fitter_.velocityAt(t)
            : output.state.vroll;
        prediction.curve_params = curve_fitter_.params();
        prediction.target_position = graph_optimizer::runeTargetPosition(
            output.state.center, output.state.normal_yaw, normal_pitch_,
            prediction.predicted_roll);
        return prediction;
    }

    void publishTargetPose(
        const auto_aim_interfaces::msg::RuneTargets& msg,
        const graph_optimizer::RuneGraphOutput& output,
        const RunePublishPrediction& prediction) {
        auto target = auto_aim_interfaces::msg::TargetPose();
        target.header = msg.header;
        target.pose.position.x = prediction.target_position.x();
        target.pose.position.y = prediction.target_position.y();
        target.pose.position.z = prediction.target_position.z();
        target.pose.orientation = planeOrientation(output.state.normal_yaw, normal_pitch_);
        target_pose_pub_->publish(target);
    }

    void publishTrackerTarget(
        const auto_aim_interfaces::msg::RuneTargets& msg,
        const graph_optimizer::RuneGraphOutput& output,
        const RunePublishPrediction& prediction) {
        auto target = auto_aim_interfaces::msg::TrackerTarget();
        target.header = msg.header;
        target.enemy.tracking = output.observed_count > 0 && !output.solve_result.failed;
        target.enemy.position.x = prediction.target_position.x();
        target.enemy.position.y = prediction.target_position.y();
        target.enemy.position.z = prediction.target_position.z();
        target.enemy.orientation_yaw = output.state.normal_yaw;
        target.enemy.v_yaw = prediction.angular_velocity;
        // 复用 TrackerTarget 的字段承载能量机关输出：
        // radius_1/radius_2 都填风车半径，v_yaw 实际表示预测曲线角速度。
        target.enemy.radius_1 = graph_optimizer::kRuneBladeRadius;
        target.enemy.radius_2 = graph_optimizer::kRuneBladeRadius;
        target.armors_num = 1;
        tracker_target_pub_->publish(target);
    }

    void publishMarkers(
        const auto_aim_interfaces::msg::RuneTargets& msg,
        const graph_optimizer::RuneGraphOutput& output,
        const RunePublishPrediction& prediction) {
        auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
        const auto ts = rclcpp::Time(msg.header.stamp);

        visualization_msgs::msg::Marker center;
        center.header = msg.header;
        center.ns = "rune_center";
        center.id = 0;
        center.type = visualization_msgs::msg::Marker::SPHERE;
        center.action = visualization_msgs::msg::Marker::ADD;
        center.lifetime = rclcpp::Duration::from_seconds(0.1);
        center.scale.x = center.scale.y = center.scale.z = 0.06;
        center.color.a = 1.0;
        center.color.g = 0.8;
        center.color.b = 1.0;
        setPoint(center.pose.position, output.state.center);
        center.pose.orientation = planeOrientation(output.state.normal_yaw, normal_pitch_);
        markers->markers.push_back(center);

        visualization_msgs::msg::Marker target;
        target.header = msg.header;
        target.ns = "rune_target";
        target.id = 0;
        target.type = visualization_msgs::msg::Marker::SPHERE;
        target.action = visualization_msgs::msg::Marker::ADD;
        target.lifetime = rclcpp::Duration::from_seconds(0.1);
        target.scale.x = kRuneBladeMarkerThinAxis;
        target.scale.y = kRuneBladeMarkerWideAxis;
        target.scale.z = kRuneBladeMarkerWideAxis;
        target.color.a = 1.0;
        target.color.r = 1.0;
        target.color.g = 0.55;
        setPoint(target.pose.position, prediction.target_position);
        target.pose.orientation = eigenQuaternionToMsg(Eigen::Quaterniond(
            graph_optimizer::runeBladeRotation(
                output.state.normal_yaw, normal_pitch_, prediction.predicted_roll)));
        markers->markers.push_back(target);

        visualization_msgs::msg::Marker target_line;
        target_line.header = msg.header;
        target_line.ns = "rune_target";
        target_line.id = 10;
        target_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        target_line.action = visualization_msgs::msg::Marker::ADD;
        target_line.lifetime = rclcpp::Duration::from_seconds(0.1);
        target_line.scale.x = 0.005;
        target_line.color.a = 0.5;
        target_line.color.r = 1.0;
        target_line.color.g = 0.55;
        geometry_msgs::msg::Point target_center_point;
        geometry_msgs::msg::Point target_point;
        setPoint(target_center_point, output.state.center);
        setPoint(target_point, prediction.target_position);
        target_line.points = {target_center_point, target_point};
        markers->markers.push_back(target_line);

        auto add_blade_markers = [&](const std::string& ns,
                                     const Eigen::Vector3d& point,
                                     const Eigen::Quaterniond& orientation,
                                     int id,
                                     float r,
                                     float g,
                                     float b,
                                     float a) {
            visualization_msgs::msg::Marker blade;
            blade.header = msg.header;
            blade.ns = ns;
            blade.id = id;
            blade.type = visualization_msgs::msg::Marker::SPHERE;
            blade.action = visualization_msgs::msg::Marker::ADD;
            blade.lifetime = rclcpp::Duration::from_seconds(0.1);
            blade.scale.x = kRuneBladeMarkerThinAxis;
            blade.scale.y = kRuneBladeMarkerWideAxis;
            blade.scale.z = kRuneBladeMarkerWideAxis;
            blade.color.r = r;
            blade.color.g = g;
            blade.color.b = b;
            blade.color.a = a;
            setPoint(blade.pose.position, point);
            blade.pose.orientation = eigenQuaternionToMsg(orientation);
            markers->markers.push_back(blade);

            visualization_msgs::msg::Marker line;
            line.header = msg.header;
            line.ns = ns;
            line.id = id + 10;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.lifetime = rclcpp::Duration::from_seconds(0.1);
            line.scale.x = 0.005;
            line.color.r = r;
            line.color.g = g;
            line.color.b = b;
            line.color.a = a * 0.5f;
            geometry_msgs::msg::Point center_point;
            geometry_msgs::msg::Point blade_point;
            setPoint(center_point, output.state.center);
            setPoint(blade_point, point);
            line.points = {center_point, blade_point};
            markers->markers.push_back(line);
        };

        for (int i = 0; i < 5; ++i) {
            const double blade_roll = output.state.roll +
                static_cast<double>(i) * graph_optimizer::kRuneSlotAngle;
            const Eigen::Matrix3d R = graph_optimizer::runeBladeRotation(
                output.state.normal_yaw, normal_pitch_, blade_roll);
            const Eigen::Vector3d blade_point = graph_optimizer::runeTargetPosition(
                output.state.center, output.state.normal_yaw, normal_pitch_, blade_roll);
            add_blade_markers(
                "rune", blade_point, Eigen::Quaterniond(R), i, 0.0f, 0.45f, 1.0f, 0.55f);
        }

        int obs_id = 0;
        const auto observed_size = std::min(
            output.observed_pnp_poses.size(), output.observed_pnp_target_positions.size());
        for (size_t i = 0; i < observed_size; ++i) {
            const auto& observed_pose = output.observed_pnp_poses[i];
            const auto& observed_point = output.observed_pnp_target_positions[i];
            visualization_msgs::msg::Marker obs;
            obs.header = msg.header;
            obs.ns = "observed_position";
            obs.id = obs_id++;
            obs.type = visualization_msgs::msg::Marker::SPHERE;
            obs.action = visualization_msgs::msg::Marker::ADD;
            obs.lifetime = rclcpp::Duration::from_seconds(0.1);
            obs.scale.x = kRuneBladeMarkerThinAxis;
            obs.scale.y = kRuneBladeMarkerWideAxis;
            obs.scale.z = kRuneBladeMarkerWideAxis;
            obs.color.r = 1.0f;
            obs.color.g = 0.85f;
            obs.color.b = 0.0f;
            obs.color.a = 0.75f;
            setPoint(obs.pose.position, observed_point);
            obs.pose.orientation =
                eigenQuaternionToMsg(Eigen::Quaterniond(observed_pose.rotation()));
            markers->markers.push_back(obs);
        }

        visualization_msgs::msg::Marker text;
        text.header = msg.header;
        text.ns = "rune_status";
        text.id = 0;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.lifetime = rclcpp::Duration::from_seconds(0.1);
        setPoint(text.pose.position, output.target_position);
        text.pose.position.z += 0.12;
        text.scale.z = 0.08;
        text.color.a = 1.0;
        text.color.r = 1.0;
        text.color.g = 0.8;
        text.text = "roll=" + std::to_string(prediction.optimized_roll) +
            " pred=" + std::to_string(prediction.predicted_roll) +
            " obs=" + std::to_string(output.observed_count);
        (void)ts;
        markers->markers.push_back(text);
        marker_pub_->publish(std::move(markers));
    }

    double predict_dt_ = 0.15;
    double normal_pitch_ = 0.0;
    bool start_time_initialized_ = false;
    bool curve_time_initialized_ = false;
    bool roll_unwrap_initialized_ = false;
    double continuous_roll_ = 0.0;
    rclcpp::Time start_time_;
    graph_optimizer::RuneBigCurveFitter curve_fitter_;
    graph_optimizer::RuneCvGraph graph_;
    graph_optimizer::FrameTimeTracker frame_time_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    rclcpp::Subscription<auto_aim_interfaces::msg::RuneTargets>::SharedPtr targets_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::RuneInfo>::SharedPtr info_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::RuneInfo>::SharedPtr legacy_info_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::TargetPose>::SharedPtr target_pose_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::TrackerTarget>::SharedPtr tracker_target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace filter_test

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<filter_test::RuneGraphOptimizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

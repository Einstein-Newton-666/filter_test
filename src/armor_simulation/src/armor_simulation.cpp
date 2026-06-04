#include "armor_simulation/armor_simulation.hpp"
#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace armor_sim {

ArmorSimulation::ArmorSimulation(const rclcpp::NodeOptions & options)
: Node("armor_simulation_node", options) {
    // ═══════════════════════════════════════════════════════
    // 运动参数
    // ═══════════════════════════════════════════════════════
    publish_rate       = declare_parameter<int>("publish_rate", 15);
    linear_limit       = declare_parameter<double>("linear_limit", 3.5);
    angle_limit        = declare_parameter<double>("angle_limit", 10.0);
    linear_speed_limit = declare_parameter<double>("linear_speed_limit", 1.2);
    angle_speed_limit  = declare_parameter<double>("angle_speed_limit", 10.0);
    linear_acc         = declare_parameter<double>("linear_acceleration", 0.2);
    angle_acc          = declare_parameter<double>("angular_acceleration", 0.3924);

    x     = declare_parameter<double>("initial_state.x", 0.0);
    y     = declare_parameter<double>("initial_state.y", -5.0);
    yaw   = declare_parameter<double>("initial_state.yaw", 0.1);
    x_v   = declare_parameter<double>("initial_state.x_velocity", 0.0);
    y_v   = declare_parameter<double>("initial_state.y_velocity", -0.8);
    yaw_v = declare_parameter<double>("initial_state.yaw_velocity", 6.0);
    x_a   = declare_parameter<double>("initial_state.x_acceleration", 0.0);
    y_a   = declare_parameter<double>("initial_state.y_acceleration", 0.0);
    yaw_a = declare_parameter<double>("initial_state.yaw_acceleration", 0.1);

    z1 = declare_parameter<double>("geometry.z1", 0.0);
    z2 = declare_parameter<double>("geometry.z2", 0.15);
    r1 = declare_parameter<double>("geometry.r1", 0.28);
    r2 = declare_parameter<double>("geometry.r2", 0.38);

    radial_min_    = declare_parameter<double>("radial_min", 3.0);
    radial_max_    = declare_parameter<double>("radial_max", 7.0);
    process_noise_xy_  = declare_parameter<double>("process_noise_xy", 0.1);
    process_noise_yaw_ = declare_parameter<double>("process_noise_yaw", 0.5);
    noise_seed_        = declare_parameter<int>("process_noise_seed", 42);
    rng_ = std::mt19937(noise_seed_);
    accel_noise_dist_  = std::normal_distribution<double>(0.0, 1.0);

    // ═══════════════════════════════════════════════════════
    // 相机模型 (内参固定, 外参每帧从 TF 更新)
    // ═══════════════════════════════════════════════════════
    CameraIntrinsics intr;
    intr.fx           = declare_parameter<double>("camera_fx", 2411.0);
    intr.fy           = declare_parameter<double>("camera_fy", 2411.0);
    intr.cx           = declare_parameter<double>("camera_cx", 720.0);
    intr.cy           = declare_parameter<double>("camera_cy", 640.0);
    intr.k1           = declare_parameter<double>("camera_k1", -0.093);
    intr.k2           = declare_parameter<double>("camera_k2", 0.154);
    intr.p1           = declare_parameter<double>("camera_p1", 0.0001);
    intr.p2           = declare_parameter<double>("camera_p2", -0.0006);
    intr.image_width  = declare_parameter<int>("image_width", 1920);
    intr.image_height = declare_parameter<int>("image_height", 1440);
    image_width_  = intr.image_width;
    image_height_ = intr.image_height;
    cx_ = intr.cx;
    cy_ = intr.cy;
    camera_model_ = CameraModel(intr,
        // R_o2c=[0,-1,0; 0,0,-1; 1,0,0], ZYX欧拉: roll=π/2, pitch=-π/2, yaw=0
        // t_o2c=-R_o2c*(0,-0.045,0.08557)=(-0.045, 0.08557, 0)
        CameraModel::fromEulerAngles(M_PI_2, -M_PI_2, 0.0, -0.045, 0.08557, 0.0));

    // TF2 — 动态获取相机外参
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    // ═══════════════════════════════════════════════════════
    // 噪声模型
    // ═══════════════════════════════════════════════════════
    DetectionNoiseParams noise_params;
    noise_params.pixel_noise_optimal          = declare_parameter<double>("pixel_noise_optimal", 1.5);
    noise_params.pixel_noise_optimal_distance = declare_parameter<double>("pixel_noise_optimal_distance", 5.0);
    noise_params.pixel_noise_curvature        = declare_parameter<double>("pixel_noise_curvature", 0.125);
    noise_params.pixel_noise_common_ratio     = declare_parameter<double>("pixel_noise_common_ratio", 0.7);
    noise_params.use_outliers                 = declare_parameter<bool>("use_outliers", true);
    noise_params.outlier_probability          = declare_parameter<double>("outlier_probability", 0.05);
    noise_params.outlier_std                  = declare_parameter<double>("outlier_std", 10.0);
    noise_params.min_detectable_area          = declare_parameter<double>("min_detectable_area", 100.0);
    noise_params.max_detectable_distance      = declare_parameter<double>("max_detectable_distance", 8.0);
    noise_params.detection_probability        = declare_parameter<double>("detection_probability", 0.95);
    noise_params.miss_probability             = declare_parameter<double>("miss_probability", 0.05);
    noise_model_ = std::make_unique<DetectionNoise>(noise_params,
        declare_parameter<int>("noise_seed", 42));
    pixel_noise_enabled_ = declare_parameter<bool>("pixel_noise_enabled", true);
    enemy_color_ = declare_parameter<std::string>("enemy_color", "blue");
    publish_image_ = declare_parameter<bool>("publish_image", false);
    publish_gimbal_gt_ = declare_parameter<bool>("publish_gimbal_gt", false);
    img_disp_.ground_truth = declare_parameter<bool>("image_show_ground_truth", true);
    img_disp_.noisy        = declare_parameter<bool>("image_show_noisy", true);
    img_disp_.labels       = declare_parameter<bool>("image_show_labels", true);
    img_disp_.crosshair    = declare_parameter<bool>("image_show_crosshair", true);
    img_disp_.resize_scale = declare_parameter<double>("image_resize_scale", 1.0);

    // ═══════════════════════════════════════════════════════
    // ROS2 接口
    // ═══════════════════════════════════════════════════════
    detector_pub_ = create_publisher<auto_aim_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS());
    ground_truth_pub_ = create_publisher<armor_simulation::msg::GroundTruth>(
        "/armor_simulation/ground_truth", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/simulation/marker", 10);
    if (publish_image_) {
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/simulation/image", rclcpp::SensorDataQoS());
    }
    if (publish_gimbal_gt_) {
        gimbal_pub_ = create_publisher<auto_aim_interfaces::msg::SendData>(
            "/send_pack", rclcpp::SensorDataQoS());
    }

    position_marker_.ns = "position_";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.r = 0.0; position_marker_.color.g = 1.0; position_marker_.color.b = 1.0;

    // 真值 marker: 始终显示全部 4 块 (半透明绿色)
    gt_marker_.ns = "armors_gt";
    gt_marker_.type = visualization_msgs::msg::Marker::CUBE;
    gt_marker_.scale.x = 0.005; gt_marker_.scale.y = 0.135; gt_marker_.scale.z = SMALL_ARMOR_HEIGHT;
    gt_marker_.color.a = 0.5;
    gt_marker_.color.r = 0.0; gt_marker_.color.g = 1.0; gt_marker_.color.b = 0.0;
    gt_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    // 观测 marker: 仅通过过滤的装甲板 (不透明, 模拟检测器输出)
    obs_marker_.ns = "armors_obs";
    obs_marker_.type = visualization_msgs::msg::Marker::CUBE;
    obs_marker_.scale.x = 0.005; obs_marker_.scale.y = 0.135; obs_marker_.scale.z = SMALL_ARMOR_HEIGHT;
    obs_marker_.color.a = 1.0;
    obs_marker_.color.g = 0.7; obs_marker_.color.b = 1.0;
    obs_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    last_t = now();
    timer_ = create_wall_timer(
        1000ms / publish_rate, std::bind(&ArmorSimulation::publishSimulation, this));

    RCLCPP_INFO(get_logger(), "Armor simulation initialized (camera pose from TF)");
    RCLCPP_INFO(get_logger(), "  Camera: fx=%.1f fy=%.1f cx=%.1f cy=%.1f %dx%d",
                intr.fx, intr.fy, intr.cx, intr.cy, image_width_, image_height_);
    RCLCPP_INFO(get_logger(), "  Noise: sigma(d)=%.1f+%.3f*(d-%.1f)^2, outliers=%s(%d%%@%.0fpx)",
                noise_params.pixel_noise_optimal, noise_params.pixel_noise_curvature,
                noise_params.pixel_noise_optimal_distance,
                noise_params.use_outliers ? "on" : "off",
                static_cast<int>(noise_params.outlier_probability * 100),
                noise_params.outlier_std);
    RCLCPP_INFO(get_logger(), "  Image publish: %s", publish_image_ ? "ON (/simulation/image)" : "OFF");
}

void ArmorSimulation::publishSimulation() {
    rclcpp::Time now = this->now();
    double dt = std::min((now - last_t).seconds(), 1.0);
    last_t = now;

    // ── 仿真图像画布 (持久化缓冲区, 按需重建) ──
    if (publish_image_ && (sim_image_.empty() ||
        sim_image_.cols != image_width_ || sim_image_.rows != image_height_)) {
        sim_image_ = cv::Mat(image_height_, image_width_, CV_8UC3);
    }
    if (publish_image_) {
        sim_image_ = cv::Scalar(30, 30, 30);  // 清空为深灰背景
    }

    // ── 过程噪声 ──
    double dt_safe = (dt > 1e-9) ? dt : 0.01;
    double ax_noise = std::sqrt(process_noise_xy_ / dt_safe) * accel_noise_dist_(rng_);
    double ay_noise = std::sqrt(process_noise_xy_ / dt_safe) * accel_noise_dist_(rng_);
    double ayaw_noise = std::sqrt(process_noise_yaw_ / dt_safe) * accel_noise_dist_(rng_);
    double ax_eff = x_a + ax_noise, ay_eff = y_a + ay_noise, ayaw_eff = yaw_a + ayaw_noise;

    // ── 运动更新 ──
    x   += x_v * dt + 0.5 * ax_eff * dt * dt;
    y   += y_v * dt + 0.5 * ay_eff * dt * dt;
    yaw += yaw_v * dt + 0.5 * ayaw_eff * dt * dt;
    x_v   += ax_eff * dt;
    y_v   += ay_eff * dt;
    yaw_v += ayaw_eff * dt;

    // ── 从 TF 获取相机外参 (由 gimbal_simulation 节点动态更新) ──
    // 同时提取相机在 odom 系的位置 (用于正确的朝向过滤)
    Eigen::Vector3d camera_pos_odom = Eigen::Vector3d::Zero();
    try {
        auto tf = tf2_buffer_->lookupTransform(
            "odom", "camera_optical_frame", now, rclcpp::Duration::from_seconds(0.1));
        Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                             tf.transform.rotation.y, tf.transform.rotation.z);
        // TF lookup("odom","camera") 返回 camera→odom 变换:
        //   p_odom = R_c2o * p_cam + t_c2o
        // CameraModel.project3Dto2D 需要 odom→camera 变换:
        //   p_cam = R_o2c * p_odom + t_o2c  =  R_c2o^T * p_odom - R_c2o^T * t_c2o
        Eigen::Matrix3d R_c2o = q.toRotationMatrix();
        Eigen::Vector3d t_c2o(tf.transform.translation.x,
                              tf.transform.translation.y,
                              tf.transform.translation.z);
        camera_model_.setExtrinsics(R_c2o.transpose(), -(R_c2o.transpose() * t_c2o));
        // 平移量即相机在 odom 系的位置 (朝向过滤用)
        camera_pos_odom = t_c2o;
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_SKIPFIRST(get_logger(), "TF lookup odom→camera_optical_frame: %s", e.what());
        // 降级: camera_pos_odom 保持 Zero(), 等价于旧版原点到装甲板的近似
    }

    // ── 装甲板计算 + 相机投影 + 噪声 + PnP ──
    auto_aim_interfaces::msg::Armors detector_msg;
    detector_msg.header.stamp = now;
    detector_msg.header.frame_id = "odom";

    marker_array_.markers.clear();

    // 每帧先删全部 obs marker (配合 0.1s lifetime 双保险), 再 ADD 可见的
    for (int i = 0; i < 4; i++) {
        obs_marker_.id = i;
        obs_marker_.action = visualization_msgs::msg::Marker::DELETE;
        marker_array_.markers.push_back(obs_marker_);
    }

    static int frame_cnt = 0;
    int n_vis = 0;
    frame_cnt++;

    // ── 第一遍: 收集全部装甲板信息 + GT marker ──
    struct ArmorInfo {
        int idx; double armor_yaw; Eigen::Vector3d pos;
        Eigen::Quaterniond orient; double dist_cam; bool on_camera_side;
        std::array<Eigen::Vector2d, 4> corners_px;
        bool corners_valid = false;  // 全部角点投影有效
    };
    std::vector<ArmorInfo> armors_info;

    // 机器人中心→相机的方向
    double yaw_center_to_cam = std::atan2(
        camera_pos_odom.y() - y, camera_pos_odom.x() - x);

    for (int i = 0; i < 4; i++) {
        double armor_yaw = yaw + i * M_PI_2;
        double r  = (i % 2 == 0) ? r1 : r2;
        double dz = (i % 2 == 0) ? 0.0 : (z2 - z1);
        Eigen::Vector3d pos(x - cos(armor_yaw) * r, y - sin(armor_yaw) * r, z1 + dz);  // center - r*(cos,sin) 对齐 jlu
        double dist_cam = (pos - camera_pos_odom).norm();

        // 中心→装甲板方向 = armor_yaw + π (pos = center - r*n, 对齐 jlu)
        double yaw_center_to_armor = angles::normalize_angle(armor_yaw + M_PI);
        double angle_diff = std::abs(
            angles::shortest_angular_distance(yaw_center_to_armor, yaw_center_to_cam));
        bool on_camera_side = angle_diff <= M_PI / 3.0;  // ±60°

        // 装甲板方向
        tf2::Quaternion tf_q;
        tf_q.setRPY(0, 15.0 * M_PI / 180.0, angles::normalize_angle(armor_yaw));
        Eigen::Quaterniond orient(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());

        // 投影角点 (与检测使用相同约定: computeArmorCorners)
        std::array<Eigen::Vector2d, 4> corners_px;
        bool corners_valid = false;
        if (publish_image_) {
            auto gt_corners = computeArmorCorners(pos, orient, SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT);
            corners_valid = true;
            for (int j = 0; j < 4; j++) {
                corners_px[j] = camera_model_.project3Dto2D(gt_corners[j]);
                if (!camera_model_.isInImage(corners_px[j], 0.0))
                    corners_valid = false;
            }
        }

        armors_info.push_back({i, armor_yaw, pos, orient, dist_cam,
                               on_camera_side, corners_px, corners_valid});

        // GT marker (始终显示)
        gt_marker_.id = i;
        gt_marker_.action = visualization_msgs::msg::Marker::ADD;
        gt_marker_.header.stamp = now;
        gt_marker_.header.frame_id = "odom";
        gt_marker_.pose.position.x = pos.x();
        gt_marker_.pose.position.y = pos.y();
        gt_marker_.pose.position.z = pos.z();
        gt_marker_.pose.orientation = tf2::toMsg(tf_q);
        marker_array_.markers.push_back(gt_marker_);
    }

    // ── 图像: 绘制全部 4 块真值装甲板 + 旋转中心 + 半径连线 ──
    if (publish_image_) {
        // 机器人旋转中心投影
        Eigen::Vector3d center_pos(x, y, z1);
        Eigen::Vector2d center_px = camera_model_.project3Dto2D(center_pos);
        bool center_in_img = camera_model_.isInImage(center_px, 0.0);

        for (const auto& info : armors_info) {
            if (!info.corners_valid) continue;
            std::array<cv::Point, 4> pts;
            for (int j = 0; j < 4; j++)
                pts[j] = cv::Point(static_cast<int>(std::round(info.corners_px[j].x())),
                                   static_cast<int>(std::round(info.corners_px[j].y())));

            // 装甲板 GT 角点 (黄色半透明)
            for (int j = 0; j < 4; j++) {
                cv::circle(sim_image_, pts[j], 2, cv::Scalar(0, 220, 220), -1);
                cv::line(sim_image_, pts[j], pts[(j + 1) % 4], cv::Scalar(0, 180, 180), 1);
            }
            // 装甲板中心
            cv::Point ac(static_cast<int>((pts[0].x + pts[2].x) / 2),
                         static_cast<int>((pts[0].y + pts[2].y) / 2));

            // 旋转中心到装甲板连线 (青色)
            if (center_in_img) {
                cv::Point cc(static_cast<int>(std::round(center_px.x())),
                             static_cast<int>(std::round(center_px.y())));
                cv::line(sim_image_, cc, ac, cv::Scalar(200, 200, 0), 1);
            }
        }
        // 旋转中心 (绿色十字)
        if (center_in_img) {
            cv::Point cc(static_cast<int>(std::round(center_px.x())),
                         static_cast<int>(std::round(center_px.y())));
            cv::drawMarker(sim_image_, cc, cv::Scalar(0, 255, 0),
                           cv::MARKER_CROSS, 10, 2);
        }
    }

    // ── 筛选: 在相机侧的装甲板, 按距离排 ──
    std::vector<ArmorInfo*> candidates;
    for (auto& info : armors_info)
        if (info.on_camera_side) candidates.push_back(&info);
    std::sort(candidates.begin(), candidates.end(),
        [](const ArmorInfo* a, const ArmorInfo* b) { return a->dist_cam < b->dist_cam; });

    // ── 第二遍: 处理相机侧的装甲板 ──
    for (auto* info : candidates) {
        int i = info->idx;
        double armor_yaw = info->armor_yaw;
        Eigen::Vector3d pos = info->pos;
        double distance = info->dist_cam;


        double width  = SMALL_ARMOR_WIDTH;
        double height = SMALL_ARMOR_HEIGHT;
        auto corners = computeArmorCorners(pos, info->orient, width, height);
        double area = camera_model_.computeArea(corners);

        if (!noise_model_->shouldDetect(distance, area)) continue;

        std::array<Eigen::Vector2d, 4> pixels;
        bool all_in_image = true;
        for (int j = 0; j < 4; j++) {
            pixels[j] = camera_model_.project3Dto2D(corners[j]);
            if (!camera_model_.isInImage(pixels[j], 10.0)) { all_in_image = false; break; }
        }
        if (!all_in_image) {
            if (frame_cnt <= 3 || frame_cnt % 30 == 1) {
                RCLCPP_WARN(get_logger(), "  armor[%d] VIS FAIL dist=%.1f", i, distance);
            }
            continue;
        }
        n_vis++;

        // 保存真值投影
        std::array<Eigen::Vector2d, 4> pixels_gt;
        if (publish_image_) pixels_gt = pixels;

        if (pixel_noise_enabled_) pixels = noise_model_->addArmorPixelNoise(pixels, distance);

        auto pnp = camera_model_.estimatePose(pixels, width, height);
        if (!pnp.success) continue;

        auto_aim_interfaces::msg::Armor armor;
        armor.number = "4";
        armor.type = "small";
        armor.color = "blue";
        armor.area = area;
        armor.pose.position.x = pnp.position_odom.x();
        armor.pose.position.y = pnp.position_odom.y();
        armor.pose.position.z = pnp.position_odom.z();

        // 修正共面 PnP 双解歧义: 两个解 yaw 差 π, 选接近真值 (armor_yaw) 的.
        // 必须先修正 orientation 再写消息, 否则 pose.orientation 和 armor.yaw 会不一致.
        auto corrected_pose = correctPlanarPnPAmbiguity(
            pnp.orientation_odom, angles::normalize_angle(armor_yaw));
        armor.pose.orientation.x = corrected_pose.orientation.x();
        armor.pose.orientation.y = corrected_pose.orientation.y();
        armor.pose.orientation.z = corrected_pose.orientation.z();
        armor.pose.orientation.w = corrected_pose.orientation.w();
        armor.yaw = static_cast<float>(corrected_pose.yaw);
        armor.optimized_yaw = armor.yaw;
        for (int j = 0; j < 4; j++) {
            armor.detected_points[j].x = pixels[j].x();
            armor.detected_points[j].y = pixels[j].y();
        }
        detector_msg.armors.push_back(armor);

        // 观测 marker
        obs_marker_.id = i;
        obs_marker_.action = visualization_msgs::msg::Marker::ADD;
        obs_marker_.header.stamp = now;
        obs_marker_.header.frame_id = "odom";
        obs_marker_.pose.position.x = pos.x();
        obs_marker_.pose.position.y = pos.y();
        obs_marker_.pose.position.z = pos.z();
        tf2::Quaternion tf_q;
        tf_q.setRPY(0, 15.0 * M_PI / 180.0, angles::normalize_angle(armor_yaw));
        obs_marker_.pose.orientation = tf2::toMsg(tf_q);
        marker_array_.markers.push_back(obs_marker_);

        // 仿真图像绘制
        if (publish_image_) {
            std::array<cv::Point, 4> gt_pts, noise_pts;
            for (int j = 0; j < 4; j++) {
                gt_pts[j] = cv::Point(static_cast<int>(std::round(pixels_gt[j].x())),
                                      static_cast<int>(std::round(pixels_gt[j].y())));
                noise_pts[j] = cv::Point(static_cast<int>(std::round(pixels[j].x())),
                                         static_cast<int>(std::round(pixels[j].y())));
            }
            if (img_disp_.ground_truth) {
                for (int j = 0; j < 4; j++) {
                    cv::circle(sim_image_, gt_pts[j], 4, cv::Scalar(0, 255, 0), -1);
                    cv::line(sim_image_, gt_pts[j], gt_pts[(j + 1) % 4], cv::Scalar(0, 180, 0), 1);
                }
            }
            if (img_disp_.noisy) {
                for (int j = 0; j < 4; j++) {
                    cv::circle(sim_image_, noise_pts[j], 4, cv::Scalar(0, 0, 255), -1);
                    cv::line(sim_image_, noise_pts[j], noise_pts[(j + 1) % 4], cv::Scalar(0, 0, 200), 1);
                }
            }
            if (img_disp_.labels) {
                cv::putText(sim_image_, std::to_string(i), noise_pts[0] + cv::Point(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            }
        }
    }

    if (frame_cnt % 100 == 1) {
        static int last_fcnt = 0;
        static auto last_tick = std::chrono::steady_clock::now();
        auto now_tick = std::chrono::steady_clock::now();
        if (last_fcnt > 0) {
            double elapsed = std::chrono::duration<double>(now_tick - last_tick).count();
            RCLCPP_INFO(get_logger(),
                "frame %d: vis=%d pub=%zu pos=(%.1f,%.1f) r=%.1f %.0fHz dt=%.1fms",
                frame_cnt, n_vis, detector_msg.armors.size(), x, y,
                std::sqrt(x*x + y*y), 100.0 / elapsed, dt * 1000);
        }
        last_fcnt = frame_cnt;
        last_tick = now_tick;
    }

    if (!detector_msg.armors.empty())
        detector_pub_->publish(detector_msg);

    // ── 真值 ──
    armor_simulation::msg::GroundTruth truth;
    truth.header.stamp = now;
    truth.header.frame_id = "odom";
    truth.x    = x;     truth.y    = y;     truth.z    = z1;
    truth.yaw  = yaw;
    truth.vx   = x_v;   truth.vy   = y_v;   truth.vz   = 0.0;   truth.vyaw = yaw_v;
    truth.r1   = r1;    truth.r2   = r2;    truth.dz   = z2 - z1;
    ground_truth_pub_->publish(truth);

    // ── 可视化 ──
    position_marker_.header.stamp = now;
    position_marker_.header.frame_id = "odom";    // 球在 odom 系
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = x;
    position_marker_.pose.position.y = y;
    position_marker_.pose.position.z = z1;
    marker_array_.markers.emplace_back(position_marker_);
    marker_pub_->publish(marker_array_);

    // ── 发布仿真图像 ──
    if (publish_image_ && image_pub_) {
        // 左上角叠加帧诊断信息
        cv::putText(sim_image_,
            "f:" + std::to_string(frame_cnt) +
            " v:" + std::to_string(n_vis),
            cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.55,
            cv::Scalar(200, 200, 200), 1);

        // 按比例缩放 (减少发布带宽)
        const cv::Mat* out = &sim_image_;
        cv::Mat resized;
        double scale = std::max(0.1, std::min(img_disp_.resize_scale, 1.0));
        if (std::abs(scale - 1.0) > 1e-6) {
            cv::resize(sim_image_, resized, cv::Size(), scale, scale, cv::INTER_NEAREST);
            out = &resized;
        }

        auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
        img_msg->header.stamp = now;
        img_msg->header.frame_id = "camera_optical_frame";
        img_msg->height = out->rows;
        img_msg->width  = out->cols;
        img_msg->encoding = "bgr8";
        img_msg->is_bigendian = false;
        img_msg->step = static_cast<uint32_t>(out->step);
        size_t sz = out->rows * out->step;
        img_msg->data.assign(out->data, out->data + sz);
        image_pub_->publish(std::move(img_msg));
    }

    // ── 云台真值追踪: 直接发布目标角度到 /send_pack ──
    if (publish_gimbal_gt_ && gimbal_pub_) {
        auto send_msg = auto_aim_interfaces::msg::SendData();
        send_msg.header.stamp = now;
        double gimbal_yaw   = std::atan2(y, x);             // R_o2c 光轴 odom (cosθ,sinθ) → θ=atan2(ry,rx)
        double gimbal_pitch = -std::atan2(z1, std::sqrt(x * x + y * y));
        send_msg.yaw   = static_cast<float>(gimbal_yaw * 180.0 / M_PI);
        send_msg.pitch = static_cast<float>(gimbal_pitch * 180.0 / M_PI);
        gimbal_pub_->publish(send_msg);
    }

    // ── 运动限幅 ──
    constexpr double eps = 1e-6;
    if (std::abs(x_v) > linear_speed_limit && linear_speed_limit != -1)
        x_v = x_v / (std::abs(x_v) + eps) * linear_speed_limit;
    if (std::abs(y_v) > linear_speed_limit && linear_speed_limit != -1)
        y_v = y_v / (std::abs(y_v) + eps) * linear_speed_limit;
    if (std::abs(yaw_v) > angle_speed_limit && angle_speed_limit != -1)
        yaw_v = yaw_v / (std::abs(yaw_v) + eps) * angle_speed_limit;

    // ── 径向边界 (和 linear_limit 一样的直接反向) ──
    double r_now = std::sqrt(x*x + y*y);
    if (r_now > radial_max_) {
        if (std::abs(x) > eps) x_a = -x / (std::abs(x) + eps) * linear_acc;
        if (std::abs(y) > eps) y_a = -y / (std::abs(y) + eps) * linear_acc;
    } else if (r_now < radial_min_ && r_now > eps) {
        if (std::abs(x) > eps) x_a = x / (std::abs(x) + eps) * linear_acc;
        if (std::abs(y) > eps) y_a = y / (std::abs(y) + eps) * linear_acc;
    }

    if (std::abs(x) > linear_limit && linear_limit != -1)
        x_a = -x / (std::abs(x) + eps) * linear_acc;
    if (std::abs(y) > linear_limit && linear_limit != -1)
        y_a = -y / (std::abs(y) + eps) * linear_acc;
    if (std::abs(yaw) > angle_limit && angle_limit != -1)
        yaw_a = -yaw / (std::abs(yaw) + eps) * angle_acc;
}

}  // namespace armor_sim

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<armor_sim::ArmorSimulation>());
    rclcpp::shutdown();
    return 0;
}

#include "filter_test/graph_optimizer_test.hpp"
#include "filter_test/auto_graph_optimizer/utils/helpers.hpp"
#include "filter_test/common.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <algorithm>

namespace filter_test {

GraphOptimizerTest::GraphOptimizerTest(const rclcpp::NodeOptions& options)
    : Node("graph_optimizer_test", options) {
    // 声明参数
    declare_parameter("use_2d_observation", false);
    declare_parameter("s2qxy", 0.1);
    declare_parameter("s2qz", 0.1);
    declare_parameter("s2qyaw", 0.1);
    declare_parameter("s2qr", 10.0);
    declare_parameter("s2qdz", 0.1);
    declare_parameter("s2qvel", 0.1);
    declare_parameter("s2qvyaw", 0.1);
    declare_parameter("vel_sigma", 0.01);
    declare_parameter("vyaw_sigma", 0.05);
    declare_parameter("r_pose", 0.01);
    declare_parameter("r_distance", 0.01);
    declare_parameter("r_yaw", 0.01);
    declare_parameter("pixel_noise_std", 2.0);

    // 相机内参 (从参数服务器读取)
    declare_parameter("camera_fx", 2411.0);
    declare_parameter("camera_fy", 2411.0);
    declare_parameter("camera_cx", 720.0);
    declare_parameter("camera_cy", 640.0);
    // 畸变系数
    declare_parameter("distortion_k1", -0.093);
    declare_parameter("distortion_k2", 0.154);
    declare_parameter("distortion_p1", 0.0001);
    declare_parameter("distortion_p2", -0.0006);
    // 获取参数
    use_2d_observation_ = get_parameter("use_2d_observation").as_bool();
    s2qxy_ = get_parameter("s2qxy").as_double();
    s2qz_ = get_parameter("s2qz").as_double();
    s2qyaw_ = get_parameter("s2qyaw").as_double();
    s2qr_ = get_parameter("s2qr").as_double();
    s2qdz_ = get_parameter("s2qdz").as_double();
    s2qvel_ = get_parameter("s2qvel").as_double();
    s2qvyaw_ = get_parameter("s2qvyaw").as_double();
    vel_sigma_ = get_parameter("vel_sigma").as_double();
    vyaw_sigma_ = get_parameter("vyaw_sigma").as_double();
    r_pose_ = get_parameter("r_pose").as_double();
    r_distance_ = get_parameter("r_distance").as_double();
    r_yaw_ = get_parameter("r_yaw").as_double();
    pixel_sigma_ = get_parameter("pixel_noise_std").as_double();

    // 相机内参
    double camera_fx = get_parameter("camera_fx").as_double();
    double camera_fy = get_parameter("camera_fy").as_double();
    double camera_cx = get_parameter("camera_cx").as_double();
    double camera_cy = get_parameter("camera_cy").as_double();
    camera_matrix_ << camera_fx, 0.0, camera_cx,
                      0.0, camera_fy, camera_cy,
                      0.0, 0.0, 1.0;

    distortion_[0] = get_parameter("distortion_k1").as_double();
    distortion_[1] = get_parameter("distortion_k2").as_double();
    distortion_[2] = get_parameter("distortion_p1").as_double();
    distortion_[3] = get_parameter("distortion_p2").as_double();
    distortion_[4] = 0.0;
    // TF2: 相机外参从 TF 树动态获取 (odom → camera_optical_frame)
    // 跨钟: 云台仿真器发布 odom→gimbal_link (动态) + gimbal_link→camera_optical_frame (固定)
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    // 配置图优化器
    declare_parameter("verbose", true);
    declare_parameter("cold_start_frames", 3);
    declare_parameter("smoother_lag", 0.0);
    declare_parameter("smoother_type", "incremental");
    declare_parameter("relinearize_threshold", 0.001);
    declare_parameter("extra_iterations", 2);
    declare_parameter("max_window_frames", 500);
    config_.cold_start_frames = get_parameter("cold_start_frames").as_int();
    config_.verbose = get_parameter("verbose").as_bool();
    config_.smoother_lag = get_parameter("smoother_lag").as_double();
    std::string st = get_parameter("smoother_type").as_string();
    config_.smoother_type = (st == "batch")
        ? auto_graph::SmootherType::Batch
        : auto_graph::SmootherType::Incremental;
    config_.relinearize_threshold = get_parameter("relinearize_threshold").as_double();
    config_.extra_iterations = get_parameter("extra_iterations").as_int();
    config_.max_window_frames = get_parameter("max_window_frames").as_int();

    // 创建订阅者
    armors_sub_ = create_subscription<auto_aim_interfaces::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS(),
        [this](const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
            armorsCallback(msg);
        });

    // 创建发布者
    result_pub_ = create_publisher<auto_aim_interfaces::msg::Armors>(
        "/graph_optimizer/armors", rclcpp::SensorDataQoS());
    tracker_target_pub_ = create_publisher<auto_aim_interfaces::msg::TrackerTarget>(
        "/tracker/target", rclcpp::SensorDataQoS());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/graph_optimizer/marker", 10);

    RCLCPP_INFO(get_logger(), "Graph optimizer test initialized");
    RCLCPP_INFO(get_logger(), "  use_2d_observation: %s", use_2d_observation_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  s2qxy: %.3f, s2qz: %.3f, s2qyaw: %.3f", s2qxy_, s2qz_, s2qyaw_);
}

void GraphOptimizerTest::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
    if (msg->armors.empty()) {
        return;
    }

    // 初始化
    if (!initialized_) {
        initializeOptimizer(msg);
        initialized_ = true;
        last_time_ = msg->header.stamp;
        return;
    }

    // 计算dt
    rclcpp::Time current_time = msg->header.stamp;
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // 限制dt范围
    if (dt <= 0 || dt > 1.0) {
        dt = 0.01;
    }

    // 构建过程噪声 Q (11D)
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
    double t = dt;
    double q_xx = std::pow(t, 4) / 4 * s2qxy_, q_xv = std::pow(t, 3) / 2 * s2qxy_, q_vv = std::pow(t, 2) * s2qxy_;
    Q(0, 0) = q_xx; Q(0, 1) = q_xv; Q(1, 0) = q_xv; Q(1, 1) = q_vv;
    Q(2, 2) = q_xx; Q(2, 3) = q_xv; Q(3, 2) = q_xv; Q(3, 3) = q_vv;
    Q(4, 4) = std::pow(t, 4) / 4 * s2qz_; Q(4, 5) = std::pow(t, 3) / 2 * s2qz_;
    Q(5, 4) = Q(4, 5); Q(5, 5) = std::pow(t, 2) * s2qz_;
    Q(6, 6) = std::pow(t, 4) / 4 * s2qyaw_; Q(6, 7) = std::pow(t, 3) / 2 * s2qyaw_;
    Q(7, 6) = Q(6, 7); Q(7, 7) = std::pow(t, 2) * s2qyaw_;
    Q(8, 8) = std::pow(t, 4) / 4 * s2qr_; Q(9, 9) = Q(8, 8); Q(10, 10) = std::pow(t, 4) / 4 * s2qdz_;

    // ── 运动预测 ──
    if (use_2d_observation_) {
        // Per-group 运动模型: advanceFrame + addMotionFactor
        optimizer_->advanceFrame(dt);

        // pos_vel: 对角噪声 (替换 CWNA). dt=0.01 时 CWNA σ_pos≈1.6e-5m
        // → 运动因子比观测因子强 10^7 倍 → 数值奇异.
        // 对角噪声: 位置宽松 (让观测主导), 速度收紧 (防止随机游走累积发散)
        // σ_pos=0.32, σ_vel=0.01 → cond(Q)=1000, 1800帧后累积σ_v≈0.42m/s
        Eigen::MatrixXd Qt = Eigen::MatrixXd::Zero(6, 6);
        Qt(0, 0) = s2qxy_;   Qt(1, 1) = s2qvel_;
        Qt(2, 2) = s2qxy_;   Qt(3, 3) = s2qvel_;
        Qt(4, 4) = s2qz_;    Qt(5, 5) = s2qvel_;
        optimizer_->addMotionFactor<TranslationModel>("pos_vel", TranslationModel(dt), Qt);

        // yaw_vyaw: 对角噪声, 同样 yaw 宽松 vyaw 收紧
        Eigen::MatrixXd Qy = Eigen::MatrixXd::Zero(2, 2);
        Qy(0, 0) = s2qyaw_;  Qy(1, 1) = s2qvyaw_;
        optimizer_->addMotionFactor<YawModel>("yaw_vyaw", YawModel(dt), Qy);
    } else {
        optimizer_->predict<ArmorCVMotionModel>(dt, Q);
    }

    // 构建观测 (匹配移到每个装甲板循环内)
    if (use_2d_observation_) {
        // ── 两级像素观测 (参考 jlu_vision_26) ──
        // TF lookup("odom","camera") 返回 camera→odom 变换: p_odom = R_c2o * p_cam + t_c2o
        Eigen::Isometry3d T_camera_to_odom{Eigen::Isometry3d::Identity()};
        Eigen::Isometry3d T_odom_to_camera{Eigen::Isometry3d::Identity()};
        try {
            auto tf = tf2_buffer_->lookupTransform(
                "odom", "camera_optical_frame", current_time, rclcpp::Duration::from_seconds(0.1));
            Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                                tf.transform.rotation.y, tf.transform.rotation.z);
            Eigen::Matrix3d R_c2o = q.toRotationMatrix();
            Eigen::Vector3d t_c2o(tf.transform.translation.x, tf.transform.translation.y,
                                  tf.transform.translation.z);
            // 直接构造 T_camera_to_odom (与 jlu_vision_26 因子约定一致)
            T_camera_to_odom = Eigen::Isometry3d::Identity();
            T_camera_to_odom.rotate(R_c2o);
            T_camera_to_odom.pretranslate(t_c2o);
            // 逆变换用于 armor pose 转换到 camera 系
            T_odom_to_camera = T_camera_to_odom.inverse();
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_SKIPFIRST(get_logger(), "TF lookup failed: %s", e.what());
        }

        Eigen::VectorXd st = optimizer_->getState();
        uint64_t fid = optimizer_->getFrameId();
        auto k_pos_vel = optimizer_->key("pos_vel", fid);
        auto k_yaw_vyaw = optimizer_->key("yaw_vyaw", fid);
        auto k_radius = optimizer_->key("radius", 0);
        auto k_dz = optimizer_->key("dz", 0);

        // ── 速度平滑正则化 (对齐 jlu VelocityFactor/VyawFactor) ──
        // 提供额外的因子冗余, 改善 6D 密集团的计算条件
        gtsam::Key k_pos_prev = optimizer_->key("pos_vel", fid - 1);
        optimizer_->addCustomFactor<VelSmoothFactor>(
            k_pos_prev, k_pos_vel,
            gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector3(vel_sigma_, vel_sigma_, vel_sigma_)));

        gtsam::Key k_yaw_prev = optimizer_->key("yaw_vyaw", fid - 1);
        optimizer_->addCustomFactor<VyawSmoothFactor>(
            k_yaw_prev, k_yaw_vyaw,
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(vyaw_sigma_)));

        // 装甲板角点局部坐标 (与 armor_simulation 一致)
        // autoaim 约定: X=前(法向), Y=左(宽度), Z=上(高度)
        std::array<Eigen::Vector3d, 4> corners_local = {{
            {0, -0.0675, -0.0625}, {0, -0.0675, 0.0625},
            {0,  0.0675,  0.0625}, {0,  0.0675,-0.0625}
        }};

        for (size_t ai = 0; ai < msg->armors.size(); ai++) {
            const auto& armor = msg->armors[ai];
            int idx = matchArmor(armor);
            gtsam::Key akey = optimizer_->armorPoseKey(fid, idx);

            // ── Armor Pose3 初值: 从 PnP 观测提取 RPY (对齐 jlu) ──
            Eigen::Vector3d armor_pos_odom(armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);
            Eigen::Vector3d armor_pos_camera = T_odom_to_camera * armor_pos_odom;

            // 从 PnP 四元数提取完整 RPY (不用硬编码 pitch!)
            tf2::Quaternion q_armor_odom;
            tf2::fromMsg(armor.pose.orientation, q_armor_odom);
            Eigen::Quaterniond eq_armor_odom(q_armor_odom.w(), q_armor_odom.x(),
                                             q_armor_odom.y(), q_armor_odom.z());
            Eigen::Quaterniond eq_armor_camera(
                T_odom_to_camera.rotation() * eq_armor_odom.toRotationMatrix());

            gtsam::Pose3 armor_pose_camera(
                gtsam::Rot3::Quaternion(eq_armor_camera.w(), eq_armor_camera.x(),
                                        eq_armor_camera.y(), eq_armor_camera.z()),
                gtsam::Point3(armor_pos_camera.x(), armor_pos_camera.y(), armor_pos_camera.z()));

            optimizer_->insertArmorPose(akey, armor_pose_camera);

            // ── ReprojFactor: 4 个因子, 每个角点一个 (2D 误差) ──
            double pixel_std = pixel_sigma_;
            gtsam::SharedNoiseModel pixel_noise =
                gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(pixel_std, pixel_std));

            for (int ci = 0; ci < 4; ci++) {
                if (static_cast<size_t>(ci) < armor.detected_points.size()) {
                    Eigen::Vector2d px_obs(armor.detected_points[ci].x,
                                           armor.detected_points[ci].y);
                    optimizer_->addCustomFactor<ArmorReprojFactor>(
                        akey, pixel_noise, corners_local[ci],
                        camera_matrix_, distortion_, px_obs);
                }
            }

            // ── Armor Pose3 先验 (从 PnP, 强化防止欠约束) ──
            // σ=0.1 给 armor pose 足够的约束, 防止 iSAM2 报 "underconstrained"
            gtsam::Vector6 prior_sig; prior_sig << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
            optimizer_->addPose3Prior(akey, armor_pose_camera,
                gtsam::noiseModel::Diagonal::Sigmas(prior_sig));

            // ── ArmorCenterFactor: camera 系 Pose3 → 几何约束 ──
            gtsam::Vector4 geo_sig(geo_noise_.tangential, geo_noise_.radial,
                                    geo_noise_.height, geo_noise_.yaw);
            optimizer_->addCustomFactor<ArmorCenterFactor>(
                akey, k_pos_vel, k_yaw_vyaw, k_radius, k_dz,
                gtsam::noiseModel::Diagonal::Sigmas(geo_sig), idx, T_camera_to_odom,
                kGraphOptimizerRadiusMin, kGraphOptimizerRadiusMax);
        }
    } else {
        // YPD观测 (支持单板/双板)
        if (msg->armors.size() >= 2) {
            // ── 双板观测: 相邻对匹配 + 8D 观测 ──
            constexpr std::array<std::pair<int, int>, 4> adjacent_pairs = {{
                {0, 1}, {1, 2}, {2, 3}, {3, 0}
            }};
            double yaw1 = msg->armors[0].yaw;
            double yaw2 = msg->armors[1].yaw;
            auto best_pair = adjacent_pairs[0];
            double min_total_diff = std::numeric_limits<double>::max();
            for (const auto& pair : adjacent_pairs) {
                double diff1 = std::abs(auto_graph::shortestAngularDistance(
                    pair.first * M_PI_2, yaw1)) +
                    std::abs(auto_graph::shortestAngularDistance(
                    pair.second * M_PI_2, yaw2));
                double diff2 = std::abs(auto_graph::shortestAngularDistance(
                    pair.second * M_PI_2, yaw1)) +
                    std::abs(auto_graph::shortestAngularDistance(
                    pair.first * M_PI_2, yaw2));
                double total_diff = std::min(diff1, diff2);
                if (total_diff < min_total_diff) {
                    min_total_diff = total_diff;
                    best_pair = diff1 < diff2
                        ? std::make_pair(pair.first, pair.second)
                        : std::make_pair(pair.second, pair.first);
                }
            }

            // 构建 8D 观测向量
            Eigen::VectorXd z = Eigen::VectorXd::Zero(8);
            Eigen::MatrixXd R_mat = Eigen::MatrixXd::Zero(8, 8);
            for (int i = 0; i < 2; i++) {
                const auto& armor = msg->armors[i];
                double yaw_obs = std::atan2(armor.pose.position.y, armor.pose.position.x);
                double dist = std::sqrt(armor.pose.position.x * armor.pose.position.x +
                                        armor.pose.position.y * armor.pose.position.y +
                                        armor.pose.position.z * armor.pose.position.z);
                double pitch = std::atan2(armor.pose.position.z,
                    std::sqrt(armor.pose.position.x * armor.pose.position.x +
                              armor.pose.position.y * armor.pose.position.y));
                int off = i * 4;
                z[off + 0] = yaw_obs;
                z[off + 1] = pitch;
                z[off + 2] = dist;
                z[off + 3] = armor.yaw;
                R_mat(off + 0, off + 0) = r_pose_;
                R_mat(off + 1, off + 1) = r_pose_;
                R_mat(off + 2, off + 2) = r_distance_ * dist * dist;
                R_mat(off + 3, off + 3) = r_yaw_;
            }
            ArmorCVMeasureYPDDouble measure_double(best_pair.first, best_pair.second);

            // 角度连续性处理
            Eigen::VectorXd x_st = optimizer_->getState();
            for (int i = 0; i < 2; ++i) {
                int off = i * 4;
                int idx = (i == 0) ? best_pair.first : best_pair.second;
                double pred_yaw = std::atan2(x_st[2], x_st[0]) + idx * M_PI_2;
                z[off] = get_closest_angle(z[off], pred_yaw);
                z[off + 3] = get_closest_angle(z[off + 3],
                    x_st[6] + M_PI_2 * idx);
            }
            optimizer_->update<ArmorCVMeasureYPDDouble>(measure_double, z, R_mat);
        } else {
            // 单板观测 (fallback)
            const auto& armor = msg->armors[0];
            matchArmor(armor);
            double yaw_obs = std::atan2(armor.pose.position.y, armor.pose.position.x);
            double distance = std::sqrt(
                armor.pose.position.x * armor.pose.position.x +
                armor.pose.position.y * armor.pose.position.y +
                armor.pose.position.z * armor.pose.position.z);
            double pitch = std::atan2(armor.pose.position.z,
                std::sqrt(armor.pose.position.x * armor.pose.position.x +
                          armor.pose.position.y * armor.pose.position.y));

            Eigen::VectorXd z = Eigen::VectorXd::Zero(4);
            z[0] = yaw_obs;
            z[1] = pitch;
            z[2] = distance;
            z[3] = armor.yaw;  // orient_yaw

            Eigen::MatrixXd R_mat = Eigen::MatrixXd::Zero(4, 4);
            R_mat(0, 0) = r_pose_;
            R_mat(1, 1) = r_pose_;
            R_mat(2, 2) = r_distance_ * distance * distance;
            R_mat(3, 3) = r_yaw_;
            optimizer_->update<ArmorCVMeasureYPD>(z, R_mat);
        }
    }

    // iSAM2 增量优化 (冷启动前 3 帧只建图, 之后每帧优化)
    optimizer_->solve();

    // 发布结果
    publishResult();
    publishMarkers(msg);
    publishTrackerTarget();

    // 打印状态（每10帧打印一次）
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 10 == 0) {
        Eigen::VectorXd state = optimizer_->getState();
        double r1 = graphOptimizerRadiusFromState(state[8]);
        double r2 = graphOptimizerRadiusFromState(state[9]);
        RCLCPP_INFO(get_logger(), "Frame %d: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f, r1=%.3f, r2=%.3f, dz=%.3f",
                    frame_count, state[0], state[2], state[4], state[6], r1, r2, state[10]);
    }
}

void GraphOptimizerTest::initializeOptimizer(const auto_aim_interfaces::msg::Armors::SharedPtr& msg) {
    // 创建优化器
    optimizer_ = std::make_unique<auto_graph::GraphOptimizer>(config_);

    // 定义变量布局: 11D 全状态 → 4 个 GTSAM 子变量
    auto layout = auto_graph::VariableLayout(11)
        .addDynamic("pos_vel",  {0, 1, 2, 3, 4, 5})  // 6D: xc,vxc,yc,vyc,za,vza
        .addDynamic("yaw_vyaw", {6, 7})               // 2D: yaw,vyaw
        .addStatic ("radius",   {8, 9})               // 2D: r1_u,r2_u (logistic空间)
        .addStatic ("dz",       {10});                // 1D: dz

    // 从第一个观测初始化状态
    const auto& armor = msg->armors[0];

    double init_r_phys = 0.25;
    double init_r_u = graphOptimizerRadiusToState(init_r_phys);

    // 单块装甲板初始化时, 若初始 r1/r2 相等且 dz=0, 装甲板编号只能确定到
    // π/2 等价类: center = armor_pos + r * [cos(armor_yaw), sin(armor_yaw)].
    // 先把首个观测视为 index 0, 后续帧由 matchArmor() 维持连续编号。
    int best_init_idx = 0;
    last_armor_index_ = best_init_idx;
    double robot_yaw_init = armor.yaw;

    // 状态向量：[xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r1_u, r2_u, dz]
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(11);
    x0[0] = armor.pose.position.x + init_r_phys * std::cos(armor.yaw);  // xc
    x0[1] = 0.0;
    x0[2] = armor.pose.position.y + init_r_phys * std::sin(armor.yaw);  // yc
    x0[3] = 0.0;
    x0[4] = armor.pose.position.z;                                        // za
    x0[5] = 0.0;
    x0[6] = robot_yaw_init;                                               // yaw
    x0[7] = 0.0;
    x0[8] = init_r_u;
    x0[9] = init_r_u;
    x0[10] = 0.0;

    // 初始协方差
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(11, 11);
    P0(0, 0) = P0(2, 2) = P0(4, 4) = 0.01;    // σ_pos=0.1 (对齐 jlu)
    P0(1, 1) = P0(3, 3) = P0(5, 5) = P0(7, 7) = 0.25;  // σ_vel=0.5 (对齐 jlu)
    P0(6, 6) = 0.05;
    P0(8, 8) = P0(9, 9) = 0.25;  // σ_radius=0.5 (对齐 jlu)
    P0(10, 10) = 0.25;            // σ_dz=0.5 (对齐 jlu)

    optimizer_->initialize(layout, x0, P0);

    RCLCPP_INFO(get_logger(), "Optimizer initialized: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f (robot_yaw=%.2f, best_idx=%d)",
                x0[0], x0[2], x0[4], x0[6], robot_yaw_init, best_init_idx);
}

int GraphOptimizerTest::matchArmor(const auto_aim_interfaces::msg::Armor& armor) {
    double obs_yaw = armor.yaw;
    double obs_x = armor.pose.position.x;
    double obs_y = armor.pose.position.y;
    double obs_z = armor.pose.position.z;
    Eigen::VectorXd state = optimizer_->getState();

    double best_cost = std::numeric_limits<double>::max();
    int best_index = last_armor_index_;
    for (int i = 0; i < 4; i++) {
        double pred_yaw = state[6] + i * M_PI_2;
        double yaw_diff = std::abs(pred_yaw - obs_yaw);
        yaw_diff = std::fmod(yaw_diff, 2.0 * M_PI);
        if (yaw_diff > M_PI) yaw_diff = 2.0 * M_PI - yaw_diff;

        // logistic → physical radius
        double r_phys = graphOptimizerRadiusFromState(state[8 + i % 2]);
        double dz = (i % 2 == 0) ? 0.0 : state[10];
        double pred_x = state[0] - std::cos(pred_yaw) * r_phys;
        double pred_y = state[2] - std::sin(pred_yaw) * r_phys;
        double pred_z = state[4] + dz;
        double pos_err = std::sqrt((obs_x - pred_x) * (obs_x - pred_x) +
                                   (obs_y - pred_y) * (obs_y - pred_y) +
                                   (obs_z - pred_z) * (obs_z - pred_z));

        double cost = yaw_diff + 3.0 * pos_err;
        if (i == last_armor_index_) cost *= 0.85;
        if (cost < best_cost) { best_cost = cost; best_index = i; }
    }

    last_armor_index_ = best_index;
    return best_index;
}

void GraphOptimizerTest::publishResult() {
    auto_aim_interfaces::msg::Armors result;
    result.header.stamp = now();
    result.header.frame_id = "odom";

    auto_aim_interfaces::msg::Armor armor;
    Eigen::VectorXd state = optimizer_->getState();

    armor.pose.position.x = state[0];
    armor.pose.position.y = state[2];
    armor.pose.position.z = state[4];

    tf2::Quaternion q;
    q.setRPY(0, 0, state[6]);
    armor.pose.orientation = tf2::toMsg(q);

    armor.yaw = state[6];
    armor.type = "small";
    armor.number = "4";

    result.armors.push_back(armor);
    result_pub_->publish(result);
}

void GraphOptimizerTest::publishTrackerTarget() {
    auto_aim_interfaces::msg::TrackerTarget target;
    target.header.stamp = now();
    target.header.frame_id = "odom";
    Eigen::VectorXd state = optimizer_->getState();
    double r1_phys = graphOptimizerRadiusFromState(state[8]);
    double r2_phys = graphOptimizerRadiusFromState(state[9]);

    target.enemy.tracking = true;
    target.enemy.position.x = state[0];
    target.enemy.position.y = state[2];
    target.enemy.position.z = state[4];
    target.enemy.velocity.x = state[1];
    target.enemy.velocity.y = state[3];
    target.enemy.velocity.z = state[5];
    target.enemy.orientation_yaw = state[6];
    target.enemy.v_yaw = state[7];
    target.enemy.radius_1 = r1_phys;
    target.enemy.radius_2 = r2_phys;
    target.enemy.dz = state[10];

    target.armors_num = 4;
    tracker_target_pub_->publish(target);
}

void GraphOptimizerTest::publishMarkers(const auto_aim_interfaces::msg::Armors::SharedPtr& msg) {
    auto state = optimizer_->getState();
    double xc = state[0], yc = state[2], za = state[4];
    double yaw = state[6], dz = state[10];
    // 物理半径 (预先从 logistic 空间转换, 参考 jlu_tracker_node)
    double r1 = graphOptimizerRadiusFromState(state[8]);
    double r2 = graphOptimizerRadiusFromState(state[9]);

    auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
    auto ts = now();

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
    for (size_t i = 0; i < msg->armors.size(); i++) {
        const auto& obs = msg->armors[i];
        visualization_msgs::msg::Marker am;
        am.header.stamp = ts; am.header.frame_id = "odom";
        am.ns = "armors_obs"; am.id = static_cast<int>(i);
        am.type = visualization_msgs::msg::Marker::CUBE;
        am.action = visualization_msgs::msg::Marker::ADD;
        am.lifetime = rclcpp::Duration::from_seconds(0.1);
        am.pose.position.x = obs.pose.position.x;
        am.pose.position.y = obs.pose.position.y;
        am.pose.position.z = obs.pose.position.z;
        // 直接使用 PnP yaw (已修正共面双解歧义), 避免 atan2 在 π 附近跳变
        am.pose.orientation = tf2::toMsg(observedArmorMarkerQuaternion(obs.yaw));
        am.scale.x = 0.005; am.scale.y = 0.135; am.scale.z = 0.125;
        am.color.a = 1.0; am.color.r = 1.0; am.color.g = 0.0; am.color.b = 0.0;
        markers->markers.push_back(am);
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

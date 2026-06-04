#pragma once

#include <rclcpp/rclcpp.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/armor.hpp>
#include <auto_aim_interfaces/msg/tracker_target.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "filter_test/auto_graph_optimizer/graph_optimizer.hpp"
#include "filter_test/auto_graph_optimizer/models/motion_model.hpp"
#include "filter_test/auto_graph_optimizer/models/measure_model.hpp"
#include "filter_test/auto_graph_optimizer/utils/helpers.hpp"
#include "filter_test/visualization_marker_utils.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace filter_test {

/**
 * 图优化测试节点
 *
 * 订阅兼容autoaim的Armors消息，使用图优化框架进行跟踪。
 * 支持两种观测模型：
 * 1. YPD观测（3D球坐标）
 * 2. 像素坐标观测（2D图像）
 */
class GraphOptimizerTest : public rclcpp::Node {
public:
    GraphOptimizerTest(const rclcpp::NodeOptions& options);

private:
    /**
     * 订阅Armors消息（兼容autoaim接口）
     */
    void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg);

    /**
     * 初始化图优化器
     */
    void initializeOptimizer(const auto_aim_interfaces::msg::Armors::SharedPtr& msg);

    /**
     * 匹配装甲板
     */
    int matchArmor(const auto_aim_interfaces::msg::Armor& armor);

    /**
     * 发布跟踪结果
     */
    void publishResult();

    /**
     * 发布 tracker 目标 (供 angle_solver 使用)
     */
    void publishTrackerTarget();

    /**
     * 发布可视化标记
     */
    void publishMarkers(const auto_aim_interfaces::msg::Armors::SharedPtr& msg);

    // 图优化器
    std::unique_ptr<auto_graph::GraphOptimizer> optimizer_;
    auto_graph::GraphOptimizerConfig config_;

    // 状态
    bool initialized_ = false;
    rclcpp::Time last_time_;
    int last_armor_index_ = -1;

    // 参数
    bool use_2d_observation_ = false;

    // 过程噪声 (per-group)
    double s2qxy_ = 0.1;
    double s2qz_ = 0.1;
    double s2qyaw_ = 0.1;
    double s2qr_ = 10.0;
    double s2qdz_ = 0.1;

    // 速度平滑噪声 (2D观测模式)
    double s2qvel_ = 0.1;
    double s2qvyaw_ = 0.1;

    // 速度正则化 sigma (对齐 jlu VelocityFactor / VyawFactor)
    double vel_sigma_ = 0.01;
    double vyaw_sigma_ = 0.05;

    // 观测噪声 (YPD)
    double r_pose_ = 0.01;
    double r_distance_ = 0.01;
    double r_yaw_ = 0.01;

    // 像素观测噪声
    double pixel_sigma_ = 1.0;

    // 几何约束噪声 [tangential, radial, z, yaw]
    struct GeoNoise {
        double tangential = 0.01;
        double radial = 0.03;
        double height = 0.01;
        double yaw = 0.005;
    } geo_noise_;

    // 先验噪声
    struct PriorNoise {
        double radius = 1.0;
        double dz = 1.0;
    } prior_noise_;

    // 相机内参 (从参数服务器读取)
    Eigen::Matrix3d camera_matrix_ = Eigen::Matrix3d::Identity();
    std::array<double, 5> distortion_ = {0, 0, 0, 0, 0};

    // TF2 (相机外参从 TF 树动态获取，兼容 autoaim 云台架构)
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    // ROS2接口
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr result_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::TrackerTarget>::SharedPtr tracker_target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

/**
 * Per-group 运动模型 — 每个模型只负责自己的 group, 维度独立
 *
 * 配合 advanceFrame() + addMotionFactor() 使用
 */

// Translation: pos(k) = pos(k-1) + vel(k-1) * dt
// 状态: [xc, yc, za, vxc, vyc, vza] (6D)
struct TranslationModel : auto_graph::MotionModel<TranslationModel, 6> {
    double dt;
    TranslationModel(double dt) : dt(dt) {}
    template<typename T>
    void operator()(const T& x, T& x_next) const {
        x_next[0] = x[0] + x[1] * dt;  // xc += vxc
        x_next[1] = x[1];              // vxc
        x_next[2] = x[2] + x[3] * dt;  // yc += vyc
        x_next[3] = x[3];              // vyc
        x_next[4] = x[4] + x[5] * dt;  // za += vza
        x_next[5] = x[5];              // vza
    }
};


// Yaw: yaw(k) = yaw(k-1) + vyaw(k-1) * dt (2D)
struct YawModel : auto_graph::MotionModel<YawModel, 2> {
    double dt;
    YawModel(double dt) : dt(dt) {}
    template<typename T>
    void operator()(const T& x, T& x_next) const {
        x_next[0] = x[0] + x[1] * dt;
        x_next[1] = x[1];
    }
};

/**
 * 11D 运动模型 (含 r1,r2,dz, 旧版兼容)
 */
struct ArmorCVMotionModel : auto_graph::MotionModel<ArmorCVMotionModel, 11> {
    double dt;
    ArmorCVMotionModel(double dt) : dt(dt) {}

    template<typename T>
    void operator()(const T& x, T& x_next) const {
        // 匀速模型
        x_next[0] = x[0] + x[1] * dt;  // xc = xc + v_xc * dt
        x_next[1] = x[1];               // v_xc = v_xc
        x_next[2] = x[2] + x[3] * dt;  // yc = yc + v_yc * dt
        x_next[3] = x[3];               // v_yc = v_yc
        x_next[4] = x[4] + x[5] * dt;  // za = za + v_za * dt
        x_next[5] = x[5];               // v_za = v_za
        x_next[6] = x[6] + x[7] * dt;  // yaw = yaw + v_yaw * dt
        x_next[7] = x[7];               // v_yaw = v_yaw
        x_next[8] = x[8];               // r1 = r1 (常值)
        x_next[9] = x[9];               // r2 = r2 (常值)
        x_next[10] = x[10];             // dz = dz (常值)
    }
};

/**
 * YPD观测模型（与cv_model::MeasureSingle相同）
 *
 * 观测向量：[yaw, pitch, distance, orient_yaw]
 */
struct ArmorCVMeasureYPD : auto_graph::MeasureModel<ArmorCVMeasureYPD, 11, 4> {
    int I;  // 装甲板索引

    ArmorCVMeasureYPD(int i) : I(i) {}

    template<typename T>
    void operator()(const std::vector<T>& x, std::vector<T>& z) const {
        // 计算装甲板3D位置
        // 使用 x[6] * 0.0 来创建与x[6]相同维度的零向量
        // 然后加上常数偏移
        T zero = x[6] * 0.0;
        T angle = x[6] + (zero + M_PI_2 * I);
        T cos_angle = ceres::cos(angle);
        T sin_angle = ceres::sin(angle);
        T r = x[8 + I % 2];
        T armor_x = x[0] - cos_angle * r;
        T armor_y = x[2] - sin_angle * r;
        T armor_z = x[4] + (I % 2 == 0 ? zero : x[10]);

        // 转换到球坐标
        z[0] = ceres::atan2(armor_y, armor_x);  // yaw
        z[1] = ceres::atan2(armor_z, ceres::sqrt(armor_x * armor_x + armor_y * armor_y));  // pitch
        z[2] = ceres::sqrt(armor_x * armor_x + armor_y * armor_y + armor_z * armor_z);  // distance
        z[3] = x[6] + (zero + M_PI_2 * I);  // orient_yaw
    }
};

// YPD 双板观测: 同时预测两块装甲板的 YPD (8D 观测, 参照 cv_model::MeasureDouble)
struct ArmorCVMeasureYPDDouble : auto_graph::MeasureModel<ArmorCVMeasureYPDDouble, 11, 8> {
    int I, J;

    ArmorCVMeasureYPDDouble(int i, int j) : I(i), J(j) {}

    template<typename T>
    void operator()(const std::vector<T>& x, std::vector<T>& z) const {
        int idx = 0;
        for (int k : {I, J}) {
            T zero = x[6] * T(0);
            T angle = x[6] + T(M_PI_2 * k);
            T cos_angle = ceres::cos(angle);
            T sin_angle = ceres::sin(angle);
            T r = x[8 + k % 2];
            T armor_x = x[0] - cos_angle * r;
            T armor_y = x[2] - sin_angle * r;
            T armor_z = x[4] + (k % 2 == 0 ? zero : x[10]);

            z[idx + 0] = ceres::atan2(armor_y, armor_x);  // yaw
            z[idx + 1] = ceres::atan2(armor_z, ceres::sqrt(armor_x * armor_x + armor_y * armor_y));  // pitch
            z[idx + 2] = ceres::sqrt(armor_x * armor_x + armor_y * armor_y + armor_z * armor_z);  // distance
            z[idx + 3] = x[6] + (zero + M_PI_2 * k);  // orient_yaw
            idx += 4;
        }
    }
};

inline Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R) {
    double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
    double roll = std::atan2(R(2, 1), R(2, 2));
    double yaw = std::atan2(R(1, 0), R(0, 0));
    return {roll, pitch, yaw};
}

inline constexpr double kGraphOptimizerRadiusMin = 0.10;
inline constexpr double kGraphOptimizerRadiusMax = 0.60;

inline double graphOptimizerRadiusFromState(double radius_u) {
    return auto_graph::logisticFunction(
        radius_u, kGraphOptimizerRadiusMin, kGraphOptimizerRadiusMax);
}

inline double graphOptimizerRadiusToState(double radius) {
    return auto_graph::logisticInverse(
        radius, kGraphOptimizerRadiusMin, kGraphOptimizerRadiusMax);
}

}  // namespace filter_test

// ═══════════════════════════════════════════════════
// ArmorReprojFactor — 单角点重投影因子 (参考 jlu_vision_26)
//   1-key: armor Pose3 (camera 系)
//   使用 GTSAM 内置 PinholeCamera<Cal3DS2> 做投影+畸变
//   每装甲板 4 个此因子 (每个角点一个)
// ═══════════════════════════════════════════════════
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

struct ArmorReprojFactor : gtsam::NoiseModelFactorN<gtsam::Pose3> {
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;
    using Pose3 = gtsam::Pose3;
    using Point3 = gtsam::Point3;
    using Point2 = gtsam::Point2;

    Point3 armor_point_;
    gtsam::Cal3DS2 calib_;
    Point2 px_obs_;

    ArmorReprojFactor(gtsam::Key key, const gtsam::SharedNoiseModel& noise,
                      const Eigen::Vector3d& corner_local,
                      const Eigen::Matrix3d& K,
                      const std::array<double, 5>& dist,
                      const Eigen::Vector2d& observed_pixel)
        : Base(noise, key),
          armor_point_(corner_local.x(), corner_local.y(), corner_local.z()),
          calib_(K(0,0), K(1,1), K(0,1), K(0,2), K(1,2),
                  dist[0], dist[1], dist[2], dist[3]),
          px_obs_(observed_pixel.x(), observed_pixel.y()) {}

    gtsam::Vector evaluateError(const Pose3& armor_pose_camera,
                                gtsam::Matrix* H) const override {
        gtsam::Matrix36 H_transform;
        Point3 p_cam = armor_pose_camera.transformFrom(
            armor_point_, H ? &H_transform : nullptr, nullptr);
        gtsam::Matrix23 H_norm;
        Point2 pn = gtsam::PinholeCamera<gtsam::Cal3DS2>::Project(
            p_cam, H ? &H_norm : nullptr);
        gtsam::Matrix22 H_calib;
        Point2 px = calib_.uncalibrate(pn, {}, H ? &H_calib : nullptr);

        if (H) *H = H_calib * H_norm * H_transform;
        return px - px_obs_;
    }
};

// ═══════════════════════════════════════════════════
// ArmorCenterFactor — 几何约束因子 (参考 jlu_vision_26)
//   armor Pose3(camera系) + center_pos + center_yaw + radius + dz
//   → 4D 误差 [切向, 径向, z, yaw_err]
//
// 内部: Pose3 camera→odom 变换后提取 yaw 和位置做几何约束
// ═══════════════════════════════════════════════════
struct ArmorCenterFactor : gtsam::NoiseModelFactor5<
    gtsam::Pose3, gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> {

    using Base = gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector>;
    int armor_index_;
    double radius_min_, radius_max_;
    Eigen::Isometry3d T_camera_to_odom_;

    ArmorCenterFactor(gtsam::Key k_armor, gtsam::Key k_pos, gtsam::Key k_yaw_vyaw,
                      gtsam::Key k_radius, gtsam::Key k_dz,
                      const gtsam::SharedNoiseModel& noise, int idx,
                      const Eigen::Isometry3d& T_c2o,
                      double r_min = 0.15, double r_max = 0.40)
        : Base(noise, k_armor, k_pos, k_yaw_vyaw, k_radius, k_dz),
          armor_index_(idx), radius_min_(r_min), radius_max_(r_max),
          T_camera_to_odom_(T_c2o) {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3& armor_pose_camera,
        const gtsam::Vector& pos_vel,     // 6D: [xc, vxc, yc, vyc, za, vza]
        const gtsam::Vector& yaw_vyaw,    // 2D: [yaw, vyaw]
        const gtsam::Vector& radius_vec,  // 2D: [r1_u, r2_u]
        const gtsam::Vector& dz_vec,      // 1D: [dz]
        gtsam::Matrix* H1, gtsam::Matrix* H2, gtsam::Matrix* H3,
        gtsam::Matrix* H4, gtsam::Matrix* H5) const override {

        // camera → odom
        Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
        pose.pretranslate(armor_pose_camera.translation());
        pose.rotate(armor_pose_camera.rotation().matrix());
        Eigen::Isometry3d armor_pose_odom = T_camera_to_odom_ * pose;

        Eigen::Vector3d armor_position = armor_pose_odom.translation();
        Eigen::Vector3d rpy = filter_test::rotationMatrixToRPY(armor_pose_odom.rotation().matrix());
        double armor_yaw = rpy.z();

        // logistic 半径
        double r_u = radius_vec[armor_index_ % 2];
        double r = auto_graph::logisticFunction(r_u, radius_min_, radius_max_);
        double dlog = auto_graph::logisticDerivative(r, radius_min_, radius_max_);

        // 使用 armor_yaw 定义切向/径向 (参考 jlu_vision_26)
        double nx = std::cos(armor_yaw), ny = std::sin(armor_yaw);
        double tx = -ny, ty = nx;

        double center_x = pos_vel[0], center_y = pos_vel[2], center_z = pos_vel[4];
        double center_yaw = yaw_vyaw[0];
        double dz = (armor_index_ % 2 == 0) ? 0.0 : dz_vec[0];

        double dx = center_x - armor_position.x();
        double dy = center_y - armor_position.y();

        double tangential_err = tx * dx + ty * dy;
        double radial_err = nx * dx + ny * dy - r;
        double z_err = center_z - armor_position.z() + dz;
        // yaw_err: 使用角度包裹 (对齐 jlu Rot2::localCoordinates)
        double pred_armor_yaw = center_yaw + armor_index_ * M_PI_2;
        double yaw_err = std::remainder(armor_yaw - pred_armor_yaw, 2.0 * M_PI);

        gtsam::Vector4 err(tangential_err, radial_err, z_err, yaw_err);

        // Jacobians (参考 jlu_vision_26 ArmorRadiusCenterZFactor/DZFactor)
        if (H1) {
            H1->setZero(4, 6);
            Eigen::Matrix<double, 4, 3> J_p;
            J_p << -tx, -ty, 0.0,
                   -nx, -ny, 0.0,
                    0.0, 0.0, -1.0,
                    0.0, 0.0, 0.0;
            // d(err)/d(armor_yaw angle psi)
            double radial_proj = nx * dx + ny * dy;
            Eigen::Matrix<double, 4, 1> d_e_d_psi;
            d_e_d_psi << -radial_proj, tangential_err, 0.0, 1.0;
            // d(psi)/d(omega) — right-multiply perturbation: body-frame omega → yaw change
            double roll = rpy.x(), pitch = rpy.y();
            double cos_pitch = std::cos(pitch);
            Eigen::Matrix<double, 1, 3> d_psi_d_omega;
            d_psi_d_omega << 0.0, std::sin(roll) / cos_pitch, std::cos(roll) / cos_pitch;
            Eigen::Matrix<double, 4, 3> J_rot = d_e_d_psi * d_psi_d_omega;
            Eigen::Matrix<double, 4, 3> J_trans = J_p * armor_pose_odom.rotation().matrix();
            H1->leftCols<3>() = J_rot;
            H1->rightCols<3>() = J_trans;
        }
        if (H2) {
            H2->setZero(4, 6);  // pos_vel is 6D: [xc, vxc, yc, vyc, za, vza]
            (*H2)(0, 0) =  tx;   // tangential / d(xc)
            (*H2)(0, 2) =  ty;   // tangential / d(yc)
            (*H2)(1, 0) =  nx;   // radial / d(xc)
            (*H2)(1, 2) =  ny;   // radial / d(yc)
            (*H2)(2, 4) =  1.0;  // z / d(za)
        }
        if (H3) {
            H3->setZero(4, 2);
            // center_yaw 只直接影响 yaw_err
            (*H3)(3, 0) = -1.0;
        }
        if (H4) {
            H4->setZero(4, 2);
            (*H4)(1, armor_index_ % 2) = -dlog;  // radial / d(r_u), through logistic
        }
        if (H5) {
            H5->setZero(4, 1);
            if (armor_index_ % 2 == 1) (*H5)(2, 0) = 1.0;  // z / d(dz)
        }
        return err;
    }
};

// ═══════════════════════════════════════════════════
// VelSmoothFactor — 速度平滑因子 (参考 jlu_vision_26 VelocityFactor)
//   2-key: pos_vel(k-1) → pos_vel(k)
//   仅约束速度分量 [vxc, vyc, vza] 的帧间变化
//   解决 CWNA 运动模型下速度分量欠约束的问题
// ═══════════════════════════════════════════════════
struct VelSmoothFactor : gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {
    using Base = gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector>;

    VelSmoothFactor(gtsam::Key k_prev, gtsam::Key k_cur,
                    const gtsam::SharedNoiseModel& noise)
        : Base(noise, k_prev, k_cur) {}

    gtsam::Vector evaluateError(
        const gtsam::Vector& x1, const gtsam::Vector& x2,
        gtsam::Matrix* H1, gtsam::Matrix* H2) const override {
        // pos_vel = 6D: [xc, vxc, yc, vyc, za, vza]
        // error = 3D: [Δvxc, Δvyc, Δvza]
        gtsam::Vector3 err;
        err(0) = x2[1] - x1[1];
        err(1) = x2[3] - x1[3];
        err(2) = x2[5] - x1[5];

        if (H1) {
            H1->setZero(3, 6);
            (*H1)(0, 1) = -1.0;
            (*H1)(1, 3) = -1.0;
            (*H1)(2, 5) = -1.0;
        }
        if (H2) {
            H2->setZero(3, 6);
            (*H2)(0, 1) = 1.0;
            (*H2)(1, 3) = 1.0;
            (*H2)(2, 5) = 1.0;
        }
        return err;
    }
};

// ═══════════════════════════════════════════════════
// VyawSmoothFactor — yaw 速度平滑因子 (参考 jlu_vision_26 VyawFactor)
//   2-key: yaw_vyaw(k-1) → yaw_vyaw(k)
//   仅约束 vyaw 的帧间变化
// ═══════════════════════════════════════════════════
struct VyawSmoothFactor : gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {
    using Base = gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector>;

    VyawSmoothFactor(gtsam::Key k_prev, gtsam::Key k_cur,
                     const gtsam::SharedNoiseModel& noise)
        : Base(noise, k_prev, k_cur) {}

    gtsam::Vector evaluateError(
        const gtsam::Vector& x1, const gtsam::Vector& x2,
        gtsam::Matrix* H1, gtsam::Matrix* H2) const override {
        // yaw_vyaw = 2D: [yaw, vyaw]
        // error = 1D: Δvyaw
        gtsam::Vector1 err;
        err(0) = x2[1] - x1[1];

        if (H1) {
            H1->setZero(1, 2);
            (*H1)(0, 1) = -1.0;
        }
        if (H2) {
            H2->setZero(1, 2);
            (*H2)(0, 1) = 1.0;
        }
        return err;
    }
};

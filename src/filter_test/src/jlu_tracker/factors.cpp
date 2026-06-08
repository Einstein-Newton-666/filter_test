// Adapted from jlu_vision_26 auto_aim tracker factors
#include "filter_test/jlu_tracker/factors.hpp"
#include "filter_test/graph_optimizer/graph_math.hpp"

#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <cmath>
#include <vector>

namespace jlu {

// ═══════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════

// 装甲板之间等距分布: index 0→0°, 1→90°, 2→180°, 3→270°
inline double getArmorBetweenYawFromIndex(ArmorIndex index, int armor_numbers = 4) {
  return static_cast<int>(index) * (M_PI * 2.0 / armor_numbers);
}

// ZYX 欧拉角: R = Rz(yaw)*Ry(pitch)*Rx(roll), pitch=-R(2,0)
inline Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R) {
  double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
  double roll  = std::atan2(R(2, 1), R(2, 2));
  double yaw   = std::atan2(R(1, 0), R(0, 0));
  return {roll, pitch, yaw};
}

// ── Armor points (X=normal, Y=width, Z=height) ──
// 对齐 armor_geometry.hpp: SMALL_ARMOR_HEIGHT=0.125, half=0.0625
static const std::vector<std::array<float, 3>> kSmallArmorPointsCV = {
    {0.0f, -0.0675f, -0.0625f},  // LeftBottom
    {0.0f, -0.0675f,  0.0625f},  // LeftTop
    {0.0f,  0.0675f,  0.0625f},  // RightTop
    {0.0f,  0.0675f, -0.0625f},  // RightBottom
};

// ═══════════════════════════════════════════════════════════════════════
// TranslationFactor — 3-key 位置预测 (CWNA 运动模型)
//   X(k-1) + V(k-1)*dt → X(k)
//   误差: X(k) - X(k-1) - V(k-1)*dt  (3D)
//   噪声: σ=0.001m (位置预测精度 ~1mm, 提供强正则化)
//   雅可比: H1=-I, H2=-dt*I, H3=I
// ═══════════════════════════════════════════════════════════════════════
TranslationFactor::TranslationFactor(const gtsam::SharedNoiseModel& model,
                                     gtsam::Key x_pre, gtsam::Key v_pre,
                                     gtsam::Key x_cur, double dt)
    : Base(model, x_pre, v_pre, x_cur), dt_(dt) {}

gtsam::Vector TranslationFactor::evaluateError(
    const gtsam::Point3& x_pre, const gtsam::Vector3& v_pre,
    const gtsam::Point3& x_cur, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const {
  gtsam::Vector3 error = x_cur - (x_pre + v_pre * dt_);
  if (H1) *H1 = -gtsam::Matrix3::Identity();
  if (H2) *H2 = -dt_ * gtsam::Matrix3::Identity();
  if (H3) *H3 = gtsam::Matrix3::Identity();
  return error;
}

// ═══════════════════════════════════════════════════════════════════════
// YawFactor — 3-key 朝向预测 (CWNA 角运动)
//   R(k-1) * Rot2(W(k-1)*dt) → R(k)
//   误差: (R(k-1) * Rot2(W*dt)).localCoordinates(R(k))  (1D, SO(2) 包裹)
//   噪声: σ=0.005rad
//   雅可比: H1=-1, H2=-dt, H3=1
// ═══════════════════════════════════════════════════════════════════════
YawFactor::YawFactor(const gtsam::SharedNoiseModel& model, gtsam::Key r_pre,
                     gtsam::Key w_pre, gtsam::Key r_cur, double dt)
    : Base(model, r_pre, w_pre, r_cur), dt_(dt) {}

gtsam::Vector YawFactor::evaluateError(const gtsam::Rot2& r_pre,
                                        const double& w_pre,
                                        const gtsam::Rot2& r_cur,
                                        gtsam::OptionalMatrixType H1,
                                        gtsam::OptionalMatrixType H2,
                                        gtsam::OptionalMatrixType H3) const {
  gtsam::Vector1 error =
      (r_pre * gtsam::Rot2::fromAngle(w_pre * dt_)).localCoordinates(r_cur);
  if (H1) *H1 = gtsam::Matrix::Constant(1, 1, -1.0);
  if (H2) *H2 = gtsam::Matrix::Constant(1, 1, -dt_);
  if (H3) *H3 = gtsam::Matrix::Identity(1, 1);
  return error;
}

// ═══════════════════════════════════════════════════════════════════════
// VelocityFactor — 2-key 速度平滑 (防止速度欠约束)
//   V(k-1) → V(k)
//   误差: V(k) - V(k-1)  (3D)
//   噪声: σ=0.01m/s (帧间速度变化 ~0.03m/s@3σ)
//
// 这是 jlu 收敛的关键: 观测只约束位置不约束速度,
// VelocityFactor 提供独立的速度正则化链, 保证 ISAM2 信息矩阵满秩.
// ═══════════════════════════════════════════════════════════════════════
VelocityFactor::VelocityFactor(const gtsam::SharedNoiseModel& model,
                               gtsam::Key v_pre, gtsam::Key v_cur)
    : Base(model, v_pre, v_cur) {}

gtsam::Vector VelocityFactor::evaluateError(const gtsam::Vector3& v_pre,
                                             const gtsam::Vector3& v_cur,
                                             gtsam::OptionalMatrixType H1,
                                             gtsam::OptionalMatrixType H2) const {
  gtsam::Vector3 error = v_cur - v_pre;
  if (H1) *H1 = -gtsam::Matrix3::Identity();
  if (H2) *H2 = gtsam::Matrix3::Identity();
  return error;
}

// ═══════════════════════════════════════════════════════════════════════
// VyawFactor — 2-key 角速度平滑
//   W(k-1) → W(k)
//   误差: W(k) - W(k-1)  (1D)
//   噪声: σ=0.05rad/s
// ═══════════════════════════════════════════════════════════════════════
VyawFactor::VyawFactor(const gtsam::SharedNoiseModel& model,
                       gtsam::Key w_pre, gtsam::Key w_cur)
    : Base(model, w_pre, w_cur) {}

gtsam::Vector VyawFactor::evaluateError(const double& w_pre,
                                         const double& w_cur,
                                         gtsam::OptionalMatrixType H1,
                                         gtsam::OptionalMatrixType H2) const {
  gtsam::Vector1 error{w_cur - w_pre};
  if (H1) *H1 = -gtsam::Matrix1::Identity();
  if (H2) *H2 = gtsam::Matrix1::Identity();
  return error;
}

// ═══════════════════════════════════════════════════════════════════════
// ArmorRadiusCenterZFactor — 几何约束 (armors 0,2: r1 装甲板)
//   4-key: armor_pose(Pose3) + radius(double) + center_yaw(Rot2) + center_pos(Point3)
//   4D 误差: [切向, 径向, z, yaw_err]
//
// 坐标系: camera系 armor_pose → T_camera_to_odom → odom系
// yaw 误差使用 Rot2::localCoordinates (SO(2) 流形包裹)
// radius 通过 logistic 映射到 [radius_min, radius_max]
// ═══════════════════════════════════════════════════════════════════════
ArmorRadiusCenterZFactor::ArmorRadiusCenterZFactor(
    const gtsam::SharedNoiseModel& model, gtsam::Key armor_pose_key,
    gtsam::Key radius_key, gtsam::Key center_yaw_key,
    gtsam::Key center_point_key, const Eigen::Isometry3d& T_camera_to_odom,
    ArmorIndex armor_index, double radius_min, double radius_max)
    : Base(model, armor_pose_key, radius_key, center_yaw_key, center_point_key),
      T_camera_to_odom_(T_camera_to_odom), armor_index_(armor_index),
      radius_min_(radius_min), radius_max_(radius_max) {}

gtsam::Vector ArmorRadiusCenterZFactor::evaluateError(
    const gtsam::Pose3& armor_pose_camera, const double& radius,
    const gtsam::Rot2& center_yaw, const gtsam::Point3& center_point,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
    gtsam::OptionalMatrixType H3, gtsam::OptionalMatrixType H4) const {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  pose.pretranslate(armor_pose_camera.translation());
  pose.rotate(armor_pose_camera.rotation().matrix());
  Eigen::Isometry3d armor_pose_odom = T_camera_to_odom_ * pose;
  auto armor_yaw = gtsam::Rot2::fromAngle(
      rotationMatrixToRPY(armor_pose_odom.rotation().matrix()).z());
  Eigen::Vector3d armor_position = armor_pose_odom.translation();
  auto radius_b = auto_graph::logisticFunction(radius, radius_min_, radius_max_);
  auto nx = std::cos(armor_yaw.theta());
  auto ny = std::sin(armor_yaw.theta());
  auto tx = -ny;
  auto ty = nx;
  auto dx = center_point.x() - armor_position.x();
  auto dy = center_point.y() - armor_position.y();
  auto tangential_err = tx * dx + ty * dy;
  auto radial_err = nx * dx + ny * dy - radius_b;
  auto z_err = center_point.z() - armor_position.z();
  auto pred_armor_yaw = gtsam::Rot2::fromAngle(
      center_yaw.theta() + getArmorBetweenYawFromIndex(armor_index_));
  auto yaw_err = armor_yaw.localCoordinates(pred_armor_yaw).x();
  gtsam::Vector4 error{tangential_err, radial_err, z_err, yaw_err};
  if (H1) {
    Eigen::Matrix<double, 4, 3> J_p;
    J_p << -tx, -ty, 0.0, -nx, -ny, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0;
    auto radial_proj = nx * dx + ny * dy;
    Eigen::Matrix<double, 4, 1> d_e_d_psi;
    d_e_d_psi << -radial_proj, tangential_err, 0.0, -1.0;
    Eigen::Vector3d rpy = rotationMatrixToRPY(armor_pose_odom.rotation().matrix());
    auto roll = rpy.x();
    auto pitch = rpy.y();
    auto cos_pitch = std::cos(pitch);
    Eigen::Matrix<double, 1, 3> d_psi_d_omega;
    d_psi_d_omega << 0.0, std::sin(roll) / cos_pitch, std::cos(roll) / cos_pitch;
    Eigen::Matrix<double, 4, 3> J_rot = d_e_d_psi * d_psi_d_omega;
    Eigen::Matrix<double, 4, 3> J_trans =
        J_p * armor_pose_odom.rotation().matrix();
    Eigen::Matrix<double, 4, 6> H1_mat;
    H1_mat.leftCols<3>() = J_rot;
    H1_mat.rightCols<3>() = J_trans;
    (*H1) = H1_mat;
  }
  if (H2) {
    double d_radius_d_radius =
        auto_graph::logisticDerivative(radius_b, radius_min_, radius_max_);
    (*H2) = (gtsam::Matrix(4, 1) << 0.0, -d_radius_d_radius, 0.0, 0.0).finished();
  }
  if (H3) {
    (*H3) = (gtsam::Matrix(4, 1) << 0.0, 0.0, 0.0, 1.0).finished();
  }
  if (H4) {
    (*H4) = (gtsam::Matrix(4, 3) << tx, ty, 0.0, nx, ny, 0.0, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0)
                .finished();
  }
  return error;
}

// ═══════════════════════════════════════════════════════════════════════
// ArmorRadiusDZFactor — 几何约束 (armors 1,3: r2+dz 装甲板)
//   5-key: armor_pose + radius + dz + center_yaw + center_pos
//   4D 误差: [切向, 径向, z, yaw_err]
//   与 ArmorRadiusCenterZFactor 的区别: z 误差包含 dz 项
// ═══════════════════════════════════════════════════════════════════════
ArmorRadiusDZFactor::ArmorRadiusDZFactor(
    const gtsam::SharedNoiseModel& model, gtsam::Key armor_pose_key,
    gtsam::Key radius_key, gtsam::Key dz_key, gtsam::Key center_yaw_key,
    gtsam::Key center_point_key, const Eigen::Isometry3d& T_camera_to_odom,
    ArmorIndex armor_index, double radius_min, double radius_max, int armor_numbers)
    : Base(model, armor_pose_key, radius_key, dz_key, center_yaw_key,
           center_point_key),
      T_camera_to_odom_(T_camera_to_odom), armor_index_(armor_index),
      radius_min_(radius_min), radius_max_(radius_max),
      armor_numbers_(armor_numbers) {}

gtsam::Vector ArmorRadiusDZFactor::evaluateError(
    const gtsam::Pose3& armor_pose_camera, const double& radius,
    const double& dz, const gtsam::Rot2& center_yaw,
    const gtsam::Point3& center_point, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
    gtsam::OptionalMatrixType H4, gtsam::OptionalMatrixType H5) const {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  pose.pretranslate(armor_pose_camera.translation());
  pose.rotate(armor_pose_camera.rotation().matrix());
  Eigen::Isometry3d armor_pose_odom = T_camera_to_odom_ * pose;
  auto armor_yaw = gtsam::Rot2::fromAngle(
      rotationMatrixToRPY(armor_pose_odom.rotation().matrix()).z());
  Eigen::Vector3d armor_position = armor_pose_odom.translation();
  auto radius_b = auto_graph::logisticFunction(radius, radius_min_, radius_max_);
  auto nx = std::cos(armor_yaw.theta());
  auto ny = std::sin(armor_yaw.theta());
  auto tx = -ny;
  auto ty = nx;
  auto dx = center_point.x() - armor_position.x();
  auto dy = center_point.y() - armor_position.y();
  auto tangential_err = tx * dx + ty * dy;
  auto radial_err = nx * dx + ny * dy - radius_b;
  auto z_err = center_point.z() + dz - armor_position.z();
  auto pred_armor_yaw = gtsam::Rot2::fromAngle(
      center_yaw.theta() +
      getArmorBetweenYawFromIndex(armor_index_, armor_numbers_));
  auto yaw_err = armor_yaw.localCoordinates(pred_armor_yaw).x();
  gtsam::Vector4 error{tangential_err, radial_err, z_err, yaw_err};
  if (H1) {
    Eigen::Matrix<double, 4, 3> J_p;
    J_p << -tx, -ty, 0.0, -nx, -ny, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0;
    auto radial_proj = nx * dx + ny * dy;
    Eigen::Matrix<double, 4, 1> d_e_d_psi;
    d_e_d_psi << -radial_proj, tangential_err, 0.0, -1.0;
    Eigen::Vector3d rpy = rotationMatrixToRPY(armor_pose_odom.rotation().matrix());
    auto roll = rpy.x();
    auto pitch = rpy.y();
    auto cos_pitch = std::cos(pitch);
    Eigen::Matrix<double, 1, 3> d_psi_d_omega;
    d_psi_d_omega << 0.0, std::sin(roll) / cos_pitch, std::cos(roll) / cos_pitch;
    Eigen::Matrix<double, 4, 3> J_rot = d_e_d_psi * d_psi_d_omega;
    Eigen::Matrix<double, 4, 3> J_trans =
        J_p * armor_pose_odom.rotation().matrix();
    Eigen::Matrix<double, 4, 6> H1_mat;
    H1_mat.leftCols<3>() = J_rot;
    H1_mat.rightCols<3>() = J_trans;
    (*H1) = H1_mat;
  }
  if (H2) {
    auto d_radius_d_rb =
        auto_graph::logisticDerivative(radius_b, radius_min_, radius_max_);
    (*H2) = (gtsam::Matrix(4, 1) << 0.0, -d_radius_d_rb, 0.0, 0.0).finished();
  }
  if (H3) {
    (*H3) = (gtsam::Matrix(4, 1) << 0.0, 0.0, 1.0, 0.0).finished();
  }
  if (H4) {
    (*H4) = (gtsam::Matrix(4, 1) << 0.0, 0.0, 0.0, 1.0).finished();
  }
  if (H5) {
    (*H5) = (gtsam::Matrix(4, 3) << tx, ty, 0.0, nx, ny, 0.0, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0)
                .finished();
  }
  return error;
}

// ═══════════════════════════════════════════════════════════════════════
// ArmorReprojFactor — 1-key 角点重投影 (第一级像素观测)
//   误差: PinholeCamera::Project(armor_pose * corner_local) - pixel_observed  (2D)
//   雅可比链: Cal3DS2 → PinholeCamera → Pose3.transformFrom
//   每个装甲板 4 个此因子 (4角点 × 2D = 8 约束, 加上 Pose3 先验 6D = 14 约束 → 充分)
// ═══════════════════════════════════════════════════════════════════════
ArmorReprojFactor::ArmorReprojFactor(
    const gtsam::SharedNoiseModel& model, gtsam::Key armor_pose_key,
    const cv::Mat& camera_matrix, const cv::Mat& distortion_coefficients,
    ArmorType armor_type, ArmorPointPosition point_position,
    Eigen::Vector2d px_point)
    : Base(model, armor_pose_key), px_point_(px_point) {
  auto& pt = kSmallArmorPointsCV.at(static_cast<int>(point_position));
  armor_point_ = gtsam::Point3{pt[0], pt[1], pt[2]};
  double fx = camera_matrix.at<double>(0, 0);
  double fy = camera_matrix.at<double>(1, 1);
  double s  = camera_matrix.at<double>(0, 1);
  double u0 = camera_matrix.at<double>(0, 2);
  double v0 = camera_matrix.at<double>(1, 2);
  double k1 = distortion_coefficients.at<double>(0, 0);
  double k2 = distortion_coefficients.at<double>(0, 1);
  double p1 = distortion_coefficients.at<double>(0, 2);
  double p2 = distortion_coefficients.at<double>(0, 3);
  calib_ = gtsam::Cal3DS2(fx, fy, s, u0, v0, k1, k2, p1, p2);
}

gtsam::Vector ArmorReprojFactor::evaluateError(
    const gtsam::Pose3& armor_pose_camera, gtsam::OptionalMatrixType H) const {
  gtsam::Matrix36 H_transform;
  gtsam::Point3 p_cam = armor_pose_camera.transformFrom(
      armor_point_, H ? &H_transform : nullptr, nullptr);
  gtsam::Matrix23 H_norm;
  gtsam::Point2 pn = gtsam::PinholeCamera<gtsam::Cal3DS2>::Project(
      p_cam, H ? &H_norm : nullptr);
  gtsam::Matrix22 H_calib;
  gtsam::Point2 px = calib_.uncalibrate(pn, {}, H ? &H_calib : nullptr);
  if (H) *H = H_calib * H_norm * H_transform;
  return px - px_point_;
}

}  // namespace jlu

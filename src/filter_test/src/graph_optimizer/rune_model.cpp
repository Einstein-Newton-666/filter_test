#include "filter_test/graph_optimizer/rune_model.hpp"

#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

namespace filter_test::graph_optimizer {
namespace {

Eigen::Vector3d rotationMatrixToRpyLocal(const Eigen::Matrix3d& R) {
    return Eigen::Vector3d(
        std::atan2(R(2, 1), R(2, 2)),
        std::asin(std::clamp(-R(2, 0), -1.0, 1.0)),
        std::atan2(R(1, 0), R(0, 0)));
}

Eigen::Matrix3d runeBaseRotation(double normal_yaw, double normal_pitch) {
    // 和仿真器保持同一旋转顺序：Rz(normal_yaw) * Ry(normal_pitch) * Rx(roll)。
    // normal_pitch 是固定参数，用于预测辅助 Pose3 和输出击打点。
    return (Eigen::AngleAxisd(normal_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(normal_pitch, Eigen::Vector3d::UnitY()))
        .toRotationMatrix();
}

Eigen::Isometry3d pose3ToEigen(const gtsam::Pose3& pose) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = pose.rotation().matrix();
    T.translation() = auto_graph::point3ToEigen(pose.translation());
    return T;
}

gtsam::Pose3 eigenToPose3(const Eigen::Isometry3d& T) {
    return gtsam::Pose3(gtsam::Rot3(T.rotation()),
                        gtsam::Point3(T.translation().x(),
                                      T.translation().y(),
                                      T.translation().z()));
}

gtsam::Key bladePoseKey(uint64_t frame_id, int blade_index) {
    // 每帧每片扇叶一个辅助 Pose3 key。前缀不能和主状态变量冲突。
    static constexpr std::array<char, 5> prefixes{'q', 's', 't', 'u', 'y'};
    return gtsam::Symbol(prefixes.at(static_cast<size_t>(blade_index)), frame_id);
}

std::array<Eigen::Vector2d, 5> targetPixels(
    const auto_aim_interfaces::msg::RuneTarget& target) {
    // 消息顺序必须和 runeBladeLocalPoints() 对齐，否则重投影残差会把点绑错。
    return {
        Eigen::Vector2d(target.r_center.x, target.r_center.y),
        Eigen::Vector2d(target.near_point.x, target.near_point.y),
        Eigen::Vector2d(target.left_point.x, target.left_point.y),
        Eigen::Vector2d(target.far_point.x, target.far_point.y),
        Eigen::Vector2d(target.right_point.x, target.right_point.y),
    };
}

bool finitePixel(const Eigen::Vector2d& px) {
    return std::isfinite(px.x()) && std::isfinite(px.y()) &&
           px.x() >= 0.0 && px.y() >= 0.0;
}

Eigen::Vector2d projectPoint(
    const Eigen::Vector3d& point_camera,
    const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist) {
    const double z = std::abs(point_camera.z()) > 1e-9 ? point_camera.z() : 1e-9;
    double x = point_camera.x() / z;
    double y = point_camera.y() / z;
    const double r2 = x * x + y * y;
    const double radial = 1.0 + dist[0] * r2 + dist[1] * r2 * r2 + dist[4] * r2 * r2 * r2;
    const double x_distorted = x * radial + 2.0 * dist[2] * x * y + dist[3] * (r2 + 2.0 * x * x);
    const double y_distorted = y * radial + dist[2] * (r2 + 2.0 * y * y) + 2.0 * dist[3] * x * y;
    return Eigen::Vector2d(
        K(0, 0) * x_distorted + K(0, 1) * y_distorted + K(0, 2),
        K(1, 1) * y_distorted + K(1, 2));
}

double lineAngle(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    const Eigen::Vector2d d = b - a;
    return std::atan2(d.y(), d.x());
}

double lineLength(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return std::max((b - a).norm(), 1e-6);
}

template<typename ErrorFunc>
gtsam::Matrix finiteDiffColumns(int rows, int cols, ErrorFunc&& func) {
    constexpr double eps = 1e-6;
    const gtsam::Vector base = func(gtsam::Vector::Zero(cols));
    gtsam::Matrix H(rows, cols);
    for (int col = 0; col < cols; ++col) {
        gtsam::Vector delta = gtsam::Vector::Zero(cols);
        delta(col) = eps;
        H.col(col) = (func(delta) - base) / eps;
    }
    return H;
}

struct BigCurveResidual {
    BigCurveResidual(double time, double angle) : time_(time), angle_(angle) {}

    template<typename T>
    bool operator()(const T* const params, T* residual) const {
        const T a = params[0];
        const T w = params[1];
        const T t0 = params[2];
        const T b = params[3];
        const T c = params[4];
        residual[0] = -a * ceres::cos(w * (T(time_) + t0)) + b * T(time_) + c - T(angle_);
        return true;
    }

    double time_;
    double angle_;
};

struct RuneObservation {
    size_t target_index = 0;
    std::array<Eigen::Vector2d, 5> pixels;
    RunePnPResult pnp;
    Eigen::Isometry3d pose_odom = Eigen::Isometry3d::Identity();
    Eigen::Vector3d center_odom = Eigen::Vector3d::Zero();
    double blade_roll = 0.0;
};

struct RuneMatchedObservation {
    RuneObservation observation;
    int blade_index = 0;
    gtsam::Pose3 pose_camera;
    bool pnp_success = false;
};

struct RuneMatchCandidate {
    size_t observation_index = 0;
    int blade_index = 0;
    double primary_score = 0.0;
    double secondary_score = 0.0;
    bool pnp_success = false;
};

double bladeRollInRuneFrame(
    const Eigen::Matrix3d& blade_rotation_odom,
    double normal_yaw,
    double normal_pitch) {
    const Eigen::Matrix3d R_rel =
        runeBaseRotation(normal_yaw, normal_pitch).transpose() * blade_rotation_odom;
    return std::atan2(R_rel(2, 1), R_rel(2, 2));
}

gtsam::Vector runeDirectRollLoss(
    const Eigen::Vector3d& center,
    double normal_yaw,
    double normal_pitch,
    double roll,
    int blade_index,
    const Eigen::Isometry3d& T_camera_to_odom,
    const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist,
    const std::array<Eigen::Vector2d, 5>& observed_pixels);

double directRuneReprojError(
    const Eigen::Vector3d& center,
    double normal_yaw,
    double normal_pitch,
    double roll,
    int blade_index,
    const Eigen::Isometry3d& T_camera_to_odom,
    const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist,
    const std::array<Eigen::Vector2d, 5>& observed_pixels) {
    return runeDirectRollLoss(
        center, normal_yaw, normal_pitch, roll, blade_index,
        T_camera_to_odom, K, dist, observed_pixels).norm();
}

gtsam::Vector runeDirectRollLoss(
    const Eigen::Vector3d& center,
    double normal_yaw,
    double normal_pitch,
    double roll,
    int blade_index,
    const Eigen::Isometry3d& T_camera_to_odom,
    const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist,
    const std::array<Eigen::Vector2d, 5>& observed_pixels) {
    const auto projected = projectRuneBladePixels(
        center, normal_yaw, normal_pitch,
        roll + static_cast<double>(blade_index) * kRuneSlotAngle,
        T_camera_to_odom, K, dist);

    const double proj_angle_1 = lineAngle(projected[1], projected[3]);
    const double obs_angle_1 = lineAngle(observed_pixels[1], observed_pixels[3]);
    const double proj_angle_2 = lineAngle(projected[2], projected[4]);
    const double obs_angle_2 = lineAngle(observed_pixels[2], observed_pixels[4]);
    const double proj_len_1 = lineLength(projected[1], projected[3]);
    const double obs_len_1 = lineLength(observed_pixels[1], observed_pixels[3]);
    const double proj_len_2 = lineLength(projected[2], projected[4]);
    const double obs_len_2 = lineLength(observed_pixels[2], observed_pixels[4]);

    gtsam::Vector4 error;
    error << auto_graph::shortestAngularDistance(obs_angle_1, proj_angle_1),
        auto_graph::shortestAngularDistance(obs_angle_2, proj_angle_2),
        (proj_len_1 - obs_len_1) / obs_len_1,
        (proj_len_2 - obs_len_2) / obs_len_2;
    return error;
}

int positiveModulo5(int value) {
    const int mod = value % 5;
    return mod < 0 ? mod + 5 : mod;
}

}  // namespace

Eigen::Matrix3d runeBladeRotation(double normal_yaw, double normal_pitch, double roll) {
    return runeBaseRotation(normal_yaw, normal_pitch) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Vector3d runeTargetPosition(const Eigen::Vector3d& center,
                                   double normal_yaw,
                                   double normal_pitch,
                                   double roll) {
    return center + runeBladeRotation(normal_yaw, normal_pitch, roll) *
        Eigen::Vector3d(0.0, 0.0, kRuneBladeRadius);
}

std::array<Eigen::Vector2d, 5> projectRuneBladePixels(
    const Eigen::Vector3d& center,
    double normal_yaw,
    double normal_pitch,
    double roll,
    const Eigen::Isometry3d& T_camera_to_odom,
    const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist) {
    const auto local_points = runeBladeLocalPoints();
    const Eigen::Matrix3d R = runeBladeRotation(normal_yaw, normal_pitch, roll);
    const Eigen::Isometry3d T_odom_to_camera = T_camera_to_odom.inverse();
    std::array<Eigen::Vector2d, 5> pixels;
    for (size_t i = 0; i < local_points.size(); ++i) {
        const Eigen::Vector3d point_odom = center + R * local_points[i];
        pixels[i] = projectPoint(T_odom_to_camera * point_odom, K, dist);
    }
    return pixels;
}

RunePnPResult estimateRuneBladePosePnP(
    const std::array<Eigen::Vector2d, 5>& pixels,
    const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist) {
    RunePnPResult result;
    if (!std::all_of(pixels.begin(), pixels.end(), finitePixel)) {
        return result;
    }

    const auto local_points = runeBladeLocalPoints();
    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> image_points;
    object_points.reserve(local_points.size());
    image_points.reserve(pixels.size());
    for (size_t i = 0; i < local_points.size(); ++i) {
        object_points.emplace_back(local_points[i].x(), local_points[i].y(), local_points[i].z());
        image_points.emplace_back(pixels[i].x(), pixels[i].y());
    }

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        K(0, 0), K(0, 1), K(0, 2),
        K(1, 0), K(1, 1), K(1, 2),
        K(2, 0), K(2, 1), K(2, 2));
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) <<
        dist[0], dist[1], dist[2], dist[3], dist[4]);
    cv::Mat rvec;
    cv::Mat tvec;
    try {
        result.success = cv::solvePnP(
            object_points, image_points, camera_matrix, dist_coeffs,
            rvec, tvec, false, cv::SOLVEPNP_IPPE);
    } catch (const cv::Exception&) {
        result.success = false;
    }
    if (!result.success) {
        return result;
    }

    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    for (int r = 0; r < 3; ++r) {
        t(r) = tvec.at<double>(r, 0);
        for (int c = 0; c < 3; ++c) {
            R(r, c) = R_cv.at<double>(r, c);
        }
    }
    result.pose_camera = gtsam::Pose3(gtsam::Rot3(R), auto_graph::eigenToPoint3(t));
    return result;
}

double runeBigCurveAngle(double time, const std::array<double, 5>& params) {
    return -params[0] * std::cos(params[1] * (time + params[2])) +
        params[3] * time + params[4];
}

double runeBigCurveVelocity(double time, const std::array<double, 5>& params) {
    return params[0] * params[1] * std::sin(params[1] * (time + params[2])) + params[3];
}

RuneBigCurveFitter::RuneBigCurveFitter(RuneBigCurveFitterConfig config)
    : config_(config) {
    samples_.reserve(static_cast<size_t>(std::max(config_.max_data_size, 1)));
}

void RuneBigCurveFitter::clear() {
    samples_.clear();
    params_ = {0.470, 1.942, 0.0, 1.178, 0.0};
    has_fit_ = false;
}

void RuneBigCurveFitter::setC(double c) {
    params_[4] = c;
}

void RuneBigCurveFitter::addSample(double time, double roll) {
    if (!std::isfinite(time) || !std::isfinite(roll)) return;
    samples_.push_back({time, roll});
    const size_t max_size = static_cast<size_t>(std::max(config_.max_data_size, 1));
    if (samples_.size() > max_size) {
        samples_.erase(samples_.begin(), samples_.begin() + static_cast<std::ptrdiff_t>(samples_.size() - max_size));
    }
}

bool RuneBigCurveFitter::fitOnce() {
    if (samples_.size() < static_cast<size_t>(std::max(config_.min_data_size, 2))) {
        return false;
    }

    double p[5] = {params_[0], params_[1], params_[2], params_[3], params_[4]};
    ceres::Problem problem;
    for (const auto& sample : samples_) {
        auto* cost = new ceres::AutoDiffCostFunction<BigCurveResidual, 1, 5>(
            new BigCurveResidual(sample.time, sample.roll));
        problem.AddResidualBlock(cost, new ceres::SoftLOneLoss(0.05), p);
    }
    problem.SetParameterLowerBound(p, 0, 0.20);
    problem.SetParameterUpperBound(p, 0, 0.80);
    problem.SetParameterLowerBound(p, 1, 1.60);
    problem.SetParameterUpperBound(p, 1, 2.20);
    problem.SetParameterLowerBound(p, 2, -4.00);
    problem.SetParameterUpperBound(p, 2, 4.00);
    problem.SetParameterLowerBound(p, 3, 0.80);
    problem.SetParameterUpperBound(p, 3, 1.60);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = std::max(config_.max_iterations, 1);
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (!summary.IsSolutionUsable()) {
        return false;
    }
    std::copy(std::begin(p), std::end(p), params_.begin());
    has_fit_ = true;
    return true;
}

void RuneBigCurveFitter::fit() {
    (void)fitOnce();
}

double RuneBigCurveFitter::angleAt(double time) const {
    return runeBigCurveAngle(time, params_);
}

double RuneBigCurveFitter::velocityAt(double time) const {
    return runeBigCurveVelocity(time, params_);
}

RuneRollFactor::RuneRollFactor(const gtsam::SharedNoiseModel& noise,
                               gtsam::Key roll_prev, gtsam::Key vroll_prev,
                               gtsam::Key roll_cur, double dt)
    : Base(noise, roll_prev, vroll_prev, roll_cur), dt_(dt) {}

gtsam::Vector RuneRollFactor::evaluateError(
    const gtsam::Rot2& roll_prev, const double& vroll_prev,
    const gtsam::Rot2& roll_cur, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const {
    // 连续角度模型：roll_cur = roll_prev + vroll_prev * dt。
    // Rot2.localCoordinates() 自动处理 2*pi 周期，避免 roll 跨越 +/-pi 时跳变。
    gtsam::Vector1 error =
        (roll_prev * gtsam::Rot2::fromAngle(vroll_prev * dt_)).localCoordinates(roll_cur);
    if (H1) *H1 = gtsam::Matrix::Constant(1, 1, -1.0);
    if (H2) *H2 = gtsam::Matrix::Constant(1, 1, -dt_);
    if (H3) *H3 = gtsam::Matrix::Identity(1, 1);
    return error;
}

RuneVrollFactor::RuneVrollFactor(const gtsam::SharedNoiseModel& noise,
                                 gtsam::Key prev, gtsam::Key cur)
    : Base(noise, prev, cur) {}

gtsam::Vector RuneVrollFactor::evaluateError(
    const double& prev, const double& cur,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const {
    // 角速度随机游走。小符时它会收敛到近似常值；大符时作为曲线拟合前的平滑观测。
    gtsam::Vector1 error{cur - prev};
    if (H1) *H1 = -gtsam::Matrix1::Identity();
    if (H2) *H2 = gtsam::Matrix1::Identity();
    return error;
}

RuneBladeReprojFactor::RuneBladeReprojFactor(
    const gtsam::SharedNoiseModel& noise, gtsam::Key blade_pose_key,
    const Eigen::Vector3d& point_local, const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist, const Eigen::Vector2d& observed_pixel)
    : Base(noise, blade_pose_key),
      point_local_(point_local.x(), point_local.y(), point_local.z()),
      calib_(K(0, 0), K(1, 1), K(0, 1), K(0, 2), K(1, 2),
             dist[0], dist[1], dist[2], dist[3]),
      observed_(observed_pixel.x(), observed_pixel.y()) {}

gtsam::Vector RuneBladeReprojFactor::evaluateError(
    const gtsam::Pose3& blade_pose_camera, gtsam::OptionalMatrixType H) const {
    // 残差链路：
    // 局部 3D 点 -> 辅助 blade_pose(camera) -> 归一化平面 -> 带畸变像素。
    // 返回 px_pred - px_obs，单位是 pixel。
    gtsam::Matrix36 H_transform;
    gtsam::Point3 p_cam = blade_pose_camera.transformFrom(
        point_local_, H ? &H_transform : nullptr, nullptr);
    gtsam::Matrix23 H_project;
    gtsam::Point2 pn = gtsam::PinholeCamera<gtsam::Cal3DS2>::Project(
        p_cam, H ? &H_project : nullptr);
    gtsam::Matrix22 H_calib;
    gtsam::Point2 px = calib_.uncalibrate(pn, {}, H ? &H_calib : nullptr);
    if (H) *H = H_calib * H_project * H_transform;
    return px - observed_;
}

RuneBladeDirectReprojFactor::RuneBladeDirectReprojFactor(
    const gtsam::SharedNoiseModel& noise, gtsam::Key center_key,
    gtsam::Key roll_key, gtsam::Key normal_yaw_key,
    const Eigen::Isometry3d& T_camera_to_odom, int blade_index,
    double normal_pitch, const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist,
    const std::array<Eigen::Vector2d, 5>& observed_pixels)
    : Base(noise, center_key, roll_key, normal_yaw_key),
      T_camera_to_odom_(T_camera_to_odom),
      blade_index_(blade_index),
      normal_pitch_(normal_pitch),
      K_(K),
      dist_(dist),
      observed_pixels_(observed_pixels) {}

gtsam::Vector RuneBladeDirectReprojFactor::computeError(
    const gtsam::Point3& center,
    const gtsam::Rot2& roll,
    const gtsam::Rot2& normal_yaw) const {
    return runeDirectRollLoss(
        auto_graph::point3ToEigen(center), normal_yaw.theta(), normal_pitch_,
        roll.theta(), blade_index_, T_camera_to_odom_, K_, dist_, observed_pixels_);
}

gtsam::Vector RuneBladeDirectReprojFactor::evaluateError(
    const gtsam::Point3& center,
    const gtsam::Rot2& roll,
    const gtsam::Rot2& normal_yaw,
    gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2,
    gtsam::OptionalMatrixType H3) const {
    const auto error = computeError(center, roll, normal_yaw);
    if (H1) {
        *H1 = finiteDiffColumns(4, 3, [&](const gtsam::Vector& delta) {
            return computeError(
                gtsam::Point3(center.x() + delta[0], center.y() + delta[1], center.z() + delta[2]),
                roll, normal_yaw);
        });
    }
    if (H2) {
        *H2 = finiteDiffColumns(4, 1, [&](const gtsam::Vector& delta) {
            return computeError(center, gtsam::Rot2::fromAngle(roll.theta() + delta[0]), normal_yaw);
        });
    }
    if (H3) {
        *H3 = finiteDiffColumns(4, 1, [&](const gtsam::Vector& delta) {
            return computeError(center, roll, gtsam::Rot2::fromAngle(normal_yaw.theta() + delta[0]));
        });
    }
    return error;
}

RuneBladeRollDirectReprojFactor::RuneBladeRollDirectReprojFactor(
    const gtsam::SharedNoiseModel& noise, gtsam::Key roll_key,
    const Eigen::Vector3d& center, double normal_yaw,
    const Eigen::Isometry3d& T_camera_to_odom, int blade_index,
    double normal_pitch, const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist,
    const std::array<Eigen::Vector2d, 5>& observed_pixels)
    : Base(noise, roll_key),
      center_(center),
      normal_yaw_(normal_yaw),
      T_camera_to_odom_(T_camera_to_odom),
      blade_index_(blade_index),
      normal_pitch_(normal_pitch),
      K_(K),
      dist_(dist),
      observed_pixels_(observed_pixels) {}

gtsam::Vector RuneBladeRollDirectReprojFactor::computeError(
    const gtsam::Rot2& roll) const {
    return runeDirectRollLoss(
        center_, normal_yaw_, normal_pitch_, roll.theta(), blade_index_,
        T_camera_to_odom_, K_, dist_, observed_pixels_);
}

gtsam::Vector RuneBladeRollDirectReprojFactor::evaluateError(
    const gtsam::Rot2& roll, gtsam::OptionalMatrixType H) const {
    const auto error = computeError(roll);
    if (H) {
        *H = finiteDiffColumns(4, 1, [&](const gtsam::Vector& delta) {
            return computeError(gtsam::Rot2::fromAngle(roll.theta() + delta[0]));
        });
    }
    return error;
}

RuneBladeGeometryFactor::RuneBladeGeometryFactor(
    const gtsam::SharedNoiseModel& noise, gtsam::Key blade_pose_key,
    gtsam::Key center_key, gtsam::Key roll_key, gtsam::Key normal_yaw_key,
    const Eigen::Isometry3d& T_camera_to_odom, int blade_index, double normal_pitch)
    : Base(noise, blade_pose_key, center_key, roll_key, normal_yaw_key),
      T_camera_to_odom_(T_camera_to_odom),
      blade_index_(blade_index),
      normal_pitch_(normal_pitch) {}

gtsam::Vector RuneBladeGeometryFactor::computeError(
    const gtsam::Pose3& blade_pose_camera,
    const gtsam::Point3& center,
    const gtsam::Rot2& roll,
    const gtsam::Rot2& normal_yaw) const {
    // 辅助 blade_pose 由像素重投影因子直接约束；这里再把它和主状态绑定：
    // - 位置：blade_pose 的原点应落在同一个 R 标中心；
    // - 姿态：第 i 片扇叶姿态应等于 Rz(normal_yaw)*Ry(normal_pitch)*Rx(roll+i*72deg)。
    const Eigen::Isometry3d blade_pose_odom =
        T_camera_to_odom_ * pose3ToEigen(blade_pose_camera);
    const Eigen::Vector3d blade_center = blade_pose_odom.translation();
    const Eigen::Vector3d expected_center = auto_graph::point3ToEigen(center);

    const Eigen::Vector3d blade_rpy = rotationMatrixToRpyLocal(blade_pose_odom.rotation());
    const double expected_blade_roll = roll.theta() +
        static_cast<double>(blade_index_) * kRuneSlotAngle;
    const Eigen::Vector3d expected_rpy = rotationMatrixToRpyLocal(
        runeBladeRotation(normal_yaw.theta(), normal_pitch_, expected_blade_roll));

    gtsam::Vector error(5);
    const Eigen::Vector3d pos_err = expected_center - blade_center;
    error << pos_err.x(), pos_err.y(), pos_err.z(),
        auto_graph::shortestAngularDistance(blade_rpy.x(), expected_rpy.x()),
        auto_graph::shortestAngularDistance(blade_rpy.z(), expected_rpy.z());
    return error;
}

gtsam::Vector RuneBladeGeometryFactor::evaluateError(
    const gtsam::Pose3& blade_pose_camera,
    const gtsam::Point3& center,
    const gtsam::Rot2& roll,
    const gtsam::Rot2& normal_yaw,
    gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2,
    gtsam::OptionalMatrixType H3,
    gtsam::OptionalMatrixType H4) const {
    const auto error = computeError(blade_pose_camera, center, roll, normal_yaw);

    if (H1) {
        *H1 = finiteDiffColumns(5, 6, [&](const gtsam::Vector& delta) {
            return computeError(blade_pose_camera.retract(delta), center, roll, normal_yaw);
        });
    }
    if (H2) {
        *H2 = finiteDiffColumns(5, 3, [&](const gtsam::Vector& delta) {
            return computeError(
                blade_pose_camera,
                gtsam::Point3(center.x() + delta[0], center.y() + delta[1], center.z() + delta[2]),
                roll, normal_yaw);
        });
    }
    if (H3) {
        *H3 = finiteDiffColumns(5, 1, [&](const gtsam::Vector& delta) {
            return computeError(
                blade_pose_camera, center,
                gtsam::Rot2::fromAngle(roll.theta() + delta[0]), normal_yaw);
        });
    }
    if (H4) {
        *H4 = finiteDiffColumns(5, 1, [&](const gtsam::Vector& delta) {
            return computeError(
                blade_pose_camera, center, roll,
                gtsam::Rot2::fromAngle(normal_yaw.theta() + delta[0]));
        });
    }
    return error;
}

RuneCvGraph::Variables RuneCvGraph::declareVariables(auto_graph::GraphOptimizer& optimizer) {
    return {
        optimizer.declareDynamic<gtsam::Point3>("center", 'c'),
        optimizer.declareDynamic<gtsam::Rot2>("roll", 'r'),
        optimizer.declareDynamic<double>("vroll", 'w'),
        optimizer.declareStatic<gtsam::Rot2>("normal_yaw", 'n'),
    };
}

RuneCvGraph::RuneCvGraph(RuneTrackerConfig config)
    : config_(std::move(config)),
      optimizer_(config_.optimizer),
      vars_(declareVariables(optimizer_)) {}

void RuneCvGraph::initialize() {
    optimizer_.beginInit();
    // 冷启动只给主状态加先验，不写任何观测因子；第一帧 update() 会继续插入预测值。
    state_.center = config_.initial_center;
    state_.normal_yaw = config_.normal_yaw;
    state_.roll = 0.0;
    state_.vroll = 0.0;
    optimizer_.addPrior(vars_.center, auto_graph::eigenToPoint3(state_.center),
                        auto_graph::isotropicNoise(3, config_.center_sigma));
    optimizer_.addPrior(vars_.roll, gtsam::Rot2::fromAngle(state_.roll),
                        auto_graph::isotropicNoise(1, config_.roll_sigma));
    optimizer_.addPrior(vars_.vroll, state_.vroll,
                        auto_graph::isotropicNoise(1, config_.vroll_sigma));
    optimizer_.addPrior(vars_.normal_yaw, gtsam::Rot2::fromAngle(state_.normal_yaw),
                        auto_graph::isotropicNoise(1, config_.normal_yaw_sigma));
    optimizer_.finishInit();
    initialized_ = true;
}

RuneGraphOutput RuneCvGraph::update(
    const auto_aim_interfaces::msg::RuneTargets& msg,
    double dt,
    const Eigen::Isometry3d& T_camera_to_odom) {
    const bool had_previous_frame = initialized_ && optimizer_.getFrameId() > 0;
    if (!initialized_) initialize();
    optimizer_.beginFrame();
    addMotionFactors(dt);
    const auto matched_blade_indices =
        addObservationFactors(msg, T_camera_to_odom, had_previous_frame);
    auto solve_result = optimizer_.solve();
    return makeOutput(solve_result, matched_blade_indices);
}

void RuneCvGraph::addMotionFactors(double dt) {
    const double bounded_dt = std::clamp(dt, 1e-4, 0.2);
    const double predicted_roll = state_.roll + state_.vroll * bounded_dt;
    // 先插入当前帧初值，再添加从上一帧到当前帧的运动因子。
    // center 用强先验保持静止；roll/vroll 通过运动因子在时间上连续。
    optimizer_.insert(vars_.center, auto_graph::eigenToPoint3(state_.center));
    optimizer_.insert(vars_.roll, gtsam::Rot2::fromAngle(predicted_roll));
    optimizer_.insert(vars_.vroll, state_.vroll);
    optimizer_.insert(vars_.normal_yaw, gtsam::Rot2::fromAngle(state_.normal_yaw));

    optimizer_.addPrior(vars_.center, auto_graph::eigenToPoint3(state_.center),
                        auto_graph::isotropicNoise(3, config_.center_sigma));
    optimizer_.addFactor<RuneRollFactor>(
        auto_graph::isotropicNoise(1, config_.roll_sigma),
        optimizer_.keyPrev(vars_.roll), optimizer_.keyPrev(vars_.vroll),
        optimizer_.key(vars_.roll), bounded_dt);
    optimizer_.addFactor<RuneVrollFactor>(
        auto_graph::isotropicNoise(1, config_.vroll_sigma),
        optimizer_.keyPrev(vars_.vroll), optimizer_.key(vars_.vroll));
}

std::vector<int> RuneCvGraph::addObservationFactors(
    const auto_aim_interfaces::msg::RuneTargets& msg,
    const Eigen::Isometry3d& T_camera_to_odom,
    bool had_previous_frame) {
    last_observed_pnp_poses_.clear();
    last_observed_pnp_target_positions_.clear();
    const auto points = runeBladeLocalPoints();
    const auto pixel_noise = auto_graph::isotropicNoise(2, config_.pixel_sigma);
    const auto fallback_pose_prior_noise = auto_graph::isotropicNoise(6, config_.pose_prior_sigma);
    const auto pnp_pose_prior_noise = auto_graph::isotropicNoise(6, config_.pnp_pose_prior_sigma);
    const auto direct_noise = auto_graph::isotropicNoise(4, config_.direct_reproj_sigma);
    Eigen::Matrix<double, 5, 1> geo_sigmas;
    geo_sigmas << config_.center_sigma, config_.center_sigma,
        config_.center_sigma, config_.roll_sigma, config_.normal_yaw_sigma;
    const auto geo_noise = auto_graph::diagonalNoise(geo_sigmas);

    std::vector<RuneObservation> observations;
    observations.reserve(msg.targets.size());
    for (size_t i = 0; i < msg.targets.size(); ++i) {
        const auto& target = msg.targets[i];
        if (!target.is_detected) continue;
        const auto pixels = targetPixels(target);
        if (!std::all_of(pixels.begin(), pixels.end(), finitePixel)) continue;

        RuneObservation observation;
        observation.target_index = i;
        observation.pixels = pixels;
        observation.pnp = estimateRuneBladePosePnP(
            pixels, config_.camera_matrix, config_.distortion);
        if (observation.pnp.success) {
            observation.pose_odom = T_camera_to_odom * pose3ToEigen(observation.pnp.pose_camera);
            observation.center_odom = observation.pose_odom.translation();
            observation.blade_roll = bladeRollInRuneFrame(
                observation.pose_odom.rotation(), state_.normal_yaw, config_.normal_pitch);
        }
        observations.push_back(observation);
    }

    std::vector<RuneMatchCandidate> candidates;
    if (!had_previous_frame) {
        auto anchor_it = std::find_if(
            observations.begin(), observations.end(),
            [](const RuneObservation& obs) { return obs.pnp.success; });
        if (anchor_it == observations.end()) {
            return {};
        }
        const size_t anchor_index =
            static_cast<size_t>(std::distance(observations.begin(), anchor_it));
        candidates.push_back(
            RuneMatchCandidate{anchor_index, 0, -1.0, 0.0, true});

        for (size_t i = 0; i < observations.size(); ++i) {
            if (i == anchor_index || !observations[i].pnp.success) continue;
            const double diff =
                auto_graph::shortestAngularDistance(anchor_it->blade_roll, observations[i].blade_roll);
            const int slot_step = static_cast<int>(std::round(diff / kRuneSlotAngle));
            const int blade_index = positiveModulo5(slot_step);
            const double roll_error = std::abs(diff - static_cast<double>(slot_step) * kRuneSlotAngle);
            const double center_dist =
                (observations[i].center_odom - anchor_it->center_odom).norm();
            if (roll_error <= config_.match_max_roll_diff &&
                center_dist <= config_.match_max_center_distance) {
                candidates.push_back(RuneMatchCandidate{
                    i, blade_index, roll_error, center_dist, true});
            }
        }
    } else {
        for (size_t i = 0; i < observations.size(); ++i) {
            const auto& observation = observations[i];
            if (observation.pnp.success) {
                for (int blade_index = 0; blade_index < 5; ++blade_index) {
                    const double expected_roll =
                        state_.roll + static_cast<double>(blade_index) * kRuneSlotAngle;
                    const double roll_diff = std::abs(
                        auto_graph::shortestAngularDistance(expected_roll, observation.blade_roll));
                    const double center_dist =
                        (observation.center_odom - state_.center).norm();
                    if (roll_diff <= config_.match_max_roll_diff &&
                        center_dist <= config_.match_max_center_distance) {
                        candidates.push_back(RuneMatchCandidate{
                            i, blade_index, roll_diff, center_dist, true});
                    }
                }
            } else if (config_.use_direct_reproj_match_fallback) {
                for (int blade_index = 0; blade_index < 5; ++blade_index) {
                    const double direct_error = directRuneReprojError(
                        state_.center, state_.normal_yaw, config_.normal_pitch,
                        state_.roll, blade_index, T_camera_to_odom,
                        config_.camera_matrix, config_.distortion, observation.pixels);
                    if (direct_error <= config_.match_max_direct_reproj_error) {
                        candidates.push_back(RuneMatchCandidate{
                            i, blade_index, direct_error, 0.0, false});
                    }
                }
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
        if (lhs.primary_score != rhs.primary_score) {
            return lhs.primary_score < rhs.primary_score;
        }
        if (lhs.secondary_score != rhs.secondary_score) {
            return lhs.secondary_score < rhs.secondary_score;
        }
        return lhs.observation_index < rhs.observation_index;
    });

    std::array<bool, 5> used_blades{};
    std::vector<bool> used_observations(observations.size(), false);
    std::vector<RuneMatchedObservation> matches;
    for (const auto& candidate : candidates) {
        if (candidate.observation_index >= observations.size()) continue;
        if (used_observations[candidate.observation_index]) continue;
        if (used_blades[static_cast<size_t>(candidate.blade_index)]) continue;
        used_observations[candidate.observation_index] = true;
        used_blades[static_cast<size_t>(candidate.blade_index)] = true;
        const auto& observation = observations[candidate.observation_index];
        const auto predicted_pose =
            predictedBladePoseCamera(candidate.blade_index, T_camera_to_odom);
        matches.push_back(RuneMatchedObservation{
            observation,
            candidate.blade_index,
            candidate.pnp_success ? observation.pnp.pose_camera : predicted_pose,
            candidate.pnp_success});
    }

    std::vector<int> matched_blade_indices;
    matched_blade_indices.reserve(matches.size());
    for (const auto& match : matches) {
        const int blade_index = match.blade_index;
        matched_blade_indices.push_back(blade_index);
        if (match.pnp_success) {
            last_observed_pnp_poses_.push_back(match.observation.pose_odom);
            last_observed_pnp_target_positions_.push_back(
                match.observation.pose_odom *
                Eigen::Vector3d(0.0, 0.0, kRuneBladeRadius));
        }
        const auto& pixels = match.observation.pixels;
        const auto key = bladePoseKey(optimizer_.getFrameId(), blade_index);
        const auto pose = match.pose_camera;
        // 辅助 Pose3 是每片扇叶在 camera 下的完整位姿初值。
        // 它不是跟踪输出状态，只用于把 2D 重投影残差和低维主状态解耦。
        optimizer_.insertAux(key, pose);
        optimizer_.addAuxPrior(
            key, pose, match.pnp_success ? pnp_pose_prior_noise : fallback_pose_prior_noise);

        for (size_t j = 0; j < points.size(); ++j) {
            // 五个像素点各写一个 2D 残差，全部连接同一个辅助 Pose3。
            optimizer_.addFactor<RuneBladeReprojFactor>(
                pixel_noise, key, points[j], config_.camera_matrix,
                config_.distortion, pixels[j]);
        }
        // 几何因子把辅助 Pose3 拉回 center/roll 模型。这样 PnP 式的位姿自由度
        // 可以吸收像素噪声，而主状态只保留后续预测真正需要的量。
        optimizer_.addFactor<RuneBladeGeometryFactor>(
            geo_noise, key, optimizer_.key(vars_.center),
            optimizer_.key(vars_.roll), optimizer_.key(vars_.normal_yaw),
            T_camera_to_odom, blade_index, config_.normal_pitch);
        if (config_.use_direct_reproj_factor) {
            optimizer_.addFactor<RuneBladeRollDirectReprojFactor>(
                direct_noise, optimizer_.key(vars_.roll),
                state_.center, state_.normal_yaw,
                T_camera_to_odom, blade_index, config_.normal_pitch,
                config_.camera_matrix, config_.distortion, pixels);
        }
    }
    return matched_blade_indices;
}

RuneGraphOutput RuneCvGraph::makeOutput(
    const auto_graph::SolveResult& solve_result,
    std::vector<int> matched_blade_indices) {
    if (!solve_result.failed) {
        try {
            state_.center = auto_graph::point3ToEigen(optimizer_.estimate(vars_.center));
            state_.roll = optimizer_.estimate(vars_.roll).theta();
            state_.vroll = optimizer_.estimate(vars_.vroll);
            state_.normal_yaw = optimizer_.estimate(vars_.normal_yaw).theta();
        } catch (const std::exception&) {
            // 冷启动、空观测或固定滞后窗口裁剪时，某些 key 可能暂时没有估计值；
            // 这种情况下沿用运动预测状态，避免 ROS 节点输出中断。
        }
    }

    RuneGraphOutput out;
    out.solve_result = solve_result;
    out.state = state_;
    out.observed_count = static_cast<int>(matched_blade_indices.size());
    out.matched_blade_indices = std::move(matched_blade_indices);
    out.observed_pnp_poses = last_observed_pnp_poses_;
    out.observed_pnp_target_positions = last_observed_pnp_target_positions_;
    // 默认输出第 0 片扇叶的击打点位置。大符曲线拟合节点可在此基础上继续外推 roll。
    out.target_position = runeTargetPosition(
        state_.center, state_.normal_yaw, config_.normal_pitch, state_.roll);
    return out;
}

gtsam::Pose3 RuneCvGraph::predictedBladePoseCamera(
    int blade_index, const Eigen::Isometry3d& T_camera_to_odom) const {
    // 用当前状态生成辅助 Pose3 初值：先构造 odom 下扇叶姿态，再转到 camera。
    // 这个初值同时用于 aux prior，保证重投影优化不会从无约束位姿发散。
    Eigen::Isometry3d blade_odom = Eigen::Isometry3d::Identity();
    blade_odom.translation() = state_.center;
    blade_odom.linear() = runeBladeRotation(
        state_.normal_yaw, config_.normal_pitch,
        state_.roll + static_cast<double>(blade_index) * kRuneSlotAngle);
    return eigenToPose3(T_camera_to_odom.inverse() * blade_odom);
}

void RuneCvGraph::reset() {
    optimizer_.reset();
    initialized_ = false;
    state_ = RuneTrackerState{};
}

uint64_t RuneCvGraph::frameId() const {
    return optimizer_.getFrameId();
}

}  // namespace filter_test::graph_optimizer

#include "filter_test/graph_optimizer/armor_model.hpp"

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace filter_test::graph_optimizer {

// 装甲板索引按机器人中心 yaw 均匀展开，当前默认四装甲板模型。
// index 0 的装甲板 yaw 等于中心 yaw；1/2/3 依次增加 90/180/270 度。
// 这个函数只负责槽位角度偏移，不做角度归一化，残差处用 Rot2 处理跨 pi。
double armorYawOffset(int armor_index, int armor_count) {
    return 2.0 * M_PI * static_cast<double>(armor_index) /
           static_cast<double>(armor_count);
}

Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R) {
    // 当前几何因子只需要 yaw，但装甲板 pose 由完整 Rot3 给出。
    // 这里按 ZYX 常用约定从旋转矩阵提取 roll/pitch/yaw。
    double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
    double roll = std::atan2(R(2, 1), R(2, 2));
    double yaw = std::atan2(R(1, 0), R(0, 0));
    return {roll, pitch, yaw};
}

namespace {

const std::array<Eigen::Vector3d, 4> kSmallArmorCornersLocal = {{
    {0.0, -0.0675, -0.0625},
    {0.0, -0.0675,  0.0625},
    {0.0,  0.0675,  0.0625},
    {0.0,  0.0675, -0.0625},
}};

double sigmaFromVariance(double variance) {
    return std::sqrt(std::max(variance, 1e-12));
}

TrackerState initialStateFromFirstArmor(
    const auto_aim_interfaces::msg::Armors& msg) {
    TrackerState state;
    if (msg.armors.empty()) {
        return state;
    }
    const auto& armor = msg.armors.front();
    constexpr double init_radius = 0.25;
    // 装甲板位置 = center - r * normal，因此反推 center 时沿 armor yaw 正方向加半径。
    state.center = {
        armor.pose.position.x + init_radius * std::cos(armor.yaw),
        armor.pose.position.y + init_radius * std::sin(armor.yaw),
        armor.pose.position.z};
    state.velocity.setZero();
    state.yaw = armor.yaw;
    state.vyaw = 0.0;
    state.radius_1 = init_radius;
    state.radius_2 = init_radius;
    state.dz = 0.0;
    return state;
}

gtsam::Key armorPoseKey(uint64_t frame_id, int armor_index) {
    // 每个物理装甲板槽位独立 prefix，frame id 仍表示时间。
    static constexpr char prefixes[] = {'h', 'j', 'k', 'l'};
    const int clamped_index = std::max(0, std::min(armor_index, 3));
    return gtsam::Symbol(prefixes[clamped_index], frame_id);
}

gtsam::Pose3 armorPoseCameraFromObservation(
    const auto_aim_interfaces::msg::Armor& armor,
    const Eigen::Isometry3d& T_camera_to_odom) {
    // ROS 输入里的 armor pose 在 odom 系；像素重投影需要 camera 系 Pose3。
    const Eigen::Isometry3d T_odom_to_camera = T_camera_to_odom.inverse();
    const Eigen::Vector3d armor_pos_odom(
        armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);
    const Eigen::Vector3d armor_pos_camera = T_odom_to_camera * armor_pos_odom;

    const Eigen::Quaterniond q_armor_odom(
        armor.pose.orientation.w, armor.pose.orientation.x,
        armor.pose.orientation.y, armor.pose.orientation.z);
    const Eigen::Quaterniond q_armor_camera(
        T_odom_to_camera.rotation() * q_armor_odom.toRotationMatrix());

    return {
        gtsam::Rot3::Quaternion(
            q_armor_camera.w(), q_armor_camera.x(),
            q_armor_camera.y(), q_armor_camera.z()),
        gtsam::Point3(
            armor_pos_camera.x(), armor_pos_camera.y(), armor_pos_camera.z())};
}

double armorMatchCost(const PredictedArmor& predicted,
                      const auto_aim_interfaces::msg::Armor& observed,
                      int last_armor_index) {
    const double yaw_diff =
        std::abs(auto_graph::shortestAngularDistance(predicted.yaw, observed.yaw));
    const double dx = observed.pose.position.x - predicted.position.x();
    const double dy = observed.pose.position.y - predicted.position.y();
    const double dz_err = observed.pose.position.z - predicted.position.z();
    const double pos_err = std::sqrt(dx * dx + dy * dy + dz_err * dz_err);

    // yaw_diff 单位是 rad，pos_err 单位是 m。这里的 3.0 是经验权重，
    // 让 0.1 m 位置误差大致等价于 0.3 rad yaw 误差。
    double cost = yaw_diff + 3.0 * pos_err;
    if (predicted.index == last_armor_index) cost *= 0.85;
    return cost;
}

Eigen::Matrix<double, 4, 6> armorPoseGeometryJacobian(
    const Eigen::Isometry3d& armor_pose_odom,
    double tx, double ty, double nx, double ny,
    double dx, double dy, double tangential_err) {
    // H1 对应 armor_pose_camera 的局部扰动 [rot, trans]。
    // Pose3::retract 的平移扰动在装甲板自身局部系，因此位置雅可比需要右乘
    // armor_pose_odom.rotation()；yaw 对旋转扰动的导数由 RPY yaw 对局部角速度链式得到。
    Eigen::Matrix<double, 4, 3> J_position;
    J_position << -tx, -ty, 0.0,
                  -nx, -ny, 0.0,
                   0.0, 0.0, -1.0,
                   0.0, 0.0, 0.0;

    const double radial_projection = nx * dx + ny * dy;
    Eigen::Matrix<double, 4, 1> d_error_d_yaw;
    d_error_d_yaw << -radial_projection, tangential_err, 0.0, -1.0;

    const Eigen::Vector3d rpy =
        rotationMatrixToRPY(armor_pose_odom.rotation().matrix());
    const double roll = rpy.x();
    const double pitch = rpy.y();
    const double cos_pitch = std::cos(pitch);
    Eigen::Matrix<double, 1, 3> d_yaw_d_omega;
    d_yaw_d_omega << 0.0, std::sin(roll) / cos_pitch,
        std::cos(roll) / cos_pitch;

    Eigen::Matrix<double, 4, 6> H1;
    H1.leftCols<3>() = d_error_d_yaw * d_yaw_d_omega;
    H1.rightCols<3>() = J_position * armor_pose_odom.rotation().matrix();
    return H1;
}

}  // namespace

TranslationFactor::TranslationFactor(
    const gtsam::SharedNoiseModel& noise, gtsam::Key x_prev,
    gtsam::Key v_prev, gtsam::Key x_cur, double dt)
    : Base(noise, x_prev, v_prev, x_cur), dt_(dt) {}

gtsam::Vector TranslationFactor::evaluateError(
    const gtsam::Point3& x_prev, const gtsam::Vector3& v_prev,
    const gtsam::Point3& x_cur, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const {
    // X(k) - X(k-1) - V(k-1) * dt 的一阶雅可比。
    if (H1) *H1 = -gtsam::Matrix3::Identity();
    if (H2) *H2 = -dt_ * gtsam::Matrix3::Identity();
    if (H3) *H3 = gtsam::Matrix3::Identity();
    return x_cur - (x_prev + v_prev * dt_);
}

YawFactor::YawFactor(
    const gtsam::SharedNoiseModel& noise, gtsam::Key yaw_prev,
    gtsam::Key vyaw_prev, gtsam::Key yaw_cur, double dt)
    : Base(noise, yaw_prev, vyaw_prev, yaw_cur), dt_(dt) {}

gtsam::Vector YawFactor::evaluateError(
    const gtsam::Rot2& yaw_prev, const double& vyaw_prev,
    const gtsam::Rot2& yaw_cur, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const {
    // 先在 Rot2 流形上积分上一帧 yaw，再用 localCoordinates 计算最短角残差。
    if (H1) *H1 = gtsam::Matrix::Constant(1, 1, -1.0);
    if (H2) *H2 = gtsam::Matrix::Constant(1, 1, -dt_);
    if (H3) *H3 = gtsam::Matrix::Identity(1, 1);
    return (yaw_prev * gtsam::Rot2::fromAngle(vyaw_prev * dt_))
        .localCoordinates(yaw_cur);
}

VelocityFactor::VelocityFactor(
    const gtsam::SharedNoiseModel& noise, gtsam::Key v_prev, gtsam::Key v_cur)
    : Base(noise, v_prev, v_cur) {}

gtsam::Vector VelocityFactor::evaluateError(
    const gtsam::Vector3& v_prev, const gtsam::Vector3& v_cur,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const {
    // 常速模型的速度随机游走，噪声大小由 tracker config 控制。
    if (H1) *H1 = -gtsam::Matrix3::Identity();
    if (H2) *H2 = gtsam::Matrix3::Identity();
    return v_cur - v_prev;
}

VyawFactor::VyawFactor(
    const gtsam::SharedNoiseModel& noise, gtsam::Key w_prev, gtsam::Key w_cur)
    : Base(noise, w_prev, w_cur) {}

gtsam::Vector VyawFactor::evaluateError(
    const double& w_prev, const double& w_cur,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const {
    // 标量角速度随机游走。
    if (H1) *H1 = -gtsam::Matrix::Identity(1, 1);
    if (H2) *H2 = gtsam::Matrix::Identity(1, 1);
    return gtsam::Vector1(w_cur - w_prev);
}

std::vector<PredictedArmor> predictedArmorsFromState(const TrackerState& state) {
    std::vector<PredictedArmor> armors;
    armors.reserve(4);
    for (int i = 0; i < 4; ++i) {
        // 约定与仿真一致：
        //   armor_position = center - radius * [cos(armor_yaw), sin(armor_yaw), 0]
        // 偶数装甲板使用 radius_1 且与中心同层；奇数装甲板使用 radius_2 和 dz。
        const double armor_yaw = state.yaw + i * M_PI_2;
        const double radius = (i % 2 == 0) ? state.radius_1 : state.radius_2;
        const double dz = (i % 2 == 0) ? 0.0 : state.dz;

        PredictedArmor armor;
        armor.index = i;
        armor.yaw = armor_yaw;
        armor.radius = radius;
        armor.dz = dz;
        armor.position.x() = state.center.x() - radius * std::cos(armor_yaw);
        armor.position.y() = state.center.y() - radius * std::sin(armor_yaw);
        armor.position.z() = state.center.z() + dz;
        armors.push_back(armor);
    }
    return armors;
}

int matchArmorIndex(const TrackerState& state,
                    const auto_aim_interfaces::msg::Armor& armor,
                    int last_armor_index,
                    double max_match_cost) {
    const auto predicted_armors = predictedArmorsFromState(state);
    double best_cost = std::numeric_limits<double>::max();
    int best_index = last_armor_index >= 0 ? last_armor_index : 0;

    for (const auto& predicted_armor : predicted_armors) {
        // 单观测匹配使用和批量匹配相同的代价，但不处理同帧唯一性。
        // 主要留给单元测试和只有一块观测的场景。
        const double cost =
            armorMatchCost(predicted_armor, armor, last_armor_index);
        if (cost < best_cost) {
            best_cost = cost;
            best_index = predicted_armor.index;
        }
    }

    return best_cost <= max_match_cost ? best_index : -1;
}

std::vector<int> matchArmorIndicesUnique(
    const TrackerState& state,
    const auto_aim_interfaces::msg::Armors& msg,
    int last_armor_index,
    double max_match_cost) {
    const auto predicted_armors = predictedArmorsFromState(state);
    std::vector<int> matched;
    matched.reserve(msg.armors.size());
    std::array<bool, 4> used_indices = {false, false, false, false};

    for (const auto& armor : msg.armors) {
        double best_cost = std::numeric_limits<double>::max();
        int best_index = -1;

        // 匹配仍是轻量启发式：yaw 连续性 + 位置误差 + 上一帧索引滞回。
        // 这里没有用马氏距离，是为了让匹配层保持独立于 graph 的协方差信息。
        for (const auto& predicted_armor : predicted_armors) {
            if (used_indices[static_cast<std::size_t>(predicted_armor.index)]) {
                continue;
            }

            const double cost = armorMatchCost(
                predicted_armor, armor, last_armor_index);
            if (cost < best_cost) {
                best_cost = cost;
                best_index = predicted_armor.index;
            }
        }

        // 代价过大的观测直接拒绝，避免一帧错配把静态 r/dz 写歪。
        // 车辆最多只有 4 个物理装甲板槽位。超出的检测也保留为未匹配,
        // 避免像素后端把多块观测挂到同一个 Pose3 key 上。
        if (best_index >= 0 && best_cost <= max_match_cost) {
            used_indices[static_cast<std::size_t>(best_index)] = true;
        } else {
            best_index = -1;
        }
        matched.push_back(best_index);
    }

    return matched;
}

ArmorRadiusCenterZFactor::ArmorRadiusCenterZFactor(
    const gtsam::SharedNoiseModel& noise,
    gtsam::Key armor_pose_key, gtsam::Key radius_key,
    gtsam::Key center_yaw_key, gtsam::Key center_key,
    const Eigen::Isometry3d& T_camera_to_odom,
    int armor_index, double radius_min, double radius_max)
    : Base(noise, armor_pose_key, radius_key, center_yaw_key, center_key),
      T_camera_to_odom_(T_camera_to_odom),
      armor_index_(armor_index),
      radius_min_(radius_min),
      radius_max_(radius_max) {}

gtsam::Vector ArmorRadiusCenterZFactor::evaluateError(
    const gtsam::Pose3& armor_pose_camera, const double& radius_u,
    const gtsam::Rot2& center_yaw, const gtsam::Point3& center,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
    gtsam::OptionalMatrixType H3, gtsam::OptionalMatrixType H4) const {
    // 几何因子在 odom 系下工作：先把独立的装甲板 Pose3(camera) 转成全局位姿。
    // armor_pose_camera 是由像素重投影优化的局部变量；center/yaw/radius 是主状态。
    // 这层因子把两者连接起来，形成“像素 -> Pose3 -> 中心状态”的两级图结构。
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    pose.pretranslate(armor_pose_camera.translation());
    pose.rotate(armor_pose_camera.rotation().matrix());
    const Eigen::Isometry3d armor_pose_odom = T_camera_to_odom_ * pose;
    const Eigen::Vector3d armor_position = armor_pose_odom.translation();
    const double armor_yaw_angle =
        rotationMatrixToRPY(armor_pose_odom.rotation().matrix()).z();
    const auto armor_yaw = gtsam::Rot2::fromAngle(armor_yaw_angle);

    const double radius =
        auto_graph::logisticFunction(radius_u, radius_min_, radius_max_);
    const double nx = std::cos(armor_yaw.theta());
    const double ny = std::sin(armor_yaw.theta());
    const double tx = -ny;
    const double ty = nx;
    const double dx = center.x() - armor_position.x();
    const double dy = center.y() - armor_position.y();

    // 偶数装甲板约束中心 XY、同层 Z 和中心 yaw；半径仍使用 logistic 静态状态。
    // residual 分量:
    //   tangential: 中心到装甲板向量在切向上的分量应为 0
    //   radial:     中心到装甲板向量在法向上的分量应等于半径
    //   z:          偶数装甲板和中心同层
    //   yaw:        装甲板 yaw 应等于 center_yaw + index offset
    const double tangential_err = tx * dx + ty * dy;
    const double radial_err = nx * dx + ny * dy - radius;
    const double z_err = center.z() - armor_position.z();
    const auto predicted_yaw =
        gtsam::Rot2::fromAngle(center_yaw.theta() + armorYawOffset(armor_index_));
    const double yaw_err = armor_yaw.localCoordinates(predicted_yaw).x();

    if (H1) {
        *H1 = armorPoseGeometryJacobian(
            armor_pose_odom, tx, ty, nx, ny, dx, dy, tangential_err);
    }
    if (H2) {
        // radius_u 是无界状态，radial residual 对 radius_u 的导数为 -d(radius)/du。
        const double dr =
            auto_graph::logisticDerivative(radius, radius_min_, radius_max_);
        *H2 = (gtsam::Matrix(4, 1) << 0.0, -dr, 0.0, 0.0).finished();
    }
    if (H3) {
        // yaw residual = armor_yaw.localCoordinates(predicted_yaw)，
        // 对 predicted center_yaw 的局部导数近似为 +1。
        *H3 = (gtsam::Matrix(4, 1) << 0.0, 0.0, 0.0, 1.0).finished();
    }
    if (H4) {
        // center 只影响 tangential/radial/z 三个几何残差；yaw 残差不直接依赖 center。
        *H4 = (gtsam::Matrix(4, 3) << tx, ty, 0.0,
               nx, ny, 0.0,
               0.0, 0.0, 1.0,
               0.0, 0.0, 0.0).finished();
    }
    return gtsam::Vector4(tangential_err, radial_err, z_err, yaw_err);
}

ArmorRadiusDZFactor::ArmorRadiusDZFactor(
    const gtsam::SharedNoiseModel& noise,
    gtsam::Key armor_pose_key, gtsam::Key radius_key,
    gtsam::Key dz_key, gtsam::Key center_yaw_key,
    gtsam::Key center_key, const Eigen::Isometry3d& T_camera_to_odom,
    int armor_index, double radius_min, double radius_max)
    : Base(noise, armor_pose_key, radius_key, dz_key, center_yaw_key, center_key),
      T_camera_to_odom_(T_camera_to_odom),
      armor_index_(armor_index),
      radius_min_(radius_min),
      radius_max_(radius_max) {}

gtsam::Vector ArmorRadiusDZFactor::evaluateError(
    const gtsam::Pose3& armor_pose_camera, const double& radius_u,
    const double& dz, const gtsam::Rot2& center_yaw,
    const gtsam::Point3& center, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
    gtsam::OptionalMatrixType H4, gtsam::OptionalMatrixType H5) const {
    // 奇数装甲板比偶数装甲板多一个 dz 静态变量，用来描述上下层高度差。
    // 其余 tangential/radial/yaw 约束与偶数装甲板一致。
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    pose.pretranslate(armor_pose_camera.translation());
    pose.rotate(armor_pose_camera.rotation().matrix());
    const Eigen::Isometry3d armor_pose_odom = T_camera_to_odom_ * pose;
    const Eigen::Vector3d armor_position = armor_pose_odom.translation();
    const double armor_yaw_angle =
        rotationMatrixToRPY(armor_pose_odom.rotation().matrix()).z();
    const auto armor_yaw = gtsam::Rot2::fromAngle(armor_yaw_angle);

    const double radius =
        auto_graph::logisticFunction(radius_u, radius_min_, radius_max_);
    const double nx = std::cos(armor_yaw.theta());
    const double ny = std::sin(armor_yaw.theta());
    const double tx = -ny;
    const double ty = nx;
    const double dx = center.x() - armor_position.x();
    const double dy = center.y() - armor_position.y();

    // 奇数装甲板的高度模型为 armor_z = center_z + dz。
    const double tangential_err = tx * dx + ty * dy;
    const double radial_err = nx * dx + ny * dy - radius;
    const double z_err = center.z() + dz - armor_position.z();
    const auto predicted_yaw =
        gtsam::Rot2::fromAngle(center_yaw.theta() + armorYawOffset(armor_index_));
    const double yaw_err = armor_yaw.localCoordinates(predicted_yaw).x();

    if (H1) {
        *H1 = armorPoseGeometryJacobian(
            armor_pose_odom, tx, ty, nx, ny, dx, dy, tangential_err);
    }
    if (H2) {
        // radius_b 的导数和偶数装甲板相同，只作用在 radial residual 上。
        const double dr =
            auto_graph::logisticDerivative(radius, radius_min_, radius_max_);
        *H2 = (gtsam::Matrix(4, 1) << 0.0, -dr, 0.0, 0.0).finished();
    }
    if (H3) {
        // z residual = center.z + dz - armor.z，所以对 dz 的导数为 +1。
        *H3 = (gtsam::Matrix(4, 1) << 0.0, 0.0, 1.0, 0.0).finished();
    }
    if (H4) {
        // 对 center_yaw 的导数只影响 yaw residual。
        *H4 = (gtsam::Matrix(4, 1) << 0.0, 0.0, 0.0, 1.0).finished();
    }
    if (H5) {
        // center 的导数结构与偶数装甲板一致。
        *H5 = (gtsam::Matrix(4, 3) << tx, ty, 0.0,
               nx, ny, 0.0,
               0.0, 0.0, 1.0,
               0.0, 0.0, 0.0).finished();
    }
    return gtsam::Vector4(tangential_err, radial_err, z_err, yaw_err);
}

ArmorTypedReprojFactor::ArmorTypedReprojFactor(
    const gtsam::SharedNoiseModel& noise,
    gtsam::Key armor_pose_key,
    const Eigen::Vector3d& corner_local,
    const Eigen::Matrix3d& K,
    const std::array<double, 5>& dist,
    const Eigen::Vector2d& observed_pixel)
    : Base(noise, armor_pose_key),
      // corner_local 使用装甲板局部坐标：X 法线, Y 宽度, Z 高度。
      armor_point_(corner_local.x(), corner_local.y(), corner_local.z()),
      calib_(K(0, 0), K(1, 1), K(0, 1), K(0, 2), K(1, 2),
             dist[0], dist[1], dist[2], dist[3]),
      px_obs_(observed_pixel.x(), observed_pixel.y()) {}

gtsam::Vector ArmorTypedReprojFactor::evaluateError(
    const gtsam::Pose3& armor_pose_camera,
    gtsam::OptionalMatrixType H) const {
    // 重投影只约束单块装甲板 Pose3；中心/半径/yaw 由几何因子再连接到主状态。
    // 计算链条:
    //   local corner -> camera point -> normalized point -> distorted pixel
    // H 按链式法则从 pixel residual 回传到 Pose3。
    gtsam::Matrix36 H_transform;
    gtsam::Point3 p_cam = armor_pose_camera.transformFrom(
        armor_point_, H ? &H_transform : nullptr, nullptr);
    gtsam::Matrix23 H_norm;
    gtsam::Point2 pn = gtsam::PinholeCamera<gtsam::Cal3DS2>::Project(
        p_cam, H ? &H_norm : nullptr);
    gtsam::Matrix22 H_calib;
    gtsam::Point2 px = calib_.uncalibrate(pn, {}, H ? &H_calib : nullptr);
    if (H) *H = H_calib * H_norm * H_transform;
    // GTSAM 约定 residual = predicted - measured。
    return px - px_obs_;
}

ArmorCvPixelGraph::Variables ArmorCvPixelGraph::declareVariables(
    auto_graph::GraphOptimizer& optimizer) {
    // 主变量布局参考 jlu tracker：位置、速度、yaw、vyaw 拆成 typed 变量；
    // 半径和 dz 是 frame 0 静态变量。
    //
    // dynamic 变量的 key 会随 frame id 增长，例如 x0/x1/x2；static 变量始终
    // 使用 prefix + 0，例如 a0/b0/z0。这样半径和 dz 可以被所有帧共同约束，
    // fixed-lag smoother 也能通过 GraphOptimizer 刷新静态 key 时间戳。
    return {
        optimizer.declareDynamic<gtsam::Point3>("center", 'x'),
        optimizer.declareDynamic<gtsam::Vector3>("velocity", 'v'),
        optimizer.declareDynamic<gtsam::Rot2>("yaw", 'r'),
        optimizer.declareDynamic<double>("vyaw", 'w'),
        optimizer.declareStatic<double>("radius_a", 'a'),
        optimizer.declareStatic<double>("radius_b", 'b'),
        optimizer.declareStatic<double>("dz", 'z')};
}

ArmorCvPixelGraph::ArmorCvPixelGraph(TrackerConfig config)
    : config_(std::move(config)),
      optimizer_(config_.optimizer),
      vars_(declareVariables(optimizer_)) {}

void ArmorCvPixelGraph::initialize(
    const auto_aim_interfaces::msg::Armors& armors_msg) {
    optimizer_.beginInit();

    // 第一帧只用第一块装甲板反推中心初值，后续由图优化逐步收敛半径/dz。
    // runtime_ 是跨帧缓存，这里必须写入初始状态，否则 update() 阶段无法用
    // 上一帧状态生成本帧预测初值。
    const auto state = initialStateFromFirstArmor(armors_msg);
    runtime_.state = state;
    runtime_.last_armor_index = 0;
    runtime_.matched_indices.clear();

    const double radius_a_u = radiusToState(state.radius_1);
    const double radius_b_u = radiusToState(state.radius_2);

    // 初始先验约束 frame 0，并同时插入对应初值。静态几何参数用配置里的
    // prior_noise 控制可信度；GTSAM 半径变量使用无界 logistic 状态，避免优化时
    // 直接越过物理半径上下限。
    // 这里的先验不是最终真值，只是给 iSAM2 一个可线性化的起点；后续像素因子
    // 和几何因子会持续修正 center/yaw/radius/dz。
    optimizer_.addPrior(
        vars_.center, auto_graph::eigenToPoint3(state.center),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.1)));
    optimizer_.addPrior(
        vars_.velocity, state.velocity,
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.5)));
    optimizer_.addPrior(
        vars_.yaw, gtsam::Rot2::fromAngle(state.yaw),
        gtsam::noiseModel::Isotropic::Sigma(1, 0.25));
    optimizer_.addPrior(
        vars_.vyaw, state.vyaw,
        gtsam::noiseModel::Isotropic::Sigma(1, 0.5));
    optimizer_.addPrior(
        vars_.radius_a, radius_a_u,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.prior_noise.radius));
    optimizer_.addPrior(
        vars_.radius_b, radius_b_u,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.prior_noise.radius));
    optimizer_.addPrior(
        vars_.dz, state.dz,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.prior_noise.dz));

    optimizer_.finishInit();
    initialized_ = true;
}

ArmorCvPixelOutput ArmorCvPixelGraph::update(
    const auto_aim_interfaces::msg::Armors& armors_msg,
    double dt,
    const Eigen::Isometry3d& T_camera_to_odom) {
    // 每个增量帧先加入运动约束，再加入观测约束。运动约束负责把 k-1 与 k
    // 串起来；观测约束负责把本帧检测到的装甲板 Pose3 拉回中心/yaw/半径状态。
    optimizer_.beginFrame();
    addMotionFactors(dt);
    addObservationFactors(armors_msg, T_camera_to_odom);
    return makeOutput(optimizer_.solve());
}

void ArmorCvPixelGraph::addMotionFactors(double dt) {
    auto& state = runtime_.state;
    // 先用上一轮输出做显式预测，作为本帧 typed variables 的初值。
    // 这一步只更新 runtime 中的预测值；真正的概率约束在下面的 GTSAM 因子中。
    // dt 来自 ROS 消息时间差或调用层模拟时间，必须同时用于显式预测和
    // Translation/Yaw 因子，保证初值和运动模型一致。
    state.center += state.velocity * dt;
    state.yaw += state.vyaw * dt;

    // 本帧动态变量的初值写入 GraphOptimizer 暂存 Values。静态变量不在每帧
    // 重复插入，它们仍通过 frame 0 的 a0/b0/z0 被几何因子持续连接。
    optimizer_.insert(vars_.center, auto_graph::eigenToPoint3(state.center));
    optimizer_.insert(vars_.velocity, state.velocity);
    optimizer_.insert(vars_.yaw, gtsam::Rot2::fromAngle(state.yaw));
    optimizer_.insert(vars_.vyaw, state.vyaw);

    // 运动模型拆成位置积分、速度随机游走、yaw 积分、vyaw 随机游走四类因子。
    // config_.s2q* 是过程噪声方差配置；因子接口需要标准差，所以统一走
    // sigmaFromVariance()，同时对极小值做保护以避免零噪声。
    const auto translation_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(sigmaFromVariance(config_.s2qxy),
                       sigmaFromVariance(config_.s2qxy),
                       sigmaFromVariance(config_.s2qz)));
    optimizer_.addFactor<TranslationFactor>(
        translation_noise,
        optimizer_.keyPrev(vars_.center), optimizer_.keyPrev(vars_.velocity),
        optimizer_.key(vars_.center), dt);

    // 速度和角速度不做显式积分，只加随机游走平滑因子。vel_sigma/vyaw_sigma 越小，
    // 图越倾向于保持上一帧速度；越大则更信任观测带来的突变。
    const auto velocity_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(config_.vel_sigma, config_.vel_sigma, config_.vel_sigma));
    optimizer_.addFactor<VelocityFactor>(
        velocity_noise, optimizer_.keyPrev(vars_.velocity),
        optimizer_.key(vars_.velocity));

    optimizer_.addFactor<YawFactor>(
        gtsam::noiseModel::Isotropic::Sigma(
            1, sigmaFromVariance(config_.s2qyaw)),
        optimizer_.keyPrev(vars_.yaw), optimizer_.keyPrev(vars_.vyaw),
        optimizer_.key(vars_.yaw), dt);
    optimizer_.addFactor<VyawFactor>(
        gtsam::noiseModel::Isotropic::Sigma(1, config_.vyaw_sigma),
        optimizer_.keyPrev(vars_.vyaw), optimizer_.key(vars_.vyaw));
}

void ArmorCvPixelGraph::addObservationFactors(
    const auto_aim_interfaces::msg::Armors& armors_msg,
    const Eigen::Isometry3d& T_camera_to_odom) {
    // 匹配在 armor graph 内完成，core 只看到具体 key 和 factor。
    // armors_msg.armors 的顺序是检测器输出顺序；matched_indices 与它一一对应，
    // 值为 0..3 表示物理装甲板槽位，-1 表示该观测不参与本帧图优化。
    // last_armor_index 用于跨帧连续性，避免 yaw 接近 +-pi 时装甲板编号跳变。
    runtime_.matched_indices =
        matchArmorIndicesUnique(runtime_.state, armors_msg,
                                runtime_.last_armor_index,
                                config_.match_max_cost);

    const auto pixel_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector2(config_.pixel_sigma, config_.pixel_sigma));
    gtsam::Vector6 pose_prior_sigmas;
    pose_prior_sigmas << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    const auto pose_prior_noise =
        gtsam::noiseModel::Diagonal::Sigmas(pose_prior_sigmas);
    const auto geo_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector4(config_.geo_noise.tangential,
                       config_.geo_noise.radial,
                       config_.geo_noise.height,
                       config_.geo_noise.yaw));

    for (std::size_t i = 0; i < armors_msg.armors.size(); ++i) {
        const int armor_index = runtime_.matched_indices[i];
        if (armor_index < 0) {
            // 未通过匹配阈值的观测不建 Pose3、不建像素因子，也不影响本帧求解。
            // 这样离群点只会体现在 output.matched_indices 中，调用层可据此判断丢失。
            continue;
        }
        const auto& armor = armors_msg.armors[i];
        addArmorObservationFactors(
            armor, armor_index, T_camera_to_odom, pixel_noise,
            pose_prior_noise, geo_noise);
    }
}

void ArmorCvPixelGraph::addArmorObservationFactors(
    const auto_aim_interfaces::msg::Armor& armor,
    int armor_index,
    const Eigen::Isometry3d& T_camera_to_odom,
    const gtsam::SharedNoiseModel& pixel_noise,
    const gtsam::SharedNoiseModel& pose_prior_noise,
    const gtsam::SharedNoiseModel& geo_noise) {
    const auto& corners = kSmallArmorCornersLocal;
    // 这里采用两级观测：
    // 1. 每个装甲板观测先生成一个辅助 Pose3 key，表达相机坐标系下的装甲板位姿；
    // 2. 像素重投影因子只连接这个 Pose3，保留角点级像素信息；
    // 3. 几何因子再把 Pose3 连接回主状态 center/yaw/radius/dz。
    const auto akey = armorPoseKey(optimizer_.getFrameId(), armor_index);
    const auto armor_pose_camera =
        armorPoseCameraFromObservation(armor, T_camera_to_odom);

    // Pose3 先验让每块观测装甲板有一个局部锚点；像素因子和几何因子再共同修正。
    optimizer_.insertAux<gtsam::Pose3>(akey, armor_pose_camera);
    optimizer_.addAuxPrior<gtsam::Pose3>(
        akey, armor_pose_camera, pose_prior_noise);

    // 每个角点一个 2D 重投影因子，保留角点级像素信息。
    // detected_points 少于 4 时跳过缺失角点，已存在角点仍可贡献约束。
    for (int corner = 0; corner < 4; ++corner) {
        if (static_cast<std::size_t>(corner) >= armor.detected_points.size()) {
            continue;
        }
        optimizer_.addFactor<ArmorTypedReprojFactor>(
            pixel_noise, akey, corners[corner], config_.camera_matrix,
            config_.distortion,
            Eigen::Vector2d(armor.detected_points[corner].x,
                            armor.detected_points[corner].y));
    }

    // 偶数/奇数装甲板连接到不同静态半径；奇数额外连接 dz。
    // armor_index 的奇偶含义来自四装甲板几何：0/2 共用 radius_a，1/3 共用
    // radius_b，并且奇数槽位对应上下层高度差 dz。
    if (armor_index % 2 == 0) {
        optimizer_.addFactor<ArmorRadiusCenterZFactor>(
            geo_noise, akey, optimizer_.key(vars_.radius_a, 0),
            optimizer_.key(vars_.yaw), optimizer_.key(vars_.center),
            T_camera_to_odom, armor_index, kRadiusMin, kRadiusMax);
    } else {
        optimizer_.addFactor<ArmorRadiusDZFactor>(
            geo_noise, akey, optimizer_.key(vars_.radius_b, 0),
            optimizer_.key(vars_.dz, 0), optimizer_.key(vars_.yaw),
            optimizer_.key(vars_.center), T_camera_to_odom, armor_index,
            kRadiusMin, kRadiusMax);
    }
}

ArmorCvPixelOutput ArmorCvPixelGraph::makeOutput(
    const auto_graph::SolveResult& solve_result) {
    ArmorCvPixelOutput output;
    // 默认先返回 runtime 中的预测状态。冷启动帧、无因子帧或失败帧都可能没有新的
    // estimate；这种情况下调用层仍能拿到连续预测，而不是空输出。
    output.solve_result = solve_result;
    output.matched_indices = runtime_.matched_indices;
    output.state = runtime_.state;

    if (solve_result.optimized) {
        // 只有真正优化成功后才用 GTSAM estimate 覆盖 runtime 状态；
        // 冷启动或失败帧保持预测状态，保证调用层有连续输出。
        output.state.center = auto_graph::point3ToEigen(
            optimizer_.estimate(vars_.center));
        output.state.velocity = optimizer_.estimate(vars_.velocity);
        output.state.yaw = optimizer_.estimate(vars_.yaw).theta();
        output.state.vyaw = optimizer_.estimate(vars_.vyaw);
        output.state.radius_1 = auto_graph::logisticFunction(
            optimizer_.estimate(vars_.radius_a), kRadiusMin, kRadiusMax);
        output.state.radius_2 = auto_graph::logisticFunction(
            optimizer_.estimate(vars_.radius_b), kRadiusMin, kRadiusMax);
        output.state.dz = optimizer_.estimate(vars_.dz);
        runtime_.state = output.state;
        // 只有优化成功后才提交 last_armor_index，避免失败帧或冷启动帧把离群匹配
        // 写进跨帧连续性缓存。
        for (auto it = runtime_.matched_indices.rbegin();
             it != runtime_.matched_indices.rend(); ++it) {
            if (*it >= 0) {
                runtime_.last_armor_index = *it;
                break;
            }
        }
    }
    // predicted_armors 永远从最终 output.state 推导：优化成功则用 estimate，
    // 未优化则用预测状态。调用层不需要知道本帧是否真的提交给 iSAM2。
    output.predicted_armors = predictedArmorsFromState(output.state);
    return output;
}

void ArmorCvPixelGraph::reset() {
    optimizer_.reset();
    runtime_ = ArmorCvPixelRuntime{};
    initialized_ = false;
}

uint64_t ArmorCvPixelGraph::frameId() const {
    return initialized_ ? optimizer_.getFrameId() : 0;
}

}  // namespace filter_test::graph_optimizer

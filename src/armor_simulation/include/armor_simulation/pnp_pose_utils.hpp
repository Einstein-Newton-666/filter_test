#pragma once

#include <angles/angles.h>

#include <Eigen/Dense>
#include <cmath>

namespace armor_sim {

struct CorrectedPnPPose {
    Eigen::Quaterniond orientation;
    double yaw = 0.0;
};

// ZYX 欧拉角提取 (R = Rz(yaw) * Ry(pitch) * Rx(roll)):
//   pitch = asin(-R20)
//   roll  = atan2(R21, R22)
//   yaw   = atan2(R10, R00)
inline Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& R) {
    double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
    double roll = std::atan2(R(2, 1), R(2, 2));
    double yaw = std::atan2(R(1, 0), R(0, 0));
    return {roll, pitch, yaw};
}

inline double yawFromRotation(const Eigen::Matrix3d& R) {
    return rotationMatrixToRPY(R).z();
}

// 平面 PnP 双解修正.
//
// 对共面目标, IPPE/平面 PnP 可能给出两个重投影误差非常接近的姿态候选。
// 对当前装甲板坐标约定, 错误候选常表现为 yaw 与期望 yaw 相差约 π。
//
// 判据:
//   e = wrap(yaw_pnp - yaw_expected) ∈ [-π, π]
//   if |e| > π/2, 选择另一个平面候选近似分支:
//       R'   = R * Rz(π)
//       yaw' = wrap(yaw + π)
//
// 这里的 expected_yaw 来自仿真真值 armor_yaw；实车/非仿真场景应使用上一帧
// 连续 yaw 或 tracker 预测 yaw, 不能用不可获得的真值。
inline CorrectedPnPPose correctPlanarPnPAmbiguity(
    const Eigen::Quaterniond& orientation, double expected_yaw) {
    CorrectedPnPPose corrected{orientation.normalized(), yawFromRotation(orientation.toRotationMatrix())};
    if (std::abs(angles::shortest_angular_distance(corrected.yaw, expected_yaw)) > M_PI_2) {
        corrected.orientation = (corrected.orientation *
            Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()))).normalized();
        corrected.yaw = angles::normalize_angle(corrected.yaw + M_PI);
    }
    return corrected;
}

}  // namespace armor_sim

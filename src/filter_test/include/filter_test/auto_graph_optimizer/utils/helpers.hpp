#pragma once

#include "types.hpp"
#include <ceres/jet.h>
#include <Eigen/Dense>

namespace auto_graph {

// 辅助函数：将Eigen向量转换为GTSAM Point3
inline gtsam::Point3 eigenToPoint3(const Eigen::Vector3d& v) {
    return gtsam::Point3(v.x(), v.y(), v.z());
}

// 辅助函数：将GTSAM Point3转换为Eigen向量
inline Eigen::Vector3d point3ToEigen(const gtsam::Point3& p) {
    return Eigen::Vector3d(p.x(), p.y(), p.z());
}

// 辅助函数：将Eigen四元数转换为GTSAM Rot3
inline gtsam::Rot3 eigenToRot3(const Eigen::Quaterniond& q) {
    return gtsam::Rot3(q.w(), q.x(), q.y(), q.z());
}

// 辅助函数：创建对角噪声模型
inline gtsam::SharedNoiseModel diagonalNoise(const Eigen::VectorXd& sigmas) {
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

// 辅助函数：创建各向同性噪声模型
inline gtsam::SharedNoiseModel isotropicNoise(int dim, double sigma) {
    return gtsam::noiseModel::Isotropic::Sigma(dim, sigma);
}

// 辅助函数：计算两个角度之间的最短距离
inline double shortestAngularDistance(double from, double to) {
    double diff = to - from;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

// 辅助函数：将角度规范化到[-pi, pi]
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 辅助函数：logistic函数（用于有界参数）
inline double logisticFunction(double x, double min_val, double max_val) {
    return min_val + (max_val - min_val) / (1.0 + std::exp(-x));
}

// 辅助函数：logistic函数的导数
inline double logisticDerivative(double y, double min_val, double max_val) {
    double s = (y - min_val) / (max_val - min_val);
    return (max_val - min_val) * s * (1.0 - s);
}

// Logistic 模板版本 (兼容 ceres::Jet autodiff)
// 注意: 不能直接 T(constant), 会创建零维度 Jet 导致 Eigen 断言
template<typename T>
T logistic(T x, double min_val, double max_val) {
    T zero  = x * 0.0;                          // Jet v 与 x 同维度
    T range = zero + (max_val - min_val);
    T one   = zero + 1.0;
    T offset = zero + min_val;
    using ceres::exp;
    return offset + range / (one + exp(x * -1.0));
}

template<typename T>
T logisticDerivative(T y, double min_val, double max_val) {
    T zero = y * 0.0;
    T s = (y - T(min_val)) / (zero + (max_val - min_val));
    T one = zero + 1.0;
    return (zero + (max_val - min_val)) * s * (one - s);
}

// 辅助函数：logistic函数的逆
inline double logisticInverse(double y, double min_val, double max_val) {
    return -std::log((max_val - y) / (y - min_val));
}

} // namespace auto_graph

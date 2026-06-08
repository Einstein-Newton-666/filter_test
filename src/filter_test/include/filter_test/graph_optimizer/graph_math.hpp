#pragma once

#include <Eigen/Dense>
#include <ceres/jet.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>

#include <cmath>

namespace auto_graph {

// 通用数学工具刻意放在 graph_optimizer 外层，jlu_tracker 也可以复用，
// 避免两个 tracker 各自维护角度包裹和 logistic 半径编码。

// 将 Eigen 向量转换为 GTSAM Point3。
inline gtsam::Point3 eigenToPoint3(const Eigen::Vector3d& v) {
    return gtsam::Point3(v.x(), v.y(), v.z());
}

// 将 GTSAM Point3 转换为 Eigen 向量。
inline Eigen::Vector3d point3ToEigen(const gtsam::Point3& p) {
    return Eigen::Vector3d(p.x(), p.y(), p.z());
}

// 将 Eigen 四元数转换为 GTSAM Rot3。
inline gtsam::Rot3 eigenToRot3(const Eigen::Quaterniond& q) {
    return gtsam::Rot3(q.w(), q.x(), q.y(), q.z());
}

// 创建对角噪声模型，输入是标准差而不是方差。
inline gtsam::SharedNoiseModel diagonalNoise(const Eigen::VectorXd& sigmas) {
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

// 创建各向同性噪声模型，输入是标准差而不是方差。
inline gtsam::SharedNoiseModel isotropicNoise(int dim, double sigma) {
    return gtsam::noiseModel::Isotropic::Sigma(dim, sigma);
}

// 计算 from 到 to 的最短角距离，返回值范围为 [-pi, pi]。
inline double shortestAngularDistance(double from, double to) {
    double diff = to - from;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

// 将角度规范化到 [-pi, pi]。
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// logistic 有界编码：把无界优化变量映射到 [min_val, max_val]。
// 半径这类必须为正且有物理范围的静态参数用该编码优化。
inline double logisticFunction(double x, double min_val, double max_val) {
    return min_val + (max_val - min_val) / (1.0 + std::exp(-x));
}

// logisticFunction 对无界状态 x 的导数。这里传入的是已解码的 y,
// 方便 factor 中复用当前半径值计算雅可比。
inline double logisticDerivative(double y, double min_val, double max_val) {
    double s = (y - min_val) / (max_val - min_val);
    return (max_val - min_val) * s * (1.0 - s);
}

// Logistic 模板版本，保留给仍使用 ceres::Jet 的代码路径。
// 注意：不能直接 T(constant)，否则会创建零维度 Jet 导致 Eigen 断言。
template<typename T>
T logistic(T x, double min_val, double max_val) {
    T zero  = x * 0.0;                          // Jet v 与 x 同维度
    T range = zero + (max_val - min_val);
    T one   = zero + 1.0;
    T offset = zero + min_val;
    using ceres::exp;
    return offset + range / (one + exp(x * -1.0));
}

// logisticDerivative 的模板版本，和上面的 Jet 维度约束保持一致。
template<typename T>
T logisticDerivative(T y, double min_val, double max_val) {
    T zero = y * 0.0;
    T s = (y - T(min_val)) / (zero + (max_val - min_val));
    T one = zero + 1.0;
    return (zero + (max_val - min_val)) * s * (one - s);
}

// logisticFunction 的逆：把物理半径初始化成无界优化状态。
inline double logisticInverse(double y, double min_val, double max_val) {
    return -std::log((max_val - y) / (y - min_val));
}

} // namespace auto_graph

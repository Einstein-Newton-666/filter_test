#pragma once

#include <Eigen/Dense>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>

#include <string>
#include <cstdint>
#include <unordered_map>
#include <memory>
#include <vector>
#include <chrono>

namespace auto_graph {

// 变量类型枚举
enum class VariableType {
    SCALAR,     // double
    VECTOR2,    // Eigen::Vector2d
    VECTOR3,    // Eigen::Vector3d
    ROT2,       // gtsam::Rot2
    ROT3,       // gtsam::Rot3
    POSE3,      // gtsam::Pose3
    POINT3,     // gtsam::Point3
    MATRIX      // Eigen::MatrixXd (通用)
};

// 变量ID
struct VariableId {
    std::string name;        // 变量名
    uint64_t frame_id;       // 帧ID（0表示全局变量）
    gtsam::Key gtsam_key;    // GTSAM的Key
    VariableType type;       // 变量类型

    bool is_global() const { return frame_id == 0; }

    bool operator==(const VariableId& other) const {
        return name == other.name && frame_id == other.frame_id;
    }
};

// 因子类型枚举
enum class FactorType {
    MOTION,           // 运动因子
    MEASURE,          // 观测因子
    PRIOR,            // 先验因子
    CUSTOM            // 自定义因子
};

// 因子描述符
struct FactorDescriptor {
    FactorType type;
    std::vector<VariableId> variables;  // 连接的变量
    gtsam::SharedNoiseModel noise;      // 噪声模型
    std::shared_ptr<gtsam::NonlinearFactor> factor;  // GTSAM因子（使用std::shared_ptr）
};

// 边缘化策略
enum class MarginalizationStrategy {
    NONE,              // 不边缘化
    AFTER_N_FRAMES,    // 保留最近N帧
    KEYFRAME_BASED,    // 基于关键帧
    AGE_BASED          // 基于时间年龄
};

struct MarginalizationPolicy {
    MarginalizationStrategy strategy = MarginalizationStrategy::NONE;
    int keep_frames = 100;
    double max_age_sec = 10.0;
};

// 帧状态
enum class FrameState {
    ACTIVE,     // 活跃帧
    MARGINALIZED, // 已边缘化
    REMOVED     // 已删除
};

// 帧信息
struct FrameInfo {
    uint64_t frame_id;
    std::chrono::system_clock::time_point timestamp;
    FrameState state;
    std::vector<VariableId> variables;
};

} // namespace auto_graph

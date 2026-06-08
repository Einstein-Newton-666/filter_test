#pragma once

#include "filter_test/graph_optimizer/armor_model.hpp"

#include <auto_aim_interfaces/msg/armors.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Geometry>
#include <rclcpp/time.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace filter_test::graph_optimizer {

// tracker 每帧输入。ROS 相关 TF 和 dt 已经在 node/tracker 调用层处理完毕。
struct TrackerFrameInput {
    const auto_aim_interfaces::msg::Armors& armors_msg;
    double dt = 0.01;
    Eigen::Isometry3d T_camera_to_odom = Eigen::Isometry3d::Identity();
};

// tracker 对 ROS 发布层的返回值。solve/cold_start/failed 分开表达，
// 便于节点在冷启动和失败恢复时保持外部 topic 行为兼容。
struct TrackerUpdateResult {
    bool accepted_frame = false;
    bool initialized = false;
    bool solved = false;
    bool cold_start = false;
    bool solve_failed = false;
    std::string solve_error;
    uint64_t frame_id = 0;
    TrackerState state;
    std::vector<PredictedArmor> predicted_armors;
    std::vector<int> matched_indices;
};

// 只在 accepted frame 后 commit 时间戳，避免空观测或拒绝帧污染下一帧 dt。
class FrameTimeTracker {
public:
    double computeDt(const builtin_interfaces::msg::Time& stamp) const;
    void commit(const builtin_interfaces::msg::Time& stamp);

private:
    bool have_last_time_ = false;
    rclcpp::Time last_time_;
};

// ArmorGraphTracker 是 ROS tracker 的薄调用层：负责空观测处理、失败恢复和结果封装，
// 真正的图生命周期由 ArmorCvPixelGraph 管理。
class ArmorGraphTracker {
public:
    explicit ArmorGraphTracker(TrackerConfig config);

    TrackerUpdateResult update(const TrackerFrameInput& input);

    const TrackerState& state() const { return graph_.state(); }
    bool initialized() const { return graph_.initialized(); }
    uint64_t frameId() const;

private:
    void initialize(const auto_aim_interfaces::msg::Armors& msg);
    void reset();
    TrackerUpdateResult makeResult(bool accepted_frame, bool solved,
                                   std::vector<int> matched_indices) const;
    TrackerUpdateResult makeResult(bool accepted_frame,
                                   const auto_graph::SolveResult& solve_result,
                                   std::vector<int> matched_indices) const;

    ArmorCvPixelGraph graph_;
};

}  // namespace filter_test::graph_optimizer

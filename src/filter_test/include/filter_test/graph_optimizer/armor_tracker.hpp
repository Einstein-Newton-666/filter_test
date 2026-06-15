#pragma once

#include "filter_test/graph_optimizer/armor_model.hpp"

#include <auto_aim_interfaces/msg/armors.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Geometry>
#include <rclcpp/time.hpp>

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace filter_test::graph_optimizer {

// tracker 每帧输入。ROS 相关 TF 和 dt 已经在 node/tracker 调用层处理完毕。
// frontend_state 是可选前端估计：图优化会把它作为当前帧初值，并可按配置
// 添加弱 prior；原始装甲板观测仍会照常写入图中参与优化。
struct TrackerFrameInput {
    TrackerFrameInput(
        const auto_aim_interfaces::msg::Armors& armors,
        double frame_dt = 0.01,
        Eigen::Isometry3d camera_to_odom = Eigen::Isometry3d::Identity(),
        std::optional<TrackerState> frontend = std::nullopt)
        : armors_msg(armors),
          dt(frame_dt),
          T_camera_to_odom(std::move(camera_to_odom)),
          frontend_state(std::move(frontend)) {}

    const auto_aim_interfaces::msg::Armors& armors_msg;
    double dt = 0.01;
    Eigen::Isometry3d T_camera_to_odom = Eigen::Isometry3d::Identity();
    std::optional<TrackerState> frontend_state;
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
    std::string reset_reason;
    uint64_t frame_id = 0;
    TrackerState state;
    std::vector<PredictedArmor> predicted_armors;
    std::vector<int> matched_indices;
    std::vector<double> match_costs;
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
    void initialize(
        const auto_aim_interfaces::msg::Armors& msg,
        const TrackerState* frontend_state = nullptr);
    void reset();
    void rememberOutpostState(const TrackerState& state);
    void updateMatchQualityWindow(const std::vector<int>& matched_indices,
                                  const std::vector<double>& match_costs);
    bool matchQualityFailed() const;
    TrackerUpdateResult makeResult(bool accepted_frame, bool solved,
                                   std::vector<int> matched_indices) const;
    TrackerUpdateResult makeResult(bool accepted_frame,
                                   const auto_graph::SolveResult& solve_result,
                                   std::vector<int> matched_indices) const;

    ArmorCvPixelGraph graph_;
    TrackerConfig config_;
    std::deque<bool> match_quality_failures_;
    TrackerState last_outpost_state_;
    bool has_last_outpost_state_ = false;
};

}  // namespace filter_test::graph_optimizer

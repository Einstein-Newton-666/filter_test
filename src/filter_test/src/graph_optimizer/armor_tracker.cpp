#include "filter_test/graph_optimizer/armor_tracker.hpp"

#include <algorithm>
#include <utility>

namespace filter_test::graph_optimizer {

double FrameTimeTracker::computeDt(const builtin_interfaces::msg::Time& stamp) const {
    rclcpp::Time current_time(stamp);
    if (!have_last_time_) return 0.01;

    // 异常时间戳会直接破坏运动因子中的 dt，因此这里给一个保守默认值。
    // commit() 只应在有效观测帧后调用，空帧不会推进 last_time_。
    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
        dt = 0.01;
    }
    return dt;
}

void FrameTimeTracker::commit(const builtin_interfaces::msg::Time& stamp) {
    last_time_ = rclcpp::Time(stamp);
    have_last_time_ = true;
}

ArmorGraphTracker::ArmorGraphTracker(TrackerConfig config)
    : graph_(std::move(config)) {}

TrackerUpdateResult ArmorGraphTracker::update(const TrackerFrameInput& input) {
    if (input.armors_msg.armors.empty()) {
        // 空观测不推进图，也不推进时间戳；调用层可以继续发布上一帧状态或进入丢失逻辑。
        return makeResult(false, false, {});
    }

    // tracker 只负责输入门控和结果封装；图模型由 ArmorCvPixelGraph 执行。
    if (!graph_.initialized()) {
        // 第一帧只建立 frame 0 先验图。没有当前帧运动边，因此返回 accepted 但 solved=false。
        initialize(input.armors_msg);
        return makeResult(true, false, {});
    }

    auto output =
        graph_.update(input.armors_msg, input.dt, input.T_camera_to_odom);
    // graph 输出已经包含冷启动、失败和优化成功三种情况的状态选择。
    const bool all_unmatched = !output.matched_indices.empty() &&
        std::none_of(output.matched_indices.begin(), output.matched_indices.end(),
                     [](int idx) { return idx >= 0; });
    if (all_unmatched) {
        auto unmatched = std::move(output.matched_indices);
        reset();
        return makeResult(false, false, std::move(unmatched));
    }
    if (output.solve_result.failed) {
        // 求解失败时不丢弃 tracker 生命周期，只把错误标志传给发布层。
        return makeResult(true, output.solve_result, std::move(output.matched_indices));
    }
    TrackerUpdateResult result =
        makeResult(true, output.solve_result, std::move(output.matched_indices));
    result.state = output.state;
    result.predicted_armors = std::move(output.predicted_armors);
    return result;
}

uint64_t ArmorGraphTracker::frameId() const {
    return graph_.frameId();
}

void ArmorGraphTracker::initialize(const auto_aim_interfaces::msg::Armors& msg) {
    graph_.initialize(msg);
}

void ArmorGraphTracker::reset() {
    graph_.reset();
}

TrackerUpdateResult ArmorGraphTracker::makeResult(
    bool accepted_frame, bool solved, std::vector<int> matched_indices) const {
    TrackerUpdateResult result;
    result.accepted_frame = accepted_frame;
    result.initialized = graph_.initialized();
    result.solved = solved;
    result.frame_id = frameId();
    result.state = graph_.state();
    // 即使 solve=false，也给出基于当前状态的预测装甲板，保持 marker/target 输出连续。
    result.predicted_armors = predictedArmorsFromState(graph_.state());
    result.matched_indices = std::move(matched_indices);
    return result;
}

TrackerUpdateResult ArmorGraphTracker::makeResult(
    bool accepted_frame, const auto_graph::SolveResult& solve_result,
    std::vector<int> matched_indices) const {
    TrackerUpdateResult result = makeResult(
        accepted_frame, !solve_result.failed, std::move(matched_indices));
    result.cold_start = solve_result.cold_start;
    result.solve_failed = solve_result.failed;
    result.solve_error = solve_result.error_message;
    return result;
}

}  // namespace filter_test::graph_optimizer

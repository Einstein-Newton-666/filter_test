#include "filter_test/graph_optimizer/armor_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
    : graph_(config),
      config_(std::move(config)) {}

TrackerUpdateResult ArmorGraphTracker::update(const TrackerFrameInput& input) {
    if (input.armors_msg.armors.empty()) {
        // 空观测不推进图，也不推进时间戳；调用层可以继续发布上一帧状态或进入丢失逻辑。
        return makeResult(false, false, {});
    }

    // tracker 只负责输入门控和结果封装；图模型由 ArmorCvPixelGraph 执行。
    if (!graph_.initialized()) {
        // 第一帧只建立 frame 0 先验图。没有当前帧运动边，因此返回 accepted 但 solved=false。
        initialize(
            input.armors_msg,
            input.frontend_state ? &(*input.frontend_state) : nullptr);
        return makeResult(true, false, {});
    }

    auto output =
        graph_.update(
            input.armors_msg, input.dt, input.T_camera_to_odom,
            input.frontend_state ? &(*input.frontend_state) : nullptr);
    // graph 输出已经包含冷启动、失败和优化成功三种情况的状态选择。
    const bool all_unmatched = !output.matched_indices.empty() &&
        std::none_of(output.matched_indices.begin(), output.matched_indices.end(),
                     [](int idx) { return idx >= 0; });
    const bool has_matched = !output.matched_indices.empty() &&
        std::any_of(output.matched_indices.begin(), output.matched_indices.end(),
                    [](int idx) { return idx >= 0; });
    if (has_matched && output.solve_result.optimized) {
        rememberOutpostState(output.state);
    }
    if (all_unmatched) {
        if (output.state.armor_count != 3) {
            auto unmatched = std::move(output.matched_indices);
            reset();
            auto result = makeResult(false, false, std::move(unmatched));
            result.reset_reason = "all observations unmatched";
            return result;
        }
    }
    if (output.solve_result.optimized) {
        updateMatchQualityWindow(output.matched_indices, output.match_costs);
        if (output.state.armor_count == 3) {
            // 前哨站一帧通常只有一块板。短时错配/模糊时如果直接 reset，
            // 下一帧会把当前观测板重新初始化成 slot0，导致 center.z 换槽跳变。
            match_quality_failures_.clear();
        } else if (matchQualityFailed()) {
            auto matched = std::move(output.matched_indices);
            reset();
            auto result = makeResult(false, false, std::move(matched));
            result.reset_reason = "match quality window failed";
            return result;
        }
    }
    if (output.solve_result.failed) {
        // 求解失败时不丢弃 tracker 生命周期，只把错误标志传给发布层。
        return makeResult(true, output.solve_result, std::move(output.matched_indices));
    }
    TrackerUpdateResult result =
        makeResult(true, output.solve_result, std::move(output.matched_indices));
    result.state = output.state;
    result.predicted_armors = std::move(output.predicted_armors);
    result.match_costs = std::move(output.match_costs);
    return result;
}

uint64_t ArmorGraphTracker::frameId() const {
    return graph_.frameId();
}

void ArmorGraphTracker::initialize(
    const auto_aim_interfaces::msg::Armors& msg,
    const TrackerState* frontend_state) {
    const bool outpost = std::any_of(
        msg.armors.begin(), msg.armors.end(),
        [](const auto_aim_interfaces::msg::Armor& armor) {
            return armor.number == "outpost";
        });
    graph_.initialize(
        msg, outpost && has_last_outpost_state_ ? &last_outpost_state_ : nullptr,
        frontend_state);
}

void ArmorGraphTracker::reset() {
    graph_.reset();
    match_quality_failures_.clear();
}

void ArmorGraphTracker::rememberOutpostState(const TrackerState& state) {
    if (state.armor_count != 3) return;
    last_outpost_state_ = state;
    has_last_outpost_state_ = true;
}

void ArmorGraphTracker::updateMatchQualityWindow(
    const std::vector<int>& matched_indices,
    const std::vector<double>& match_costs) {
    if (config_.match_quality_window_size <= 0) return;

    double worst_cost = 0.0;
    bool has_match = false;
    for (std::size_t i = 0; i < matched_indices.size() && i < match_costs.size(); ++i) {
        if (matched_indices[i] < 0 || !std::isfinite(match_costs[i])) continue;
        worst_cost = std::max(worst_cost, match_costs[i]);
        has_match = true;
    }
    if (!has_match) return;

    match_quality_failures_.push_back(
        worst_cost > config_.match_quality_failure_threshold);
    while (match_quality_failures_.size() >
           static_cast<std::size_t>(config_.match_quality_window_size)) {
        match_quality_failures_.pop_front();
    }
}

bool ArmorGraphTracker::matchQualityFailed() const {
    if (config_.match_quality_window_size <= 0) return false;
    if (match_quality_failures_.size() <
        static_cast<std::size_t>(config_.match_quality_window_size)) {
        return false;
    }
    const int failure_count = static_cast<int>(std::count(
        match_quality_failures_.begin(), match_quality_failures_.end(), true));
    const double ratio = static_cast<double>(failure_count) /
        static_cast<double>(match_quality_failures_.size());
    return ratio > config_.match_quality_failure_ratio;
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

#include "filter_test/graph_optimizer/graph_core.hpp"

#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

#include <iostream>
#include <stdexcept>

namespace auto_graph {

GraphOptimizer::GraphOptimizer(const GraphOptimizerConfig& cfg)
    : config_(cfg) {}

void GraphOptimizer::beginInit() {
    graph_.resize(0);
    initial_values_.clear();
    latest_estimate_.clear();
    frame_id_ = 0;
    smootherReset();
    initialized_ = false;
    cold_start_count_ = 0;
}

void GraphOptimizer::finishInit() {
    latest_estimate_ = initial_values_;
    initialized_ = true;
    cold_start_count_ = 0;
}

uint64_t GraphOptimizer::beginFrame() {
    if (!initialized_) {
        throw std::runtime_error("GraphOptimizer::beginFrame: not initialized");
    }
    // frame_id_ 从 1 开始表示第一帧增量，frame 0 是 initialize 阶段的先验状态。
    frame_id_++;
    return frame_id_;
}

SolveResult GraphOptimizer::solve() {
    SolveResult result;
    result.frame_id = frame_id_;
    if (!initialized_ || graph_.empty()) return result;

    // 冷启动阶段只累计初值和因子，不把不稳定的早期约束提交给 iSAM2。
    // tracker 仍可使用预测状态发布兼容输出。
    cold_start_count_++;
    if (config_.cold_start_frames > 0 &&
        cold_start_count_ <= config_.cold_start_frames) {
        result.cold_start = true;
        if (config_.verbose) {
            std::cout << "[GO] cold start " << cold_start_count_
                      << "/" << config_.cold_start_frames << std::endl;
        }
        return result;
    }

    if (cold_start_count_ == config_.cold_start_frames + 1 && config_.verbose) {
        std::cout << "[GO] cold start done, submitting " << graph_.size()
                  << " factors" << std::endl;
    }

    try {
        result.attempted = true;
        ensureSmoother();
        if (config_.smoother_lag <= 0.0) {
            // smoother_lag <= 0 时使用普通 iSAM2，所有历史变量由 iSAM2 自己维护。
            isam2_->update(graph_, initial_values_);
            for (int i = 1; i < config_.update_iterations; ++i) {
                isam2_->update();
            }
            latest_estimate_ = isam2_->calculateEstimate();
        } else {
            // fixed-lag smoother 需要每个新 key 的时间戳。静态 key 也刷新到当前帧，
            // 这样半径/dz 这类长期变量不会因为时间戳过旧被边缘化。
            gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
            for (const auto& [key, value] : initial_values_) {
                (void)value;
                timestamps[key] = static_cast<double>(frame_id_);
            }
            for (const auto key : staticKeys()) {
                timestamps[key] = static_cast<double>(frame_id_);
            }
            smoother().update(graph_, initial_values_, timestamps);
            for (int i = 1; i < config_.update_iterations; ++i) {
                smoother().update();
            }
            latest_estimate_ = smoother().calculateEstimate();
        }

        // 提交成功后清空本轮增量；latest_estimate_ 保留全局可查询结果。
        graph_.resize(0);
        initial_values_.clear();
        result.optimized = true;

        if (config_.verbose) {
            std::cout << "[GO] solved typed f=" << frame_id_
                      << " values=" << latest_estimate_.size() << std::endl;
        }
    } catch (const std::exception& e) {
        result.attempted = true;
        result.failed = true;
        result.error_message = e.what();
        std::cerr << "[GO] error f=" << frame_id_ << ": " << e.what() << std::endl;
        // 失败时丢弃本轮增量，避免同一批坏因子在下一帧重复触发异常。
        // latest_estimate_ 不清空，调用方仍可使用上一轮成功估计。
        graph_.resize(0);
        initial_values_.clear();
    }

    result.frame_id = frame_id_;
    return result;
}

uint64_t GraphOptimizer::getFrameId() const {
    return frame_id_;
}

void GraphOptimizer::reset() {
    graph_.resize(0);
    initial_values_.clear();
    latest_estimate_.clear();
    smootherReset();
    frame_id_ = 0;
    initialized_ = false;
    cold_start_count_ = 0;
}

const GraphOptimizer::Entry& GraphOptimizer::at(const std::string& name) const {
    auto it = entries_.find(name);
    if (it == entries_.end()) {
        throw std::runtime_error("GraphOptimizer: unknown variable '" + name + "'");
    }
    return it->second;
}

gtsam::Key GraphOptimizer::key(const std::string& name, uint64_t frame_id) const {
    const auto& entry = at(name);
    return gtsam::Symbol(entry.prefix, entry.is_static ? 0 : frame_id);
}

std::vector<gtsam::Key> GraphOptimizer::staticKeys() const {
    std::vector<gtsam::Key> keys;
    for (const auto& [name, entry] : entries_) {
        (void)name;
        if (entry.is_static) {
            keys.push_back(gtsam::Symbol(entry.prefix, 0));
        }
    }
    return keys;
}

void GraphOptimizer::ensureSmoother() {
    if (isam2_ || inc_smoother_ || batch_smoother_) return;

    // 三种后端互斥懒创建。这样 initialize/reset 后不会保留旧 smoother 内部状态。
    if (config_.smoother_lag <= 0.0) {
        gtsam::ISAM2Params p;
        p.relinearizeThreshold = config_.relinearize_threshold;
        p.relinearizeSkip = config_.relinearize_skip;
        p.enablePartialRelinearizationCheck = true;
        p.findUnusedFactorSlots = true;
        isam2_ = std::make_unique<gtsam::ISAM2>(p);
    } else if (config_.smoother_type == SmootherType::Incremental) {
        gtsam::ISAM2Params p;
        p.relinearizeThreshold = config_.relinearize_threshold;
        p.relinearizeSkip = config_.relinearize_skip;
        p.enablePartialRelinearizationCheck = true;
        p.findUnusedFactorSlots = true;
        inc_smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
            config_.smoother_lag, p);
    } else {
        gtsam::LevenbergMarquardtParams lm;
        lm.setMaxIterations(10);
        lm.setRelativeErrorTol(1e-5);
        lm.setAbsoluteErrorTol(1e-5);
        batch_smoother_ = std::make_unique<gtsam::BatchFixedLagSmoother>(
            config_.smoother_lag, lm);
    }
}

gtsam::FixedLagSmoother& GraphOptimizer::smoother() {
    if (inc_smoother_) return *inc_smoother_;
    return *batch_smoother_;
}

void GraphOptimizer::smootherReset() {
    isam2_.reset();
    inc_smoother_.reset();
    batch_smoother_.reset();
}

}  // namespace auto_graph

#pragma once

#include "filter_test/graph_optimizer/observation_backend.hpp"
#include "filter_test/graph_optimizer/tracker_math.hpp"

#include <memory>

namespace filter_test::graph_optimizer {

class ArmorGraphTracker {
public:
    explicit ArmorGraphTracker(TrackerConfig config);

    TrackerUpdateResult update(const TrackerFrameInput& input);

    const TrackerState& state() const { return state_; }
    const Eigen::VectorXd& stateVector() const { return state_vector_; }
    bool initialized() const { return initialized_; }
    uint64_t frameId() const;

private:
    void initialize(const auto_aim_interfaces::msg::Armors& msg);
    void addMotionFactors(double dt);
    std::vector<int> matchArmors(const auto_aim_interfaces::msg::Armors& msg);
    TrackerUpdateResult makeResult(bool accepted_frame, bool solved,
                                   std::vector<int> matched_indices) const;
    TrackerUpdateResult makeResult(bool accepted_frame, const auto_graph::SolveResult& solve_result,
                                   std::vector<int> matched_indices) const;

    TrackerConfig config_;
    auto_graph::GraphOptimizer optimizer_;
    std::unique_ptr<ObservationBackend> observation_backend_;
    TrackerState state_;
    Eigen::VectorXd state_vector_;
    bool initialized_ = false;
    int last_armor_index_ = -1;
};

}  // namespace filter_test::graph_optimizer

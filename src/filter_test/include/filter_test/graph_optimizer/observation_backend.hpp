#pragma once

#include "filter_test/auto_graph_optimizer/graph_optimizer.hpp"
#include "filter_test/graph_optimizer/tracker_types.hpp"

#include <memory>
#include <string>

namespace filter_test::graph_optimizer {

struct TrackerFrameContext {
    const TrackerConfig& config;
    const auto_aim_interfaces::msg::Armors& armors_msg;
    const Eigen::Isometry3d& T_camera_to_odom;
    const Eigen::VectorXd& predicted_state;
    const std::vector<int>& matched_indices;
    uint64_t frame_id = 0;
};

struct AddFrameFactorsResult {
    bool ok = true;
    std::string error_message;
};

class ObservationBackend {
public:
    virtual ~ObservationBackend() = default;

    virtual const char* name() const = 0;

    AddFrameFactorsResult addFrameFactors(auto_graph::GraphOptimizer& optimizer,
                                          const TrackerFrameContext& context) {
        if (context.matched_indices.size() != context.armors_msg.armors.size()) {
            return {false, "matched_indices size does not match armors size"};
        }
        return addFrameFactorsImpl(optimizer, context);
    }

private:
    virtual AddFrameFactorsResult addFrameFactorsImpl(
        auto_graph::GraphOptimizer& optimizer,
        const TrackerFrameContext& context) = 0;
};

std::unique_ptr<ObservationBackend> createObservationBackend(const TrackerConfig& config);

}  // namespace filter_test::graph_optimizer

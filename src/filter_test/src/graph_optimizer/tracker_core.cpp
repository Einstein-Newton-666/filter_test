#include "filter_test/graph_optimizer/tracker_core.hpp"

#include "filter_test/graph_optimizer/motion_models.hpp"

#include <cmath>
#include <utility>

namespace filter_test::graph_optimizer {

ArmorGraphTracker::ArmorGraphTracker(TrackerConfig config)
    : config_(std::move(config)),
      optimizer_(config_.optimizer),
      observation_backend_(createObservationBackend(config_)) {}

TrackerUpdateResult ArmorGraphTracker::update(const TrackerFrameInput& input) {
    if (input.armors_msg.armors.empty()) {
        return makeResult(false, false, {});
    }

    if (!initialized_) {
        initialize(input.armors_msg);
        initialized_ = true;
        return makeResult(true, false, {});
    }

    addMotionFactors(input.dt);
    std::vector<int> matched_indices = matchArmors(input.armors_msg);

    TrackerFrameContext context{
        config_,
        input.armors_msg,
        input.T_camera_to_odom,
        optimizer_.getState(),
        matched_indices,
        optimizer_.getFrameId()};
    auto add_result = observation_backend_->addFrameFactors(optimizer_, context);
    if (!add_result.ok) {
        auto_graph::SolveResult failed_result;
        failed_result.frame_id = optimizer_.getFrameId();
        failed_result.failed = true;
        failed_result.error_message = add_result.error_message;
        return makeResult(true, failed_result, std::move(matched_indices));
    }

    auto solve_result = optimizer_.solve();
    if (solve_result.failed) {
        return makeResult(true, solve_result, std::move(matched_indices));
    }
    state_vector_ = optimizer_.getState();
    state_ = stateFromVector(state_vector_);
    return makeResult(true, solve_result, std::move(matched_indices));
}

uint64_t ArmorGraphTracker::frameId() const {
    return initialized_ ? optimizer_.getFrameId() : 0;
}

void ArmorGraphTracker::initialize(const auto_aim_interfaces::msg::Armors& msg) {
    auto layout = auto_graph::VariableLayout(11)
        .addDynamic("pos_vel",  {0, 1, 2, 3, 4, 5})
        .addDynamic("yaw_vyaw", {6, 7})
        .addStatic ("radius",   {8, 9})
        .addStatic ("dz",       {10});

    const auto& armor = msg.armors.front();
    const double init_r_phys = 0.25;
    const double init_r_u = radiusToState(init_r_phys);

    last_armor_index_ = 0;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(11);
    x0[0] = armor.pose.position.x + init_r_phys * std::cos(armor.yaw);
    x0[1] = 0.0;
    x0[2] = armor.pose.position.y + init_r_phys * std::sin(armor.yaw);
    x0[3] = 0.0;
    x0[4] = armor.pose.position.z;
    x0[5] = 0.0;
    x0[6] = armor.yaw;
    x0[7] = 0.0;
    x0[8] = init_r_u;
    x0[9] = init_r_u;
    x0[10] = 0.0;

    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(11, 11);
    P0(0, 0) = P0(2, 2) = P0(4, 4) = 0.01;
    P0(1, 1) = P0(3, 3) = P0(5, 5) = P0(7, 7) = 0.25;
    P0(6, 6) = 0.05;
    P0(8, 8) = P0(9, 9) = 0.25;
    P0(10, 10) = 0.25;

    optimizer_.initialize(layout, x0, P0);
    state_vector_ = x0;
    state_ = stateFromVector(state_vector_);
}

void ArmorGraphTracker::addMotionFactors(double dt) {
    if (config_.use_2d_observation) {
        optimizer_.advanceFrame(dt);

        Eigen::MatrixXd Qt = Eigen::MatrixXd::Zero(6, 6);
        Qt(0, 0) = config_.s2qxy;
        Qt(1, 1) = config_.s2qvel;
        Qt(2, 2) = config_.s2qxy;
        Qt(3, 3) = config_.s2qvel;
        Qt(4, 4) = config_.s2qz;
        Qt(5, 5) = config_.s2qvel;
        optimizer_.addMotionFactor<TranslationModel>("pos_vel", TranslationModel(dt), Qt);

        Eigen::MatrixXd Qy = Eigen::MatrixXd::Zero(2, 2);
        Qy(0, 0) = config_.s2qyaw;
        Qy(1, 1) = config_.s2qvyaw;
        optimizer_.addMotionFactor<YawModel>("yaw_vyaw", YawModel(dt), Qy);
        state_vector_ = optimizer_.getState();
        state_ = stateFromVector(state_vector_);
        return;
    }

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
    const double t = dt;
    const double q_xx = std::pow(t, 4) / 4.0 * config_.s2qxy;
    const double q_xv = std::pow(t, 3) / 2.0 * config_.s2qxy;
    const double q_vv = std::pow(t, 2) * config_.s2qxy;
    Q(0, 0) = q_xx;
    Q(0, 1) = q_xv;
    Q(1, 0) = q_xv;
    Q(1, 1) = q_vv;
    Q(2, 2) = q_xx;
    Q(2, 3) = q_xv;
    Q(3, 2) = q_xv;
    Q(3, 3) = q_vv;
    Q(4, 4) = std::pow(t, 4) / 4.0 * config_.s2qz;
    Q(4, 5) = std::pow(t, 3) / 2.0 * config_.s2qz;
    Q(5, 4) = Q(4, 5);
    Q(5, 5) = std::pow(t, 2) * config_.s2qz;
    Q(6, 6) = std::pow(t, 4) / 4.0 * config_.s2qyaw;
    Q(6, 7) = std::pow(t, 3) / 2.0 * config_.s2qyaw;
    Q(7, 6) = Q(6, 7);
    Q(7, 7) = std::pow(t, 2) * config_.s2qyaw;
    Q(8, 8) = std::pow(t, 4) / 4.0 * config_.s2qr;
    Q(9, 9) = Q(8, 8);
    Q(10, 10) = std::pow(t, 4) / 4.0 * config_.s2qdz;

    optimizer_.predict<ArmorCVMotionModel>(dt, Q);
    state_vector_ = optimizer_.getState();
    state_ = stateFromVector(state_vector_);
}

std::vector<int> ArmorGraphTracker::matchArmors(
    const auto_aim_interfaces::msg::Armors& msg) {
    std::vector<int> matched = matchArmorIndicesUnique(state_, msg, last_armor_index_);
    for (auto it = matched.rbegin(); it != matched.rend(); ++it) {
        if (*it >= 0) {
            last_armor_index_ = *it;
            break;
        }
    }
    return matched;
}

TrackerUpdateResult ArmorGraphTracker::makeResult(
    bool accepted_frame, bool solved, std::vector<int> matched_indices) const {
    TrackerUpdateResult result;
    result.accepted_frame = accepted_frame;
    result.initialized = initialized_;
    result.solved = solved;
    result.frame_id = frameId();
    result.state = state_;
    result.predicted_armors = predictedArmorsFromState(state_);
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

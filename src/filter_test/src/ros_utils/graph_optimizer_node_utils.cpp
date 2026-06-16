#include "filter_test/ros_utils/graph_optimizer_node_utils.hpp"
#include "filter_test/ros_utils/camera_info_utils.hpp"

#include <cmath>

namespace filter_test {

void declareTrackerParameters(rclcpp::Node& node) {
    node.declare_parameter("standard.s2qxy", 0.1);
    node.declare_parameter("standard.s2qz", 0.1);
    node.declare_parameter("standard.s2qyaw", 0.1);
    node.declare_parameter("standard.vel_sigma", 0.01);
    node.declare_parameter("standard.vyaw_sigma", 0.05);
    node.declare_parameter("outpost.s2qxy", 0.1);
    node.declare_parameter("outpost.s2qz", 0.0004);
    node.declare_parameter("outpost.s2qyaw", 0.1);
    node.declare_parameter("outpost.vel_sigma", 0.01);
    node.declare_parameter("outpost.vyaw_sigma", 0.05);
    node.declare_parameter("match_max_cost", graph_optimizer::kDefaultArmorMatchMaxCost);
    node.declare_parameter(
        "outpost.ambiguous_match_margin",
        graph_optimizer::kDefaultOutpostAmbiguousMatchMargin);
    node.declare_parameter("outpost.debug_height", false);
    node.declare_parameter("match_quality.window_size", 10);
    node.declare_parameter("match_quality.failure_threshold", 1.0);
    node.declare_parameter("match_quality.failure_ratio", 0.60);
    node.declare_parameter("standard.pixel_noise_std", 1.0);
    node.declare_parameter("outpost.pixel_noise_std", 2.0);
    node.declare_parameter("observation_noise_reference_distance", 5.0);
    node.declare_parameter("pixel_noise_distance_quadratic", 0.025);
    node.declare_parameter("pose_prior_sigma", 0.1);
    node.declare_parameter("pose_prior_distance_scale", 0.05);
    node.declare_parameter("use_edge_reproj_factor", true);
    node.declare_parameter("edge_reproj_sigma", 0.05);
    node.declare_parameter("edge_reproj_distance_scale", 0.10);
    node.declare_parameter("edge_loss_slope_k", 2.0);
    node.declare_parameter("standard.armor_pitch", 15.0 * M_PI / 180.0);
    node.declare_parameter("outpost.armor_pitch", -0.26);
    node.declare_parameter("outpost.radius", 0.2765);
    node.declare_parameter("outpost.initial_vyaw", 0.0);
    node.declare_parameter("standard.prior_sigma.radius", 1.0);
    node.declare_parameter("standard.prior_sigma.dz", 1.0);
    node.declare_parameter("outpost.prior_sigma.radius", 1.0);
    node.declare_parameter("outpost.prior_sigma.dz", 1.0);
    node.declare_parameter("standard.geo.tangential", 0.01);
    node.declare_parameter("standard.geo.radial", 0.03);
    node.declare_parameter("standard.geo.height", 0.01);
    node.declare_parameter("standard.geo.yaw", 0.005);
    node.declare_parameter("geo_tangential_distance_scale", 0.20);
    node.declare_parameter("geo_radial_distance_scale", 0.20);
    node.declare_parameter("geo_yaw_distance_scale", 0.10);
    node.declare_parameter("outpost.geo.tangential", 0.01);
    node.declare_parameter("outpost.geo.radial", 0.03);
    node.declare_parameter("outpost.geo.height", 0.05);
    node.declare_parameter("outpost.geo.yaw", 0.10);
    node.declare_parameter("frontend.prior.enabled", true);
    node.declare_parameter("frontend.prior.geometry_enabled", false);
    node.declare_parameter("frontend.prior.center_sigma", 0.20);
    node.declare_parameter("frontend.prior.velocity_sigma", 1.0);
    node.declare_parameter("frontend.prior.yaw_sigma", 0.25);
    node.declare_parameter("frontend.prior.vyaw_sigma", 1.0);
    node.declare_parameter("frontend.prior.outpost_base_sigma", 0.30);
    node.declare_parameter("frontend.prior.radius_sigma", 0.50);
    node.declare_parameter("frontend.prior.dz_sigma", 0.30);

    node.declare_parameter("camera_name", "narrow_stereo");
    node.declare_parameter(
        "camera_info_url",
        "package://armor_simulation/config/camera_info.yaml");

    node.declare_parameter("verbose", true);
    node.declare_parameter("cold_start_frames", 3);
    node.declare_parameter("smoother_lag", 0.0);
    node.declare_parameter("smoother_type", "incremental");
    node.declare_parameter("relinearize_threshold", 0.001);
    node.declare_parameter("extra_iterations", 2);
}

graph_optimizer::TrackerConfig loadTrackerConfigFromParameters(
    rclcpp::Node& node) {
    graph_optimizer::TrackerConfig config;
    config.s2qxy = node.get_parameter("standard.s2qxy").as_double();
    config.s2qz = node.get_parameter("standard.s2qz").as_double();
    config.s2qyaw = node.get_parameter("standard.s2qyaw").as_double();
    config.vel_sigma = node.get_parameter("standard.vel_sigma").as_double();
    config.vyaw_sigma = node.get_parameter("standard.vyaw_sigma").as_double();
    config.outpost_s2qxy = node.get_parameter("outpost.s2qxy").as_double();
    config.outpost_s2qz = node.get_parameter("outpost.s2qz").as_double();
    config.outpost_s2qyaw = node.get_parameter("outpost.s2qyaw").as_double();
    config.outpost_vel_sigma = node.get_parameter("outpost.vel_sigma").as_double();
    config.outpost_vyaw_sigma =
        node.get_parameter("outpost.vyaw_sigma").as_double();
    config.match_max_cost = node.get_parameter("match_max_cost").as_double();
    config.outpost_ambiguous_match_margin =
        node.get_parameter("outpost.ambiguous_match_margin").as_double();
    config.match_quality_window_size =
        node.get_parameter("match_quality.window_size").as_int();
    config.match_quality_failure_threshold =
        node.get_parameter("match_quality.failure_threshold").as_double();
    config.match_quality_failure_ratio =
        node.get_parameter("match_quality.failure_ratio").as_double();
    config.pixel_sigma =
        node.get_parameter("standard.pixel_noise_std").as_double();
    config.outpost_pixel_sigma =
        node.get_parameter("outpost.pixel_noise_std").as_double();
    config.observation_noise_reference_distance =
        node.get_parameter("observation_noise_reference_distance").as_double();
    config.pixel_sigma_distance_quadratic =
        node.get_parameter("pixel_noise_distance_quadratic").as_double();
    config.pose_prior_sigma = node.get_parameter("pose_prior_sigma").as_double();
    config.pose_prior_distance_scale =
        node.get_parameter("pose_prior_distance_scale").as_double();
    config.use_edge_reproj_factor =
        node.get_parameter("use_edge_reproj_factor").as_bool();
    config.edge_reproj_sigma =
        node.get_parameter("edge_reproj_sigma").as_double();
    config.edge_reproj_distance_scale =
        node.get_parameter("edge_reproj_distance_scale").as_double();
    config.edge_loss_slope_k =
        node.get_parameter("edge_loss_slope_k").as_double();
    config.standard_armor_pitch =
        node.get_parameter("standard.armor_pitch").as_double();
    config.outpost_armor_pitch =
        node.get_parameter("outpost.armor_pitch").as_double();
    config.outpost_radius = node.get_parameter("outpost.radius").as_double();
    config.outpost_initial_vyaw =
        node.get_parameter("outpost.initial_vyaw").as_double();
    config.prior_noise.radius =
        node.get_parameter("standard.prior_sigma.radius").as_double();
    config.prior_noise.dz =
        node.get_parameter("standard.prior_sigma.dz").as_double();
    config.outpost_prior_noise.radius =
        node.get_parameter("outpost.prior_sigma.radius").as_double();
    config.outpost_prior_noise.dz =
        node.get_parameter("outpost.prior_sigma.dz").as_double();
    config.geo_noise.tangential =
        node.get_parameter("standard.geo.tangential").as_double();
    config.geo_noise.radial =
        node.get_parameter("standard.geo.radial").as_double();
    config.geo_noise.height =
        node.get_parameter("standard.geo.height").as_double();
    config.geo_noise.yaw =
        node.get_parameter("standard.geo.yaw").as_double();
    config.geo_tangential_distance_scale =
        node.get_parameter("geo_tangential_distance_scale").as_double();
    config.geo_radial_distance_scale =
        node.get_parameter("geo_radial_distance_scale").as_double();
    config.geo_yaw_distance_scale =
        node.get_parameter("geo_yaw_distance_scale").as_double();
    config.outpost_geo_noise.tangential =
        node.get_parameter("outpost.geo.tangential").as_double();
    config.outpost_geo_noise.radial =
        node.get_parameter("outpost.geo.radial").as_double();
    config.outpost_geo_noise.height =
        node.get_parameter("outpost.geo.height").as_double();
    config.outpost_geo_noise.yaw =
        node.get_parameter("outpost.geo.yaw").as_double();
    config.frontend_prior.enabled =
        node.get_parameter("frontend.prior.enabled").as_bool();
    config.frontend_prior.geometry_enabled =
        node.get_parameter("frontend.prior.geometry_enabled").as_bool();
    config.frontend_prior.center_sigma =
        node.get_parameter("frontend.prior.center_sigma").as_double();
    config.frontend_prior.velocity_sigma =
        node.get_parameter("frontend.prior.velocity_sigma").as_double();
    config.frontend_prior.yaw_sigma =
        node.get_parameter("frontend.prior.yaw_sigma").as_double();
    config.frontend_prior.vyaw_sigma =
        node.get_parameter("frontend.prior.vyaw_sigma").as_double();
    config.frontend_prior.outpost_base_sigma =
        node.get_parameter("frontend.prior.outpost_base_sigma").as_double();
    config.frontend_prior.radius_sigma =
        node.get_parameter("frontend.prior.radius_sigma").as_double();
    config.frontend_prior.dz_sigma =
        node.get_parameter("frontend.prior.dz_sigma").as_double();

    const auto calibration = loadCameraCalibrationFromInfo(node);
    config.camera_matrix = calibration.camera_matrix;
    config.distortion = calibration.distortion;

    config.optimizer.cold_start_frames =
        node.get_parameter("cold_start_frames").as_int();
    config.optimizer.verbose = node.get_parameter("verbose").as_bool();
    config.optimizer.smoother_lag =
        node.get_parameter("smoother_lag").as_double();
    const std::string smoother_type =
        node.get_parameter("smoother_type").as_string();
    config.optimizer.smoother_type = smoother_type == "batch"
        ? auto_graph::SmootherType::Batch
        : auto_graph::SmootherType::Incremental;
    config.optimizer.relinearize_threshold =
        node.get_parameter("relinearize_threshold").as_double();
    config.optimizer.update_iterations =
        node.get_parameter("extra_iterations").as_int();
    return config;
}

}  // namespace filter_test

#pragma once
#include <Eigen/Dense>

namespace jlu {

struct ArmorObservationNoiseConfig {
  Eigen::Vector2d pixel_error{1.0, 1.0};
  double tangential_error_m = 0.01;
  double radial_error_m = 0.03;
  double height_error_m = 0.01;
  double yaw_error_rad = 0.005;
};

struct YawPitchDistanceConfig {
  double yaw_pitch_noise = 1.0;
  double basic_distance_noise = 0.001;
  double distance_noise_log_scale = 0.1;
  double basic_armor_yaw_noise = 0.01;
  double armor_yaw_log_divisor = 20.0;
};

struct ArmorMatchConfig {
  double max_match_mahalanobis_distance = 20.0; // 放宽匹配阈值 (仿真PnP噪声比真检测大)
  YawPitchDistanceConfig ypd_conf;
};

struct NisCheckConfig {
  int nis_failure_window_size = 10;
  double nis_failure_thres = 5.0;
  double reset_failure_percentage_thres = 60.0;
};

struct RobotConfig {
  ArmorMatchConfig armor_match_conf;
  NisCheckConfig nis_conf;
  int first_update_batch_size = 5;  // 增加冷启动帧数
  double lost_threshold_sec = 2.0;            // 放宽超时阈值
  double huge_vyaw_reset_thres = 20.0;

  double yaw_prior_noise = 9.0;
  double vyaw_prior_noise = 9.0;
  Eigen::Vector3d translation_prior_noise{0.1, 0.1, 0.1};
  Eigen::Vector3d velocity_prior_noise{0.5, 0.5, 0.5};

  double yaw_factor_noise = 0.005;
  double vyaw_factor_noise = 0.05;
  Eigen::Vector3d translation_factor_noise{0.001, 0.001, 0.001};
  Eigen::Vector3d velocity_factor_noise{0.01, 0.01, 0.01};

  double radius_prior_noise = 0.5;     // 放宽半径先验 (仿真r=0.28/0.38 vs jlu默认0.26)
  double default_radius = 0.30;        // 折中初始半径
  double radius_min = 0.15;
  double radius_max = 0.50;
  double dz_prior_noise = 0.5;         // 放宽dz先验
  double default_dz = 0.10;

  ArmorObservationNoiseConfig armor_observation_noise;
};

}  // namespace jlu

#pragma once

#include "filter_test/graph_optimizer/tracker_math.hpp"

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace filter_test::graph_optimizer {

struct ArmorReprojFactor : gtsam::NoiseModelFactorN<gtsam::Pose3> {
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;
    using Point2 = gtsam::Point2;
    using Point3 = gtsam::Point3;
    using Pose3 = gtsam::Pose3;

    ArmorReprojFactor(gtsam::Key key, const gtsam::SharedNoiseModel& noise,
                      const Eigen::Vector3d& corner_local,
                      const Eigen::Matrix3d& K,
                      const std::array<double, 5>& dist,
                      const Eigen::Vector2d& observed_pixel)
        : Base(noise, key),
          armor_point_(corner_local.x(), corner_local.y(), corner_local.z()),
          calib_(K(0, 0), K(1, 1), K(0, 1), K(0, 2), K(1, 2),
                 dist[0], dist[1], dist[2], dist[3]),
          px_obs_(observed_pixel.x(), observed_pixel.y()) {}

    gtsam::Vector evaluateError(const Pose3& armor_pose_camera,
                                gtsam::Matrix* H) const override {
        gtsam::Matrix36 H_transform;
        Point3 p_cam = armor_pose_camera.transformFrom(
            armor_point_, H ? &H_transform : nullptr, nullptr);
        gtsam::Matrix23 H_norm;
        Point2 pn = gtsam::PinholeCamera<gtsam::Cal3DS2>::Project(
            p_cam, H ? &H_norm : nullptr);
        gtsam::Matrix22 H_calib;
        Point2 px = calib_.uncalibrate(pn, {}, H ? &H_calib : nullptr);

        if (H) *H = H_calib * H_norm * H_transform;
        return px - px_obs_;
    }

    Point3 armor_point_;
    gtsam::Cal3DS2 calib_;
    Point2 px_obs_;
};

struct ArmorCenterFactor : gtsam::NoiseModelFactor5<
    gtsam::Pose3, gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector> {

    using Base = gtsam::NoiseModelFactor5<
        gtsam::Pose3, gtsam::Vector, gtsam::Vector, gtsam::Vector, gtsam::Vector>;

    ArmorCenterFactor(gtsam::Key k_armor, gtsam::Key k_pos, gtsam::Key k_yaw_vyaw,
                      gtsam::Key k_radius, gtsam::Key k_dz,
                      const gtsam::SharedNoiseModel& noise, int idx,
                      const Eigen::Isometry3d& T_c2o,
                      double r_min = kRadiusMin, double r_max = kRadiusMax)
        : Base(noise, k_armor, k_pos, k_yaw_vyaw, k_radius, k_dz),
          armor_index_(idx),
          radius_min_(r_min),
          radius_max_(r_max),
          T_camera_to_odom_(T_c2o) {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3& armor_pose_camera,
        const gtsam::Vector& pos_vel,
        const gtsam::Vector& yaw_vyaw,
        const gtsam::Vector& radius_vec,
        const gtsam::Vector& dz_vec,
        gtsam::Matrix* H1, gtsam::Matrix* H2, gtsam::Matrix* H3,
        gtsam::Matrix* H4, gtsam::Matrix* H5) const override {

        Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
        pose.pretranslate(armor_pose_camera.translation());
        pose.rotate(armor_pose_camera.rotation().matrix());
        Eigen::Isometry3d armor_pose_odom = T_camera_to_odom_ * pose;

        Eigen::Vector3d armor_position = armor_pose_odom.translation();
        Eigen::Vector3d rpy = rotationMatrixToRPY(armor_pose_odom.rotation().matrix());
        double armor_yaw = rpy.z();

        double r_u = radius_vec[armor_index_ % 2];
        double radius = auto_graph::logisticFunction(r_u, radius_min_, radius_max_);
        double dlog = auto_graph::logisticDerivative(radius, radius_min_, radius_max_);

        double nx = std::cos(armor_yaw);
        double ny = std::sin(armor_yaw);
        double tx = -ny;
        double ty = nx;

        double center_x = pos_vel[0];
        double center_y = pos_vel[2];
        double center_z = pos_vel[4];
        double center_yaw = yaw_vyaw[0];
        double dz = (armor_index_ % 2 == 0) ? 0.0 : dz_vec[0];

        double dx = center_x - armor_position.x();
        double dy = center_y - armor_position.y();

        double tangential_err = tx * dx + ty * dy;
        double radial_err = nx * dx + ny * dy - radius;
        double z_err = center_z - armor_position.z() + dz;
        double pred_armor_yaw = center_yaw + armor_index_ * M_PI_2;
        double yaw_err = std::remainder(armor_yaw - pred_armor_yaw, 2.0 * M_PI);

        gtsam::Vector4 err(tangential_err, radial_err, z_err, yaw_err);

        if (H1) {
            H1->setZero(4, 6);
            Eigen::Matrix<double, 4, 3> J_p;
            J_p << -tx, -ty, 0.0,
                   -nx, -ny, 0.0,
                    0.0, 0.0, -1.0,
                    0.0, 0.0, 0.0;
            double radial_proj = nx * dx + ny * dy;
            Eigen::Matrix<double, 4, 1> d_e_d_psi;
            d_e_d_psi << -radial_proj, tangential_err, 0.0, 1.0;
            double roll = rpy.x();
            double pitch = rpy.y();
            double cos_pitch = std::cos(pitch);
            Eigen::Matrix<double, 1, 3> d_psi_d_omega;
            d_psi_d_omega << 0.0, std::sin(roll) / cos_pitch,
                std::cos(roll) / cos_pitch;
            Eigen::Matrix<double, 4, 3> J_rot = d_e_d_psi * d_psi_d_omega;
            Eigen::Matrix<double, 4, 3> J_trans =
                J_p * armor_pose_odom.rotation().matrix();
            H1->leftCols<3>() = J_rot;
            H1->rightCols<3>() = J_trans;
        }
        if (H2) {
            H2->setZero(4, 6);
            (*H2)(0, 0) = tx;
            (*H2)(0, 2) = ty;
            (*H2)(1, 0) = nx;
            (*H2)(1, 2) = ny;
            (*H2)(2, 4) = 1.0;
        }
        if (H3) {
            H3->setZero(4, 2);
            (*H3)(3, 0) = -1.0;
        }
        if (H4) {
            H4->setZero(4, 2);
            (*H4)(1, armor_index_ % 2) = -dlog;
        }
        if (H5) {
            H5->setZero(4, 1);
            if (armor_index_ % 2 == 1) (*H5)(2, 0) = 1.0;
        }
        return err;
    }

    int armor_index_;
    double radius_min_;
    double radius_max_;
    Eigen::Isometry3d T_camera_to_odom_;
};

struct VelSmoothFactor : gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {
    using Base = gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector>;

    VelSmoothFactor(gtsam::Key k_prev, gtsam::Key k_cur,
                    const gtsam::SharedNoiseModel& noise)
        : Base(noise, k_prev, k_cur) {}

    gtsam::Vector evaluateError(
        const gtsam::Vector& x1, const gtsam::Vector& x2,
        gtsam::Matrix* H1, gtsam::Matrix* H2) const override {
        gtsam::Vector3 err;
        err(0) = x2[1] - x1[1];
        err(1) = x2[3] - x1[3];
        err(2) = x2[5] - x1[5];

        if (H1) {
            H1->setZero(3, 6);
            (*H1)(0, 1) = -1.0;
            (*H1)(1, 3) = -1.0;
            (*H1)(2, 5) = -1.0;
        }
        if (H2) {
            H2->setZero(3, 6);
            (*H2)(0, 1) = 1.0;
            (*H2)(1, 3) = 1.0;
            (*H2)(2, 5) = 1.0;
        }
        return err;
    }
};

struct VyawSmoothFactor : gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {
    using Base = gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector>;

    VyawSmoothFactor(gtsam::Key k_prev, gtsam::Key k_cur,
                     const gtsam::SharedNoiseModel& noise)
        : Base(noise, k_prev, k_cur) {}

    gtsam::Vector evaluateError(
        const gtsam::Vector& x1, const gtsam::Vector& x2,
        gtsam::Matrix* H1, gtsam::Matrix* H2) const override {
        gtsam::Vector1 err;
        err(0) = x2[1] - x1[1];

        if (H1) {
            H1->setZero(1, 2);
            (*H1)(0, 1) = -1.0;
        }
        if (H2) {
            H2->setZero(1, 2);
            (*H2)(0, 1) = 1.0;
        }
        return err;
    }
};

}  // namespace filter_test::graph_optimizer

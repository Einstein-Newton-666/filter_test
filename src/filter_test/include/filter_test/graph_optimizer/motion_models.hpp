#pragma once

#include "filter_test/auto_graph_optimizer/models/motion_model.hpp"
#include "filter_test/auto_graph_optimizer/models/measure_model.hpp"
#include "filter_test/auto_graph_optimizer/utils/helpers.hpp"
#include "filter_test/graph_optimizer/tracker_math.hpp"

#include <ceres/jet.h>
#include <cmath>
#include <vector>

namespace filter_test::graph_optimizer {

struct TranslationModel : auto_graph::MotionModel<TranslationModel, 6> {
    explicit TranslationModel(double dt_in) : dt(dt_in) {}
    double dt;

    template<typename T>
    void operator()(const T& x, T& x_next) const {
        x_next[0] = x[0] + x[1] * dt;
        x_next[1] = x[1];
        x_next[2] = x[2] + x[3] * dt;
        x_next[3] = x[3];
        x_next[4] = x[4] + x[5] * dt;
        x_next[5] = x[5];
    }
};

struct YawModel : auto_graph::MotionModel<YawModel, 2> {
    explicit YawModel(double dt_in) : dt(dt_in) {}
    double dt;

    template<typename T>
    void operator()(const T& x, T& x_next) const {
        x_next[0] = x[0] + x[1] * dt;
        x_next[1] = x[1];
    }
};

struct ArmorCVMotionModel : auto_graph::MotionModel<ArmorCVMotionModel, 11> {
    explicit ArmorCVMotionModel(double dt_in) : dt(dt_in) {}
    double dt;

    template<typename T>
    void operator()(const T& x, T& x_next) const {
        x_next[0] = x[0] + x[1] * dt;
        x_next[1] = x[1];
        x_next[2] = x[2] + x[3] * dt;
        x_next[3] = x[3];
        x_next[4] = x[4] + x[5] * dt;
        x_next[5] = x[5];
        x_next[6] = x[6] + x[7] * dt;
        x_next[7] = x[7];
        x_next[8] = x[8];
        x_next[9] = x[9];
        x_next[10] = x[10];
    }
};

struct ArmorCVMeasureYPD : auto_graph::MeasureModel<ArmorCVMeasureYPD, 11, 4> {
    explicit ArmorCVMeasureYPD(int i) : I(i) {}
    int I;

    template<typename T>
    void operator()(const std::vector<T>& x, std::vector<T>& z) const {
        T zero = x[6] * 0.0;
        T angle = x[6] + (zero + M_PI_2 * I);
        T cos_angle = ceres::cos(angle);
        T sin_angle = ceres::sin(angle);
        T r = auto_graph::logistic(x[8 + I % 2], kRadiusMin, kRadiusMax);
        T armor_x = x[0] - cos_angle * r;
        T armor_y = x[2] - sin_angle * r;
        T armor_z = x[4] + (I % 2 == 0 ? zero : x[10]);

        z[0] = ceres::atan2(armor_y, armor_x);
        z[1] = ceres::atan2(
            armor_z, ceres::sqrt(armor_x * armor_x + armor_y * armor_y));
        z[2] = ceres::sqrt(
            armor_x * armor_x + armor_y * armor_y + armor_z * armor_z);
        z[3] = x[6] + (zero + M_PI_2 * I);
    }
};

struct ArmorCVMeasureYPDDouble : auto_graph::MeasureModel<ArmorCVMeasureYPDDouble, 11, 8> {
    ArmorCVMeasureYPDDouble(int i, int j) : I(i), J(j) {}
    int I;
    int J;

    template<typename T>
    void operator()(const std::vector<T>& x, std::vector<T>& z) const {
        int idx = 0;
        for (int armor_index : {I, J}) {
            T zero = x[6] * 0.0;
            T angle = x[6] + (zero + M_PI_2 * armor_index);
            T cos_angle = ceres::cos(angle);
            T sin_angle = ceres::sin(angle);
            T r = auto_graph::logistic(
                x[8 + armor_index % 2], kRadiusMin, kRadiusMax);
            T armor_x = x[0] - cos_angle * r;
            T armor_y = x[2] - sin_angle * r;
            T armor_z = x[4] + (armor_index % 2 == 0 ? zero : x[10]);

            z[idx + 0] = ceres::atan2(armor_y, armor_x);
            z[idx + 1] = ceres::atan2(
                armor_z, ceres::sqrt(armor_x * armor_x + armor_y * armor_y));
            z[idx + 2] = ceres::sqrt(
                armor_x * armor_x + armor_y * armor_y + armor_z * armor_z);
            z[idx + 3] = x[6] + (zero + M_PI_2 * armor_index);
            idx += 4;
        }
    }
};

}  // namespace filter_test::graph_optimizer

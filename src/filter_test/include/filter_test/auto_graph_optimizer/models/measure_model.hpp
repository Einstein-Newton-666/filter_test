#pragma once

#include "../utils/types.hpp"
#include <Eigen/Dense>
#include <ceres/jet.h>
#include <vector>
#include <utility>

namespace auto_graph {

/**
 * 观测模型基类（CRTP模式）
 *
 * 用户继承这个基类来定义观测模型
 *
 * 使用示例：
 *   struct MeasureSingle : MeasureModel<MeasureSingle, 11, 4> {
 *       int I;
 *       MeasureSingle(int i) : I(i) {}
 *
 *       template<typename T>
 *       void operator()(const T& x, T& z) const {
 *           z[0] = ceres::atan2(x[2], x[0]);
 *           // ...
 *       }
 *   };
 */
template<typename Derived, int StateDim, int MeasureDim>
struct MeasureModel {
    static constexpr int STATE_DIM = StateDim;
    static constexpr int MEASURE_DIM = MeasureDim;

    /**
     * 自动线性化（通过Ceres Jet自动微分）
     *
     * @param x 当前状态
     * @return {预测观测, 雅可比矩阵H}
     */
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> linearize(const Eigen::VectorXd& x) const {
        const auto& self = static_cast<const Derived&>(*this);
        int n = STATE_DIM;
        int m = MEASURE_DIM;

        // 使用Ceres Jet进行自动微分
        std::vector<ceres::Jet<double, Eigen::Dynamic>> x_jet(n);
        for (int i = 0; i < n; ++i) {
            x_jet[i].a = x[i];
            x_jet[i].v.resize(n);
            x_jet[i].v.setZero();
            x_jet[i].v[i] = 1.0;  // 对自己的偏导数为1
        }

        std::vector<ceres::Jet<double, Eigen::Dynamic>> z_jet(m);
        for (int i = 0; i < m; ++i) {
            z_jet[i].v.resize(n);  // 导数向量维度 = 状态维度
            z_jet[i].v.setZero();
        }

        // 调用用户定义的operator()
        self(x_jet, z_jet);

        // 提取结果
        Eigen::VectorXd z(m);
        Eigen::MatrixXd H(m, n);
        for (int i = 0; i < m; ++i) {
            z[i] = z_jet[i].a;
            H.block(i, 0, 1, n) = z_jet[i].v.transpose();
        }

        return {z, H};
    }

    /**
     * 获取状态维度
     */
    static constexpr int stateDim() {
        return STATE_DIM;
    }

    /**
     * 获取观测维度
     */
    static constexpr int measureDim() {
        return MEASURE_DIM;
    }
};

} // namespace auto_graph

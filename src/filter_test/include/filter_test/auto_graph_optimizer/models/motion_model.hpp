#pragma once

#include "../utils/types.hpp"
#include <Eigen/Dense>
#include <ceres/jet.h>
#include <vector>
#include <utility>

namespace auto_graph {

/**
 * 运动模型基类（CRTP模式）
 *
 * 用户继承这个基类来定义运动学模型
 *
 * 使用示例：
 *   struct CVModel : MotionModel<CVModel, 11> {
 *       double dt;
 *       CVModel(double dt) : dt(dt) {}
 *
 *       template<typename T>
 *       void operator()(const T& x, T& x_next) const {
 *           x_next[0] = x[0] + dt * x[1];
 *           // ...
 *       }
 *   };
 */
template<typename Derived, int StateDim>
struct MotionModel {
    static constexpr int STATE_DIM = StateDim;

    /**
     * 自动线性化（通过Ceres Jet自动微分）
     *
     * @param x 当前状态
     * @return {预测状态, 雅可比矩阵F}
     */
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> linearize(const Eigen::VectorXd& x) const {
        const auto& self = static_cast<const Derived&>(*this);
        int n = STATE_DIM;

        // 使用Ceres Jet进行自动微分
        std::vector<ceres::Jet<double, Eigen::Dynamic>> x_jet(n);
        for (int i = 0; i < n; ++i) {
            x_jet[i].a = x[i];
            x_jet[i].v.resize(n);
            x_jet[i].v.setZero();
            x_jet[i].v[i] = 1.0;  // 对自己的偏导数为1
        }

        std::vector<ceres::Jet<double, Eigen::Dynamic>> x_next_jet(n);
        for (int i = 0; i < n; ++i) {
            x_next_jet[i].v.resize(n);
            x_next_jet[i].v.setZero();
        }

        // 调用用户定义的operator()
        self(x_jet, x_next_jet);

        // 提取结果
        Eigen::VectorXd x_next(n);
        Eigen::MatrixXd F(n, n);
        for (int i = 0; i < n; ++i) {
            x_next[i] = x_next_jet[i].a;
            F.block(i, 0, 1, n) = x_next_jet[i].v.transpose();
        }

        return {x_next, F};
    }

    /**
     * 获取状态维度
     */
    static constexpr int stateDim() {
        return STATE_DIM;
    }
};

} // namespace auto_graph

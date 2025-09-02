#pragma once

#include <Eigen/Dense>
#include <vector>
#include "common.hpp"

// 无迹卡尔曼滤波器（Unscented Kalman Filter, UKF）实现
class UnscentedKalmanFilter
{
public:
    UnscentedKalmanFilter(): x_e(Eigen::VectorXd::Zero(0)), P_mat(Eigen::MatrixXd::Identity(0,0) * INF) {}
    explicit UnscentedKalmanFilter(const Eigen::VectorXd& x): x_e(x) {}

    // 初始化状态和协方差
    void init(const Eigen::VectorXd& x0) {
        this->x_e = x0;
        this->P_mat = Eigen::MatrixXd::Identity(x0.size(), x0.size());
    }

    void setState(const Eigen::VectorXd& x0) { this->x_e = x0; }
    Eigen::VectorXd getState() const { return this->x_e; }

    struct UKFParams {
        double alpha = 1e-3; // 控制sigma点分布的参数
        double beta = 2.0;   // 先验分布的参数（高斯时取2）
        double kappa = 0.0;  // 次要缩放参数
    };

    // 只预测不更新协方差
    template<class PredictFunc>
    Eigen::VectorXd predict(PredictFunc&& f, const UKFParams& params = UKFParams()) {
        int n = f.size;
        this->x_e.resize(n);
        this->P_mat.conservativeResize(n, n);
        double lambda = params.alpha * params.alpha * (n + params.kappa) - n;
        double n_lambda = n + lambda;
        if (n_lambda <= 0) throw std::runtime_error("UKF: n + lambda must be positive!");
        // 生成sigma点
        Eigen::MatrixXd sigma_points = computeSigmaPoints(x_e, P_mat, lambda, n);
        // sigma点传播
        Eigen::MatrixXd propagated_sigma = propagateSigmaPoints(sigma_points, f);
        // 权重计算
        std::vector<double> wm(2 * n + 1, 0.5 / n_lambda);
        std::vector<double> wc(2 * n + 1, 0.5 / n_lambda);
        wm[0] = lambda / n_lambda;
        wc[0] = lambda / n_lambda + (1 - params.alpha * params.alpha + params.beta);
        // 先验状态均值
        Eigen::VectorXd x_pri = Eigen::VectorXd::Zero(n);
        for (int i = 0; i < 2 * n + 1; ++i) {
            x_pri += wm[i] * propagated_sigma.col(i);
        }

        //ekf不同的是这里先验协方差可以和先验估计分开计算，不需要返回结构体

        // 直接返回先验估计
        return x_pri;
    }

    // 预测并更新协方差
    template<class PredictFunc>
    void predict_forward(PredictFunc&& f, const Eigen::MatrixXd& Q, const UKFParams& params = UKFParams()) {
        int n = f.size;
        this->x_e.resize(n);
        this->P_mat.conservativeResize(n, n);
        double lambda = params.alpha * params.alpha * (n + params.kappa) - n;
        double n_lambda = n + lambda;
        if (n_lambda <= 0) throw std::runtime_error("UKF: n + lambda must be positive!");
        // 生成sigma点
        Eigen::MatrixXd sigma_points = computeSigmaPoints(x_e, P_mat, lambda, n);
        // sigma点传播
        Eigen::MatrixXd propagated_sigma = propagateSigmaPoints(sigma_points, f);
        // 权重计算
        std::vector<double> wm(2 * n + 1, 0.5 / n_lambda);
        std::vector<double> wc(2 * n + 1, 0.5 / n_lambda);
        wm[0] = lambda / n_lambda;
        wc[0] = lambda / n_lambda + (1 - params.alpha * params.alpha + params.beta);
        // 先验状态均值
        Eigen::VectorXd x_pri = Eigen::VectorXd::Zero(n);
        for (int i = 0; i < 2 * n + 1; ++i) {
            x_pri += wm[i] * propagated_sigma.col(i);
        }
        // 先验协方差
        Eigen::MatrixXd P_pri = Q;
        for (int i = 0; i < 2 * n + 1; ++i) {
            Eigen::VectorXd diff = propagated_sigma.col(i) - x_pri;
            P_pri += wc[i] * diff * diff.transpose();
        }
        this->x_e = x_pri;
        this->P_mat = P_pri;
    }

 
    template<class MeasureFunc>
    Eigen::VectorXd measure(MeasureFunc&& h, const UKFParams& params = UKFParams()) {
        int n = x_e.size();
        int m = h.output_size;
        double lambda = params.alpha * params.alpha * (n + params.kappa) - n;
        double n_lambda = n + lambda;
        if (n_lambda <= 0) throw std::runtime_error("UKF: n + lambda must be positive!");
        // 生成sigma点
        Eigen::MatrixXd sigma_points = computeSigmaPoints(x_e, P_mat, lambda, n);
        // sigma点传播
        Eigen::MatrixXd z_sigma = propagateSigmaPoints(sigma_points, h);
        // 权重计算
        std::vector<double> wm(2 * n + 1, 0.5 / n_lambda);
        std::vector<double> wc(2 * n + 1, 0.5 / n_lambda);
        wm[0] = lambda / n_lambda;
        wc[0] = lambda / n_lambda + (1 - params.alpha * params.alpha + params.beta);
        // 先验观测均值
        Eigen::VectorXd z_pri = Eigen::VectorXd::Zero(m);
        for (int i = 0; i < 2 * n + 1; ++i) {
            z_pri += wm[i] * z_sigma.col(i);
        }
        // 直接返回先验观测
        return z_pri;
    }


    template<class MeasureFunc>
    void update_forward(MeasureFunc&& h, const Eigen::VectorXd& z, const Eigen::MatrixXd& R, const UKFParams& params = UKFParams()) {
        int n = h.input_size; // 观测方程输入维度
        int m = h.output_size; // 观测方程输出维度
        if (x_e.size() != n) x_e.resize(n);
        if (P_mat.rows() != n || P_mat.cols() != n) P_mat.conservativeResize(n, n);
        double lambda = params.alpha * params.alpha * (n + params.kappa) - n;
        double n_lambda = n + lambda;
        if (n_lambda <= 0) throw std::runtime_error("UKF: n + lambda must be positive!");
        // 生成sigma点
        Eigen::MatrixXd sigma_points = computeSigmaPoints(x_e, P_mat, lambda, n);
        // sigma点传播
        Eigen::MatrixXd z_sigma = propagateSigmaPoints(sigma_points, h);
        // 权重计算
        std::vector<double> wm(2 * n + 1, 0.5 / n_lambda);
        std::vector<double> wc(2 * n + 1, 0.5 / n_lambda);
        wm[0] = lambda / n_lambda;
        wc[0] = lambda / n_lambda + (1 - params.alpha * params.alpha + params.beta);
        // 先验观测均值
        Eigen::VectorXd z_pri = Eigen::VectorXd::Zero(m);
        for (int i = 0; i < 2 * n + 1; ++i) {
            z_pri += wm[i] * z_sigma.col(i);
        }
        // 观测协方差和交叉协方差
        Eigen::MatrixXd S = R;
        Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n, m);
        for (int i = 0; i < 2 * n + 1; ++i) {
            Eigen::VectorXd dz = z_sigma.col(i) - z_pri;
            Eigen::VectorXd dx = sigma_points.col(i) - x_e;
            S += wc[i] * dz * dz.transpose();
            Tc += wc[i] * dx * dz.transpose();
        }
        // 卡尔曼增益
        Eigen::MatrixXd K = Tc * S.inverse();
        // 状态和协方差更新
        x_e = x_e + K * (z - z_pri);
        P_mat = P_mat - K * S * K.transpose();
    }


    template<class MeasureFunc, class PredictFunc>
    void update(
        MeasureFunc&& measure_func,
        PredictFunc&& predict_func,
        const Eigen::VectorXd& z,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const UKFParams& params = UKFParams()
    ) {
        this->predict_forward(predict_func, Q, params);
        this->update_forward(measure_func, z, R, params);
    }

private:
    Eigen::VectorXd x_e; // 估计状态变量
    Eigen::MatrixXd P_mat; // 估计状态协方差矩阵
    static constexpr double INF = 1e9;

    // 生成sigma点
    Eigen::MatrixXd computeSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, double lambda, int n) {
        Eigen::MatrixXd sigma_points(n, 2 * n + 1);
        Eigen::MatrixXd A = P.llt().matrixL(); // Cholesky分解
        sigma_points.col(0) = x;
        double scale = sqrt(n + lambda);
        for (int i = 0; i < n; ++i) {
            sigma_points.col(i + 1) = x + scale * A.col(i);
            sigma_points.col(i + 1 + n) = x - scale * A.col(i);
        }
        return sigma_points;
    }

    // sigma点传播，传入函数参数为(const T& x_pre, T& x_cur)
    template<class SigmaFunc>
    Eigen::MatrixXd propagateSigmaPoints(const Eigen::MatrixXd& sigma_points, SigmaFunc&& f) {
        int n = sigma_points.rows();
        int num_sigma = sigma_points.cols();
        Eigen::MatrixXd propagated_sigma(n, num_sigma);
        for (int i = 0; i < num_sigma; ++i) {
            std::vector<double> x_pre(n);
            std::vector<double> x_cur(n);
            for (int j = 0; j < n; ++j) x_pre[j] = sigma_points(j, i);
            f(x_pre, x_cur); // 用户自定义函数对vector操作
            for (int j = 0; j < n; ++j) propagated_sigma(j, i) = x_cur[j];
        }
        return propagated_sigma;
    }
};
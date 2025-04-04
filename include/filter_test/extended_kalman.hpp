#pragma once

#include <ceres/jet.h>

#include <Eigen/Dense>

// template<int N_X, int N_Y>
class ExtendedKalman {
    //  private:
    // using This = ExtendedKalman<N_X, N_Y>;

public:
    // using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    // using MatrixYX = Eigen::Matrix<double, N_Y, N_X>;
    // using MatrixXY = Eigen::Matrix<double, N_X, N_Y>;
    // using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
    // using MatrixX1 = Eigen::Matrix<double, N_X, 1>;
    // using MatrixY1 = Eigen::Matrix<double, N_Y, 1>;
    // Eigen::MatrixXd;
    // Eigen::VectorXd;

private:
    Eigen::VectorXd x_e; // 估计状态变量
    Eigen::MatrixXd p_mat; // 状态协方差

    static constexpr double INF = 1e9;

public:
    ExtendedKalman(): x_e { Eigen::VectorXd::Zero(0) }, p_mat { Eigen::MatrixXd::Identity(0,0) * INF } {}
    explicit ExtendedKalman(const Eigen::MatrixXd& x): x_e { x } {}

    // 初始化且方差为单位矩阵
    void init_x(const Eigen::VectorXd& x0) {
        this->x_e = x0;
        this->p_mat = Eigen::MatrixXd::Identity(x0.size(), x0.size());
    }

    // 修正用
    Eigen::MatrixXd get_x() const {
        return this->x_e;
    }

    void set_x(const Eigen::VectorXd& x) {
        this->x_e = x;
    }

    struct PredictResult {
        Eigen::VectorXd x_p;
        Eigen::MatrixXd f_mat;
        // Eigen::VectorXd x_p = Eigen::VectorXd::Zero();
        // Eigen::MatrixXd f_mat = Eigen::MatrixXd::Zero();
    };

    //只预测不更新协方差
    template<class PredictFunc>
    PredictResult predict(PredictFunc&& predict_func) const {
        this->x_e.resize(predict_func.size); 
        ceres::Jet<double, predict_func.size> x_e_jet[predict_func.size];
        for (int i = 0; i < predict_func.size; ++i) {
            x_e_jet[i].a = this->x_e[i];
            x_e_jet[i].v[i] = 1.;
            // a 对自己的偏导数为 1.
        }
        ceres::Jet<double, predict_func.size> x_p_jet[predict_func.size];
        predict_func(x_e_jet, x_p_jet);
        Eigen::VectorXd x_p = Eigen::VectorXd::Zero(predict_func.size);
        for (int i = 0; i < predict_func.size; ++i) {
            x_p[i] = x_p_jet[i].a;
        }
        Eigen::MatrixXd f_mat = Eigen::MatrixXd::Zero(predict_func.size, predict_func.size);
        for (int i = 0; i < predict_func.size; ++i) {
            f_mat.block(i, 0, 1, predict_func.size) = x_p_jet[i].v.transpose();
        }
        return PredictResult { x_p, f_mat };
    }

    //预测并更新协方差
    template<class PredictFunc>
    void predict_forward(PredictFunc&& predict_func, const Eigen::MatrixXd& q_mat) {
        PredictResult pre_res = this->predict(predict_func);
        this->x_e = pre_res.x_p;
        this->p_mat.resize(predict_func.size, predict_func.size);
        this->p_mat = pre_res.f_mat * this->p_mat * pre_res.f_mat.transpose() + q_mat;
    }

    struct MeasureResult {
        Eigen::VectorXd y_e;
        Eigen::MatrixXd h_mat;
        // Eigen::VectorXd y_e = Eigen::VectorXd::Zero();
        // Eigen::MatrixXd h_mat = Eigen::MatrixXd::Zero();
    };

    template<class MeasureFunc>
    MeasureResult measure(MeasureFunc&& measure_func) {
        ceres::Jet<double, measure_func.input_size> x_e_jet[measure_func.input_size];
        for (int i = 0; i < measure_func.input_size; ++i) {
            x_e_jet[i].a = this->x_e[i];
            x_e_jet[i].v[i] = 1;
        }
        ceres::Jet<double, measure_func.input_size> y_e_jet[measure_func.output_size];
        measure_func(x_e_jet, y_e_jet); // 转化成 Y 类型后的预测值，期间自动求导
        Eigen::VectorXd y_e = Eigen::VectorXd::Zero(measure_func.output_size);
        for (int i = 0; i < measure_func.output_size; ++i) {
            y_e[i] = y_e_jet[i].a;
        }
        Eigen::MatrixXd h_mat = Eigen::MatrixXd::Zero(measure_func.output_size, measure_func.input_size);
        for (int i = 0; i < measure_func.output_size; ++i) {
            h_mat.block(i, 0, 1, measure_func.input_size) = y_e_jet[i].v.transpose();
        }
        return MeasureResult { y_e, h_mat };
    }

    template<class MeasureFunc>
    void update_forward(MeasureFunc&& measure_func, const Eigen::VectorXd& y_mat, const Eigen::MatrixXd& r_mat) {
        MeasureResult mea_res = this->measure(measure_func);
        // K 中包含 Y 到 X 的一阶转移矩阵
        this->x_e.resize(measure_func.inputsize);
        this->p_mat.resize(measure_func.inputsize, measure_func.inputsize);
        Eigen::MatrixXd k_mat = this->p_mat * mea_res.h_mat.transpose()
            * (mea_res.h_mat * this->p_mat * mea_res.h_mat.transpose() + r_mat).inverse();
        this->x_e = this->x_e + k_mat * (y_mat - mea_res.y_e);
        this->p_mat = (Eigen::MatrixXd::Identity(measure_func.inputsize, measure_func.inputsize) 
            - k_mat * mea_res.h_mat) * this->p_mat;
    }

    template<class MeasureFunc, class PredictFunc>
    void update(
        MeasureFunc&& measure_func,
        PredictFunc&& predict_func,
        const Eigen::VectorXd& y_mat,
        const Eigen::MatrixXd& q_mat,
        const Eigen::MatrixXd& r_mat
    ) {
        this->predict_forward(predict_func, q_mat);
        this->update_forward(measure_func, y_mat, r_mat);
    }
};


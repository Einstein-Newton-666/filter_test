#pragma once

#include <Eigen/Dense>
#include <ceres/jet.h>

#include "common.hpp"

// 将弧度约束在[-pi, pi]范围内
#ifndef _std_radian
#define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * M_PI)) * (2 * M_PI))
#endif

//修改自交哥的滤波器，主要改动为将滤波器的大小修改为动态大小，但速度可能会下降QWQ
class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter(): x_e { Eigen::VectorXd::Zero(0) }, P_mat { Eigen::MatrixXd::Identity(0,0) * INF } {}
    explicit ExtendedKalmanFilter(const Eigen::MatrixXd& x): x_e { x } {}

    void init(const Eigen::VectorXd& x0) {
        this->x_e = x0;
        this->P_mat = Eigen::MatrixXd::Identity(x0.size(), x0.size());
    }// 初始化且方差为单位矩阵

    void setState(const Eigen::VectorXd & x0) {this->x_e = x0;}
    Eigen::MatrixXd getState() const {return this->x_e;}

    struct PredictResult {
        Eigen::VectorXd x_pri;
        Eigen::MatrixXd F;
    };

    // 只预测不更新协方差
    template<class PredictFunc>
    PredictResult predict(PredictFunc&& f){
        this->x_e.resize(f.size);
        std::vector<ceres::Jet<double,Eigen::Dynamic>> x_e_jet(f.size);
        for(int i = 0; i < f.size; ++i){
            x_e_jet[i].a = this->x_e[i];
            x_e_jet[i].v.resize(f.size);
            x_e_jet[i].v.setZero();
            x_e_jet[i].v[i] = 1.; // a 对自己的偏导数为1.
        }
        std::vector<ceres::Jet<double,Eigen::Dynamic>> x_p_jet(f.size);
        for(int i = 0; i < f.size; ++i){
            x_p_jet[i].v.resize(f.size);
            x_p_jet[i].v.setZero();
        }
        f(x_e_jet, x_p_jet);
        Eigen::VectorXd x_pri = Eigen::VectorXd::Zero(f.size);
        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(f.size, f.size);
        for(int i = 0; i < f.size; ++i){
            x_pri[i] = x_p_jet[i].a;
            F.block(i, 0, 1, f.size) = x_p_jet[i].v.transpose();
        }
        return PredictResult{x_pri, F};
    }
    
    // 预测并更新协方差
    template<class PredictFunc>
    void predict_forward(PredictFunc&& f, const Eigen::MatrixXd& Q){
        PredictResult predict_result = this->predict(f);
        this->x_e = predict_result.x_pri;
        this->P_mat.resize(f.size, f.size);
        this->P_mat = predict_result.F * this->P_mat * predict_result.F.transpose() + Q;
    }

    struct MeasureResult {
        Eigen::VectorXd z_pri;
        Eigen::MatrixXd H;
    };

    template<class MeasureFunc>
    MeasureResult measure(MeasureFunc&& h){
        std::vector<ceres::Jet<double,Eigen::Dynamic>> x_e_jet(h.input_size);
        for(int i = 0; i < h.input_size; i++){
            x_e_jet[i].a = this->x_e[i];
            x_e_jet[i].v.resize(h.input_size);
            x_e_jet[i].v.setZero();
            x_e_jet[i].v[i] = 1.;
        }
        std::vector<ceres::Jet<double,Eigen::Dynamic>> y_e_jet(h.output_size);
        for(int i = 0; i < h.output_size; i++){
            y_e_jet[i].v.resize(h.output_size);
            y_e_jet[i].v.setZero();
        }
        h(x_e_jet, y_e_jet);
        Eigen::VectorXd z_pri = Eigen::VectorXd::Zero(h.output_size);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(h.output_size, h.input_size);
        for(int i = 0; i < h.output_size; i++){
            z_pri[i] = y_e_jet[i].a;
            H.block(i,0,1,h.input_size) = y_e_jet[i].v.transpose();
        }
        return MeasureResult{z_pri, H};
    }

    template<class MeasureFunc>
    void update_forward(MeasureFunc&& h,const Eigen::VectorXd & z,const Eigen::MatrixXd& R){
        MeasureResult measure_result = this->measure(h);
        this->x_e.resize(h.input_size);
        this->P_mat.resize(h.input_size, h.input_size);
        Eigen::MatrixXd K = P_mat * measure_result.H.transpose() * (measure_result.H * P_mat * measure_result.H.transpose() + R).inverse();
        Eigen::VectorXd innovation = z - measure_result.z_pri;
        this->x_e = this->x_e + K * innovation;
        this->P_mat = (Eigen::MatrixXd::Identity(h.input_size, h.input_size) - K * measure_result.H) * this->P_mat;
    }

    template<class MeasureFunc, class PredictFunc>
    void update(
        MeasureFunc&& measure_func,
        PredictFunc&& predict_func,
        const Eigen::VectorXd& z, 
        const Eigen::MatrixXd& Q, 
        const Eigen::MatrixXd& R
    ) {
        this->predict_forward(predict_func, Q);
        MeasureResult measure_result = this->measure(measure_func);
        Eigen::VectorXd observation = z;
        for(int i = 0; i < z.size() / 4; i++){
            observation[i * 4] = get_closest_angle(observation[i * 4 + 0], measure_result.z_pri[i * 4 + 0]);
            observation[i * 4 + 3] = get_closest_angle(observation[i * 4 + 3], measure_result.z_pri[i * 4 + 3]);
        }

        this->update_forward(measure_func, observation, R);
    }


private:
    Eigen::VectorXd x_e; // 估计状态变量
    Eigen::MatrixXd P_mat; // 估计状态协方差矩阵

    static constexpr double INF = 1e9;

};

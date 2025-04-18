#ifndef ARMOR_PROCESSOR__ARMOR_MODEL_HPP_
#define ARMOR_PROCESSOR__ARMOR_MODEL_HPP_


#include <map>
#include <iostream>
#include "armor_tracker/common.hpp"
#include "armor_tracker/filter/kalman_filter.hpp"

namespace rm_auto_aim
{
// [ x, vx, y, vy, z, vz, yaw]  [ x, y, z, yaw]
// [ 0,  1, 2,  3, 4,  5,   6]  [ 0, 1, 2,   3]

constexpr int X_N = 7, Z_N = 4;

struct kfPredict{
    explicit kfPredict(double dt_)
    : dt(dt_){}

    template <typename T>
    void operator()(const T x_pre[X_N], T x_cur[X_N]){
        x_cur[0] = x_pre[0] + x_pre[1] * dt;
        x_cur[1] = x_pre[1];
        x_cur[2] = x_pre[2] + x_pre[3] * dt;
        x_cur[3] = x_pre[3];
        x_cur[4] = x_pre[4] + x_pre[5] * dt;
        x_cur[5] = x_pre[5];
        x_cur[6] = x_pre[6];
    }

    double dt;
};
// TODO：运用yaw,pitch,distance引入非线性，需要运动ekf模型线性化，可能会丢失精度，对比两个模型哪个效果好
struct kfMeasure{
    template <typename T>
    void operator()(const T x[X_N], T z[Z_N]){
        z[0] = x[0];
        z[1] = x[2];
        z[2] = x[4];
        z[3] = x[6];
    }
};

struct FuncA{
    explicit FuncA(const double& dt_)
    : dt(dt_){}
    template <typename T>
    void operator()(T& A){
        A = T::Identity();
        A(0, 1) = dt;
        A(2, 3) = dt;
        A(4, 5) = dt; 
        std::cout<<"A如下："<<std::endl;
        std::cout<<A<<std::endl;
    }

    double dt;
};

struct FuncH{
    template <typename T>
    void operator()(T& H){
        H = T::Zero();
        H(0, 0) = 1.; // 对应 x
        H(1, 2) = 1.; // 对应 y
        H(2, 4) = 1.; // 对应 z
        H(3, 6) = 1.; // 对应 yaw
        std::cout<<"H如下："<<std::endl;
        std::cout<<H<<std::endl;
    }
};

using KF = KalmanFilter<X_N, Z_N>;



struct ArmorTracker{
public:
    ArmorTracker() = default;
    explicit ArmorTracker(const Eigen::VectorXd& x0){
        this->kf.init(x0);
    }
    ~ArmorTracker() {};
    KF kf;

    TrackerState tracker_state;
    Eigen::VectorXd measurement; // for debug
    Eigen::VectorXd pri_estimation;
    Eigen::VectorXd post_estimation;

    int detect_count = 0;     
    int lost_count = 0;       
    int tracking_thres = 0;    
    int lost_thres = 0;     
    bool matched = false;     
};


class ArmorModel {
public:
    ArmorModel(){
        filters[1] = ArmorTracker();
        filters[2] = ArmorTracker();
    }

    void initFilter(const Eigen::VectorXd& x0,const int& index);

    Eigen::MatrixXd update_Q(const double& dt);
    Eigen::MatrixXd update_R(const Eigen::Matrix<double, Z_N, 1> &z);

    std::map<int, ArmorTracker> filters; //对于一辆车来说最多可以看到2个装甲板，会不会直接实例化两个跟踪器更好管理？

    double position_diff_thres;
    int tracking_thres;
    int lost_thres;
    double s2qx_, s2qy_, s2qz_, s2qyaw_; // 过程噪声方差
    double r_x, r_y, r_z, r_yaw; // 观测噪声方差
private:

};


}

#endif  // ARMOR_PROCESSOR__ARMOR_MODEL_HPP_
#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>

namespace rm_auto_aim
{
// KalmanFilter模板类
// N_X: 系统状态维度
// N_Z: 观测状态维度


template <int N_X, int N_Z>
class KalmanFilter
{
public:
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixZX = Eigen::Matrix<double, N_Z, N_X>;
    using MatrixXZ = Eigen::Matrix<double, N_X, N_Z>;
    using MatrixZZ = Eigen::Matrix<double, N_Z, N_Z>;
    using MatrixX1 = Eigen::Matrix<double, N_X, 1>;
    using MatrixZ1 = Eigen::Matrix<double, N_Z, 1>;
    KalmanFilter()// TODO:优化初始化
    {
        A = MatrixXX::Identity(); //TODO：A矩阵初始化全为0,还是对角线为1？
        P = MatrixXX::Identity() * INF;
        x_e = MatrixX1::Zero();
        H = MatrixZX::Zero();
        K = MatrixXZ::Zero();
    }

    void setState(const MatrixX1 &x0) noexcept { x_e = x0; }
    MatrixX1 getState(){ return x_e;}
    void init(const MatrixX1 &x0){
        x_e = x0;
        P = MatrixXX::Identity() * INF;
        // K = MatrixXZ::Zero();
    }

    template <class FuncA>
    MatrixX1 predict(FuncA&& funcA, const MatrixXX& Q){
        funcA(A);
        x_e = A * x_e;
        P = A * P * A.transpose() + Q;
        return x_e;
    }

    template <class FuncH>
    MatrixX1 update(FuncH&& funcH, const MatrixZ1 &z, const MatrixZZ &R)  {
        funcH(H);
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        x_e = x_e + K * (z - H * x_e);
        P = (MatrixXX::Identity() - K * H) * P;
        return x_e;
    }

private:
    MatrixXX A;
    MatrixZX H;
    MatrixXX P;
    MatrixXZ K;
    MatrixX1 x_e;
    static constexpr double INF = 998244353.;  // 交爷说效果好
};

}































#endif // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
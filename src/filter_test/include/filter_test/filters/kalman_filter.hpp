#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <memory>

// KalmanFilter模板类（静态维度KF实现）
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

    void init(const Eigen::VectorXd& x0) {
        if (x0.size() != N_X) {
            throw std::runtime_error("KalmanFilter::init: Input size mismatch");
        }
        x_e = x0;
        P = MatrixXX::Identity() * INF;
    }

    void setState(const Eigen::VectorXd& x0) {
        if (x0.size() != N_X) {
            throw std::runtime_error("KalmanFilter::setState: Input size mismatch");
        }
        x_e = x0;
    }

    Eigen::VectorXd getState() const {
        return x_e;
    }

    template<class PredictFunc>
    Eigen::VectorXd predict(PredictFunc&& funcA, const Eigen::MatrixXd& Q){
        funcA(A);
        x_e = A * x_e;
        P = A * P * A.transpose() + Q;
        return x_e;
    }

    template<class MeasureFunc>
    Eigen::VectorXd update(MeasureFunc&& funcH, const Eigen::VectorXd& z, const Eigen::MatrixXd& R) {
        if (z.size() != N_Z) {
            throw std::runtime_error("KalmanFilter::update: Measurement size mismatch");
        }
        funcH(H);
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        x_e = x_e + K * (z - H * x_e);
        P = (MatrixXX::Identity() - K * H) * P;
        return x_e;
    }

    // 原始接口保持兼容性
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

#endif // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
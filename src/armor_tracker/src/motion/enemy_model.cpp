#include "armor_tracker/motion/enemy_model.hpp"

namespace rm_auto_aim{

Eigen::MatrixXd EnemyModel::update_Q(const double& dt){
    Eigen::Matrix<double,10,10> q;
    // double vx = pri_estimation[1], vy = pri_estimation[3], v_yaw = pri_estimation[7];
    // double dx = pow(pow(vx, 2) + pow(vy, 2), 0.5), dy = abs(v_yaw);
    double x,y;
    double t = dt,r = s2qr_;
    // x = exp(-dy) * (s2qxy_max_ - s2qxy_min_) + s2qxy_min_;// TODO：对比验证固定噪声参数和该可变噪声参数哪个效果好
    // y = exp(-dx) * (s2qyaw_max_ - s2qyaw_min_) + s2qyaw_min_;
    x = s2qxy_max_;
    y = s2qyaw_max_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    double q_r = pow(t, 4)/ 4 * r;
    // clang-format off
    //      xc      v_xc    yc      v_yc    za1     za2     yaw     v_yaw   r1      r2
    q  <<   q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,      0,
            q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,      0,
            0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,
            0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,
            0,      0,      0,      0,      s2qz_,  0,      0,      0,      0,      0,
            0,      0,      0,      0,      0,      s2qz_,  0,      0,      0,      0,
            0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,      0,
            0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,      0,
            0,      0,      0,      0,      0,      0,      0,      0,      q_r,    0,
            0,      0,      0,      0,      0,      0,      0,      0,      0,      q_r;
    // clang-format on
    return q;
}

Eigen::MatrixXd EnemyModel::update_R(const Eigen::VectorXd &z){
    Eigen::VectorXd r(z.size());
    for(int i = 0; i < (z.size() / 4); i++){
        r.segment(i *4 , 4) << r_pose, r_pose, abs(r_distance * pow(z[i * 4 + 2],2)), r_yaw;
    }
    return r.asDiagonal();
}

} // namespace rm_auto_aim
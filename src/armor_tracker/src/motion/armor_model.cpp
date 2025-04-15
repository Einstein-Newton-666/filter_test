#include "armor_tracker/motion/armor_model.hpp"

namespace rm_auto_aim {

void ArmorModel::initFilter(const Eigen::VectorXd& x0,const int& index){
    if(filters.find(index) != filters.end()){
        filters[index].kf.init(x0);
        filters[index].tracker_state = TrackerState::DETECTING;
        filters[index].tracking_thres = this->tracking_thres;
        filters[index].lost_thres = this->lost_thres;
    }
}
Eigen::MatrixXd ArmorModel::update_Q(const double& dt){
    Eigen::Matrix<double,X_N,X_N> q;
    double t = dt , x = s2qx_, y = s2qy_, z = s2qz_ , yaw = s2qyaw_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 3 * x, q_vx_vx = pow(t, 2) / 2 * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 3 * y, q_vy_vy = pow(t, 2) / 2 * y;
    double q_z_z = pow(t, 4) / 4 * z, q_z_vz = pow(t, 3) / 3 * z, q_vz_vz = pow(t, 2) / 2 * z;
    double q_yaw_yaw = pow(t, 4) / 4 * yaw;

    //      xa      v_xa    ya      v_ya    za      v_za    yaw           
    q  <<   q_x_x,  q_x_vx, 0,      0,      0,      0,      0,            
            q_x_vx, q_vx_vx,0,      0,      0,      0,      0,             
            0,      0,      q_y_y,  q_y_vy, 0,      0,      0,           
            0,      0,      q_y_vy, q_vy_vy,0,      0,      0,           
            0,      0,      0,      0,      q_z_z,  q_z_vz, 0,                
            0,      0,      0,      0,      q_z_vz, q_vz_vz,0,                  
            0,      0,      0,      0,      0,      0,      q_yaw_yaw;
    return q;
}

Eigen::MatrixXd ArmorModel::update_R(const Eigen::Matrix<double, Z_N, 1> &z){
    Eigen::VectorXd r(z.size());
    r << r_x * std::abs(z[0]), r_y * std::abs(z[1]), r_z * std::abs(z[2]), r_yaw;
    return r.asDiagonal();
}

} // namespace rm_auto_aim
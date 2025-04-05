#include "filter_test/filter.hpp"

template<typename T>
void ceres_xyz_to_ypd(const T& xyz, T& ypd) {
    ypd[0] = ceres::atan2(xyz[1], xyz[0]); // yaw
    ypd[1] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1])); // pitch
    ypd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]); // distance
};

// EKF
// xa = x_armor, xc = x_robot_center
// state: xc, v_xc, yc, v_yc, za1, za2, orient, v_yaw, r1, r2
// index: 0,  1,    2,  3,    4,  5,    6,      7,     8,   9,
// - yaw 需要在切换时 set?
// - 对于只有单独半径的装甲板， 不更新多余的 r 和 z
// measurement: xa, ya, za, yaw
// measurement2: yaw（相机）, pitch, distance, yaw(orient)
// f - Process function

struct Predict {
public:
    explicit Predict(const double& delta_t): delta_t(delta_t) {}
    template<typename T>
    void operator()(const T& x_pre, T& x_cur) const {
        x_cur[0] = x_pre[0] + this->delta_t * x_pre[1];
        x_cur[1] = x_pre[1];
        x_cur[2] = x_pre[2] + this->delta_t * x_pre[3];
        x_cur[3] = x_pre[3];
        x_cur[4] = x_pre[4];
        x_cur[5] = x_pre[5];
        x_cur[6] = x_pre[6] + this->delta_t * x_pre[7];
        x_cur[7] = x_pre[7];
        x_cur[8] = x_pre[8];
        x_cur[9] = x_pre[9];
    }
    
    //状态向量的大小
    int size = 10;

private:
    double delta_t = 0.;
};

struct MeasureSingle {
public:
    // z 用于接收. 由 x 推导到 z
    // z: 当前需求比较的装甲板，rotate: super_yaw 到需求装甲的 rotate
    // ekf 之与测量转观测量
    // 用装甲板进行更新J
    explicit MeasureSingle(const int i): I(i) {}
    template<typename T>
    void operator()(const T& x, T& z) const {
        // x[6] 是 super 板的 yaw
        const T xyz_armor = { 
            x[0] + ceres::cos(x[6] + M_PI_2 * I) * x[8 + I % 2],
            x[2] + ceres::sin(x[6] + M_PI_2 * I) * x[8 + I % 2],
            x[4 + I % 2] 
        };
        T ypd(3);  //这里T的类型是自定义类型的容器
        ceres_xyz_to_ypd(xyz_armor, ypd);
        for (int i = 0; i < 3; i++) {
            z[i] = ypd[i];
        }
        // orien_yaw = orien_yaw
        z[3] = x[6] + M_PI_2 * I;
    }

    //状态向量的大小
    int input_size = 10;
    //观测向量的大小
    int output_size = 4;
    //I 匹配到的装甲板的索引
    int I;

private:
};


struct MeasureDouble {
public:
    // z 用于接收. 由 x 推导到 z
    // z: 当前需求比较的装甲板，rotate: super_yaw 到需求装甲的 rotate
    // ekf 之与测量转观测量
    // 用装甲板进行更新J
    explicit MeasureDouble(int i, int j): I(i), J(j) {}
    template<typename T>
    void operator()(const T& x, T& z) const {
        int idx = 0;
        for (const auto &i : {I, J})
        {
            // x[6] 是 super 板的 yaw
            T xyz_armor = { 
                x[0] + ceres::cos(x[6] + M_PI_2 * i) * x[8 + i % 2],
                x[2] + ceres::sin(x[6] + M_PI_2 * i) * x[8 + i % 2],
                x[4 + i % 2] 
            };
            T ypd(3);
            ceres_xyz_to_ypd(xyz_armor, ypd);
            for (int j = 0; j < 3; j++) {
                z[idx + j] = ypd[j];
            }
            // orien_yaw = orien_yaw
            z[idx + 3] = x[6] + M_PI_2 * i;
            idx += 4;
        }
    }
    
    //状态向量的大小
    int input_size = 10;
    //观测向量的大小
    int output_size = 8;
    // I J 匹配到的装甲板的索引
    int I, J;

private:
};

ArmorFilter::ArmorFilter(){
    //参数瞎抄的
    init_r = 0.25;
    s2qxy_ = 3, s2qz_ = 0.0001, s2qyaw_ = 15, s2qr_ = 0.0005;
    r_pose = 0.002, r_distance = 0.1, r_yaw = 0.5;

}

void ArmorFilter::init(auto_aim_interfaces::msg::Armors::SharedPtr &armors_msg){
    //选择最中心的装甲板初始化 TODO：同时使用两块装甲板初始化
    int index;
    double min_yaw_diff = M_PI;
    for (int i = 0; i < armors_msg->armors.size(); i++){
        double yaw_diff = abs(orientationToYaw(armors_msg->armors[i].pose.orientation));
        if(yaw_diff < min_yaw_diff){
            min_yaw_diff = yaw_diff;
            index = i;
        }
    }

    Eigen::VectorXd x(10);
    double yaw = orientationToYaw(armors_msg->armors[index].pose.orientation);
    x << armors_msg->armors[index].pose.position.x - init_r * cos(yaw), 0,  
        armors_msg->armors[index].pose.position.y - init_r * sin(yaw), 0,
        armors_msg->armors[index].pose.position.z, armors_msg->armors[index].pose.position.z,
        yaw, 0, init_r, init_r;
    ekf.init_x(x);

    last_time_ = armors_msg->header.stamp;
}

Eigen::VectorXd ArmorFilter::update(const auto_aim_interfaces::msg::Armors::SharedPtr &armors_msg){
    if(armors_msg->armors.empty()){return ekf.get_x();}

    Eigen::VectorXd x(10) ;
    x = ekf.get_x();
    //提高yaw的偏差获取滤波器中匹配的装甲板的索引 
    //TODO:更高效的匹配和筛选方法，可以参考西工大代码https://github.com/SnocrashWang/WMJAimer/tree/master
    std::vector<int> index(armors_msg->armors.size());
    for (int i = 0; i < armors_msg->armors.size(); i++){
        double min_yaw_diff = 1e9;
        for (int j = 0; j < 4; j++)
        {
            double yaw_diff = abs(
                angles::shortest_angular_distance(
                    orientationToYaw(armors_msg->armors[i].pose.orientation), x[6] + j * M_PI_2
                )
            );
            if(yaw_diff < min_yaw_diff){
                min_yaw_diff = yaw_diff;
                index[i] = j;
            }
        }
    }

    //通过第一块装甲板获取yaw的修正值(通过第二块得到的值也应该一样，但是我没试过。。。)
    double yaw_offset = angles::normalize_angle(x[6] + index[0] * M_PI_2) - x[6] + index[0] * M_PI_2;
    //修正滤波器中的yaw值使滤波器能正常更新
    Eigen::VectorXd fix_x(10);
    fix_x = ekf.get_x();
    fix_x[6] += yaw_offset;
    ekf.set_x(fix_x);

    rclcpp::Time time_now = armors_msg->header.stamp;
    double dt = (time_now - last_time_).seconds();
    last_time_ = time_now;

    Eigen::VectorXd z_xyz(index.size() * 4);
    for(int i = 0; i < index.size(); i++){
        z_xyz.segment(i * 4, i * 4 + 3) << 
            armors_msg->armors[i].pose.position.x, 
            armors_msg->armors[i].pose.position.y,
            armors_msg->armors[i].pose.position.z, 
            orientationToYaw(armors_msg->armors[i].pose.orientation);
    }
    Eigen::VectorXd z_pyd(index.size() * 4);
    for(int i = 0; i < index.size(); i++){
        Eigen::Vector3d xyz = z_xyz.segment(i * 4, i * 4 + 2);
        Eigen::Vector3d pyd;
        ceres_xyz_to_ypd(xyz, pyd);
        z_pyd.segment(i * 4, i * 4 + 2) << pyd;
        z_pyd[i * 4 + 3] = z_xyz[i * 4 + 3];
    }

    //根据匹配到的装甲板的数量选择不同的观测方程
    switch (index.size())
    {
    case 2:
        ekf.update(MeasureDouble(index[0], index[1]), Predict(dt), z_pyd, get_q(dt), get_r(z_xyz));
        break;
    
    default:
        ekf.update(MeasureSingle(index[0]), Predict(dt), z_pyd, get_q(dt), get_r(z_xyz));
        break;
    }

    x = ekf.get_x();
    x[6] = angles::normalize_angle(x[6]);
    return x;
}

double ArmorFilter::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  yaw = angles::normalize_angle(yaw);
  return yaw;
}

Eigen::MatrixXd ArmorFilter::get_q(double dt_){
    Eigen::MatrixXd q(10, 10);
    double t = dt_, x = s2qxy_, y = s2qyaw_, r = s2qr_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
    double q_r = pow(t, 4) / 4 * r;
    // clang-format off
    //    xc      v_xc    yc      v_yc    za1     za2     yaw     v_yaw   r1      r2
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,      0,
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
};

Eigen::MatrixXd ArmorFilter::get_r(Eigen::VectorXd & z){
    Eigen::VectorXd r(z.size());
    for(int i = 0; i < (z.size() / 4); i++){
        r.segment(i *4 , i * 4 + 3) << r_pose, r_pose, abs(r_distance * z[3 + i * 4]), r_yaw;
    }
    return r.asDiagonal();
};

// int main(){
//     ArmorFilter af;
//     auto_aim_interfaces::msg::Armors::SharedPtr armors_msg = std::make_shared<auto_aim_interfaces::msg::Armors>();;
//     auto_aim_interfaces::msg::Armor armor;
//     tf2::Quaternion q;
//     q.setRPY(0, 0, 1.5708);  // 绕 Z 轴旋转 90 度（弧度）
//     armor.pose.orientation.x = q.x();
//     armor.pose.orientation.y = q.y();
//     armor.pose.orientation.z = q.z();
//     armor.pose.orientation.w = q.w();
//     armor.pose.position.x = 1.0;
//     armor.pose.position.y = 2.0;
//     armor.pose.position.z = 0.5;
//     armors_msg->armors.push_back(armor);

//     armors_msg->header.stamp = rclcpp::Clock().now();
//     af.init(armors_msg);
//     armors_msg->header.stamp = rclcpp::Clock().now() + rclcpp::Duration::from_seconds(0.01);
//     auto state = af.update(armors_msg);
//     std::cout << state <<std::endl; 
// }
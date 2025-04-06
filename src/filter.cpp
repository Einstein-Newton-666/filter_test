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
// index: 0,  1,    2,  3,    4,   5,   6,      7,     8,   9,
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
        // T ypd(3);  //这里T的类型是自定义类型的容器
        // ceres_xyz_to_ypd(xyz_armor, ypd);
        // for (int i = 0; i < 3; i++) {
        //     z[i] = ypd[i];
        // }
        z[0] = ceres::atan2(xyz_armor[1], xyz_armor[0]); // yaw
        z[1] = ceres::atan2(xyz_armor[2], ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1])); // pitch
        z[2] = ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1] + xyz_armor[2] * xyz_armor[2]); // distance
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
            // T ypd(3);
            // ceres_xyz_to_ypd(xyz_armor, ypd);
            // for (int j = 0; j < 3; j++) {
            //     z[idx + j] = ypd[j];
            // }
            z[idx + 0] = ceres::atan2(xyz_armor[1], xyz_armor[0]); // yaw
            z[idx + 1] = ceres::atan2(xyz_armor[2], ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1])); // pitch
            z[idx + 2] = ceres::sqrt(xyz_armor[0] * xyz_armor[0] + xyz_armor[1] * xyz_armor[1] + xyz_armor[2] * xyz_armor[2]); // distance
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
    s2qxy_ = 3, s2qz_ = 0.01, s2qyaw_ = 15, s2qr_ = 0.5;
    r_pose = 0.02, r_distance = 0.1, r_yaw = 0.6;

    last_armor_number = 0;

    last_yaw = 0;
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
    last_yaw = yaw;
    last_armor_number = armors_msg->armors.size();
}

Eigen::VectorXd ArmorFilter::update(const auto_aim_interfaces::msg::Armors::SharedPtr &armors_msg){
    if(armors_msg->armors.empty()){return ekf.get_x();}
    int armor_number = armors_msg->armors.size();
    // if(armor_number == 2 && last_armor_number == 1) {last_yaw += M_PI_2; std::cout<<" s "<<std::endl;}
    last_armor_number = armor_number;

    rclcpp::Time time_now = armors_msg->header.stamp;
    double dt = (time_now - last_time_).seconds();
    last_time_ = time_now;

    Eigen::VectorXd x(10) ;
    x = ekf.predict(Predict(dt)).x_p;
    //获取滤波器中匹配的装甲板的索引 
    //TODO:更高效的匹配和筛选方法，可以参考西工大代码https://github.com/SnocrashWang/WMJAimer/tree/master
    // std::vector<int> index(armors_msg->armors.size());
    // std::vector<double> offset(armors_msg->armors.size());
    // for (int i = 0; i < armors_msg->armors.size(); i++){
    //     double min_match_diff = 1e9;
    //     for (int j = 0; j < 4; j++){
    //         double yaw_1 = orientationToYaw(armors_msg->armors[i].pose.orientation);
    //         double yaw_2 = x[6] + M_PI_2 * j;
    //         // if(angles::normalize_angle(yaw_2) > M_PI_2 && angles::normalize_angle(yaw_2) < -M_PI_2) continue; //删除后两块装甲板
    //         // double x_1 = x[0] + cos(x[6] + M_PI_2 * j) * x[8 + j % 2];
    //         // double x_2 = armors_msg->armors[i].pose.position.x;
    //         // double y_1 = x[2] + sin(x[6] + M_PI_2 * j) * x[8 + j % 2];
    //         // double y_2 = armors_msg->armors[i].pose.position.y;
    //         double match_diff = abs(angles::shortest_angular_distance(yaw_1, yaw_2));
    //         // std::cout << match_diff << " ";
    //         if(match_diff < min_match_diff){
    //             // if(!index.empty()){
    //             //     if(j == index[0]) continue; //防止匹配到同一块装甲板上
    //             // }
    //             min_match_diff = match_diff;
    //             index[i] = j;
    //             double yaw_offset = floor((yaw_2 - yaw_1 + M_PI_2)/ M_PI /2.) * M_PI * 2;
    //             offset.push_back(yaw_offset);
    //         }
    //     }
    //     if(last_armor == 1 && armor == 2){std::cout << min_match_diff <<"    ";}
    //     // std::cout << min_match_diff  <<std::endl;
    //     // std::cout << index[i]  <<std::endl;
    //     // if(min_match_diff > M_PI_4){
    //         // double yaw_1 = orientationToYaw(armors_msg->armors[i].pose.orientation);
    //         // std::cout << min_match_diff << std::endl << yaw_1 <<std::endl;
    //         // for (int j = 0; j < 4; j++){
    //             // double yaw_2 = x[6] + M_PI_2 * index[i];
    //             // double yaw_2 = x[6] + M_PI_2 * j;
    //             // std::cout<<yaw_2<< "   "<< abs(angles::shortest_angular_distance(yaw_1, yaw_2)) <<std::endl;
    //         // }
    //     // }
    // }

    auto calculate_yaw_diff = [&](int idx, double observed_yaw) {
        const double predicted_yaw = x[6] + idx * M_PI_2;
        return std::abs(angles::shortest_angular_distance(observed_yaw, predicted_yaw));
      };
    
      std::vector<int> index;
      // 单装甲板匹配
      if (armors_msg->armors.size() == 1) {
        const double obs_yaw = orientationToYaw(armors_msg->armors[0].pose.orientation);
        
        // 直接寻找最小角度差索引
        int best_idx = 0;
        double min_diff = calculate_yaw_diff(0, obs_yaw);
        for (int i = 1; i < 4; ++i) {
          const double diff = calculate_yaw_diff(i, obs_yaw);
          if (diff < min_diff) {
            min_diff = diff;
            best_idx = i;
          }
        }
        index.push_back(best_idx);
      }
      // 双装甲板匹配
      else if (armors_msg->armors.size() == 2) {
        // 仅允许相邻装甲板对
        constexpr std::array<std::pair<int, int>, 4> adjacent_pairs = {
          {{0,1}, {1,2}, {2,3}, {3,0}}
        };
    
        // 获取观测角度（已处理连续性）
        const double yaw1 = orientationToYaw(armors_msg->armors[0].pose.orientation);
        const double yaw2 = orientationToYaw(armors_msg->armors[1].pose.orientation);
    
        // 寻找最佳相邻对
        auto best_pair = adjacent_pairs[0];
        double min_total_diff = std::numeric_limits<double>::max();
        
        for (const auto& pair : adjacent_pairs) {
          // 考虑两种顺序匹配可能性
          const double diff1 = calculate_yaw_diff(pair.first, yaw1) + 
                              calculate_yaw_diff(pair.second, yaw2);
          const double diff2 = calculate_yaw_diff(pair.second, yaw1) + 
                              calculate_yaw_diff(pair.first, yaw2);
          const double total_diff = std::min(diff1, diff2);
          
          if (total_diff < min_total_diff) {
            min_total_diff = total_diff;
            best_pair = diff1 < diff2 ? std::make_pair(pair.first, pair.second) : std::make_pair(pair.second, pair.first);
          }
        }
        
        index = {best_pair.first, best_pair.second};
      }

    //   for (auto &&i : index)
    //   {
    //     std::cout << i <<"  ";
    //   }
    //   std::cout << std::endl;

    Eigen::VectorXd z_xyz(index.size() * 4);
    for(int i = 0; i < index.size(); i++){
        z_xyz[i * 4] = armors_msg->armors[i].pose.position.x;
        z_xyz[i * 4 + 1] = armors_msg->armors[i].pose.position.y;
        z_xyz[i * 4 + 2] = armors_msg->armors[i].pose.position.z;
        // z_xyz[i * 4 + 3] = orientationToYaw(armors_msg->armors[i].pose.orientation) + offset[i];
        z_xyz[i * 4 + 3] = orientationToYaw(armors_msg->armors[i].pose.orientation);

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
        ekf.update(MeasureDouble(index[0], index[1]), Predict(dt), z_pyd, get_q(dt), get_r(z_pyd));
        break;
    
    default:
        ekf.update(MeasureSingle(index[0]), Predict(dt), z_pyd, get_q(dt), get_r(z_pyd));
        break;
    }

    x = ekf.get_x();
    // x[6] = angles::normalize_angle(x[6]);
    // ekf.set_x(x);
    last_yaw = x[6];
    // if(abs(x[7]) > M_PI) x[7] = x[7] / abs(x[7]) * M_PI;
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
    //yaw = angles::normalize_angle(yaw);
    yaw = last_yaw + angles::shortest_angular_distance(last_yaw, yaw);
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
        r.segment(i *4 , i * 4 + 3) << r_pose, r_pose, abs(r_distance * z[i * 4 + 2]), r_yaw;
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
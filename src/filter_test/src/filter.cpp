#include "filter_test/filter.hpp"
#include "filter_test/cv_model.hpp"
#include "filter_test/singer_model.hpp"

//1 ekf 0 ukf
#define FILTER 1

//1 cv 0 singer
#define MODEL 0

#if MODEL == 1
using namespace cv_model;
#else
using namespace singer_model;
#endif

ArmorFilter::ArmorFilter(){
    //参数瞎抄的
    init_r = 0.25;
    s2qxyz_ = 0.5, s2qyaw_ = 0.5, s2qr_ = 10.0;
    r_pose = 0.01, r_distance = 0.01, r_yaw = 0.01;

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

#if FILTER == 1
    ekf.init(x);
#else
    ukf.init(x);
#endif

    last_time_ = armors_msg->header.stamp;
    last_yaw = yaw;
    last_armor_number = armors_msg->armors.size();
}

Eigen::VectorXd ArmorFilter::update(const auto_aim_interfaces::msg::Armors::SharedPtr &armors_msg){
    if(armors_msg->armors.empty()){
#if FILTER == 1
        return ekf.getState();
#else
        return ukf.getState();
#endif
    }
    int armor_number = armors_msg->armors.size();
    // if(armor_number == 2 && last_armor_number == 1) {last_yaw += M_PI_2; std::cout<<" s "<<std::endl;}
    last_armor_number = armor_number;

    rclcpp::Time time_now = armors_msg->header.stamp;
    double dt = (time_now - last_time_).seconds();
    last_time_ = time_now;

    Eigen::VectorXd x(10) ;
#if FILTER == 1
    x = ekf.predict(Predict(dt)).x_pri;
#else
    x = ukf.predict(Predict(dt));
#endif
    
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

    Eigen::VectorXd z_xyz(index.size() * 4);
    for(int i = 0; i < index.size(); i++){
        z_xyz[i * 4] = armors_msg->armors[i].pose.position.x;
        z_xyz[i * 4 + 1] = armors_msg->armors[i].pose.position.y;
        z_xyz[i * 4 + 2] = armors_msg->armors[i].pose.position.z;
        z_xyz[i * 4 + 3] = orientationToYaw(armors_msg->armors[i].pose.orientation);

    }
    Eigen::VectorXd z_pyd(index.size() * 4);
    for(int i = 0; i < index.size(); i++){
        Eigen::Vector3d xyz = z_xyz.segment(i * 4, 3);
        Eigen::Vector3d pyd;
        ceres_xyz_to_ypd(xyz, pyd);
        z_pyd.segment(i * 4, 3) << pyd;
        z_pyd[i * 4 + 3] = z_xyz[i * 4 + 3];
    }

    //根据匹配到的装甲板的数量选择不同的观测方程
    switch (index.size())
    {
    case 2:
#if FILTER == 1
        ekf.update(MeasureDouble(index[0], index[1]), Predict(dt), z_pyd, get_q(dt), get_r(z_pyd));
#else        
        ukf.update(MeasureDouble(index[0], index[1]), Predict(dt), z_pyd, get_q(dt), get_r(z_pyd));
#endif

        break;
    
    default:
#if FILTER == 1
        ekf.update(MeasureSingle(index[0]), Predict(dt), z_pyd, get_q(dt), get_r(z_pyd));
#else
        ukf.update(MeasureSingle(index[0]), Predict(dt), z_pyd, get_q(dt), get_r(z_pyd));
#endif
        break;
    }
#if FILTER == 1
    x = ekf.getState();
#else
    x = ukf.getState();
#endif
    last_yaw = x[6];
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
    return predict_q(dt_, s2qxyz_, s2qyaw_, s2qr_);
};

Eigen::MatrixXd ArmorFilter::get_r(Eigen::VectorXd & z){
    return measure_r(z, r_pose, r_distance, r_yaw);
};

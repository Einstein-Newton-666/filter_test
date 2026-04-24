#include "filter_test/filter.hpp"
#include "filter_test/cv_model.hpp"
#include "filter_test/singer_model.hpp"

ArmorFilter::ArmorFilter(bool use_ekf, bool use_cv_model,
    double s2qxy_cv, double s2qz_cv, double s2qyaw_cv, double s2qr_cv, double s2qdz_cv,
    double s2qxy_singer, double s2qz_singer, double s2qyaw_singer, double s2qr_singer, double s2qdz_singer,
    double tau_singer,
    double ukf_alpha, double ukf_beta, double ukf_kappa)
    : use_ekf_(use_ekf), use_cv_model_(use_cv_model)
{
    init_r = 0.25;
    
    // CV模型过程噪声参数
    s2qxy_cv_ = s2qxy_cv;
    s2qz_cv_ = s2qz_cv;
    s2qyaw_cv_ = s2qyaw_cv;
    s2qr_cv_ = s2qr_cv;
    s2qdz_cv_ = s2qdz_cv;
    
    // Singer模型过程噪声参数
    s2qxy_singer_ = s2qxy_singer;
    s2qz_singer_ = s2qz_singer;
    s2qyaw_singer_ = s2qyaw_singer;
    s2qr_singer_ = s2qr_singer;
    s2qdz_singer_ = s2qdz_singer;
    
    // Singer模型机动时间常数
    tau_singer_ = tau_singer;
    
    // UKF参数
    ukf_alpha_ = ukf_alpha;
    ukf_beta_ = ukf_beta;
    ukf_kappa_ = ukf_kappa;

    r_pose_ = 0.01, r_distance_ = 0.01, r_yaw_ = 0.01;
    use_fixed_r_ = false;

    last_armor_number = 0;

    last_yaw = 0;
    last_result_.resize(11);
    last_result_.setZero();
}

void ArmorFilter::init(auto_aim_interfaces::msg::Armors::SharedPtr &armors_msg){
    //选择最中心的装甲板初始化 TODO：同时使用两块装甲板初始化
    int index;
    double min_yaw_diff = M_PI;
    for (size_t i = 0; i < armors_msg->armors.size(); i++){
        double yaw_diff = abs(orientationToYaw(armors_msg->armors[i].pose.orientation));
        if(yaw_diff < min_yaw_diff){
            min_yaw_diff = yaw_diff;
            index = i;
        }
    }

    double yaw = orientationToYaw(armors_msg->armors[index].pose.orientation);
    Eigen::VectorXd x;
    if (use_cv_model_) {
        // cv_model: x = [xc, vxc, yc, vyc, za, vza, yaw, vyaw, r1, r2, dz], size = 11
        x.resize(11);
        x << armors_msg->armors[index].pose.position.x + init_r * cos(yaw), 0,  
            armors_msg->armors[index].pose.position.y + init_r * sin(yaw), 0,
            armors_msg->armors[index].pose.position.z, 0,
            yaw, 0, init_r, init_r, 0;  // dz = 0 (初始时认为两块装甲板等高)
    } else {
        // singer_model: x = [x, vx, ax, y, vy, ay, z, vz, az, yaw, vyaw, ayaw, r1, r2, dz], size = 15
        x.resize(15);
        x << armors_msg->armors[index].pose.position.x + init_r * cos(yaw), 0, 0,
            armors_msg->armors[index].pose.position.y + init_r * sin(yaw), 0, 0,
            armors_msg->armors[index].pose.position.z, 0, 0,
            yaw, 0, 0,
            init_r, init_r, 0;  // dz = 0 (初始时认为两块装甲板等高)
    }

    if (use_ekf_) {
        ekf.init(x);
    } else {
        ukf.init(x);
        ukf.setParams(ukf_alpha_, ukf_beta_, ukf_kappa_);
    }

    last_time_ = armors_msg->header.stamp;
    last_yaw = yaw;
    last_armor_number = armors_msg->armors.size();
}

Eigen::VectorXd ArmorFilter::update(const auto_aim_interfaces::msg::Armors::SharedPtr &armors_msg){
    if(armors_msg->armors.empty()){
        // 即使没有观测，也要更新时间戳，避免下次dt过大
        last_time_ = armors_msg->header.stamp;
        if (use_ekf_) {
            return ekf.getState();
        } else {
            return ukf.getState();
        }
    }
    int armor_number = armors_msg->armors.size();
    // if(armor_number == 2 && last_armor_number == 1) {last_yaw += M_PI_2; std::cout<<" s "<<std::endl;}
    last_armor_number = armor_number;

    rclcpp::Time time_now = armors_msg->header.stamp;
    double dt = (time_now - last_time_).seconds();
    last_time_ = time_now;

    // 计算预测状态x（用于匹配）
    Eigen::VectorXd x(use_cv_model_ ? 11 : 15);
    if (use_cv_model_) {
        cv_model::Predict predict_obj(dt);
        if (use_ekf_) {
            x = ekf.predict(predict_obj).x_pri;
        } else {
            x = ukf.predict(predict_obj);
        }
    } else {
        singer_model::Predict predict_obj(dt, tau_singer_);
        if (use_ekf_) {
            x = ekf.predict(predict_obj).x_pri;
        } else {
            x = ukf.predict(predict_obj);
        }
    }
    
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
        // cv_model: yaw at index 6, singer_model: yaw at index 9
        const double predicted_yaw = (use_cv_model_ ? x[6] : x[9]) + idx * M_PI_2;
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

        auto best_pair = adjacent_pairs[0];
        double min_total_diff = std::numeric_limits<double>::max();
        
        for (const auto& pair : adjacent_pairs) {
          // 考虑两种顺序匹配可能性
          const double diff1 = calculate_yaw_diff(pair.first, yaw1) + calculate_yaw_diff(pair.second, yaw2);
          const double diff2 = calculate_yaw_diff(pair.second, yaw1) + calculate_yaw_diff(pair.first, yaw2);
          const double total_diff = std::min(diff1, diff2);
          
          if (total_diff < min_total_diff) {
            min_total_diff = total_diff;
            best_pair = diff1 < diff2 ? std::make_pair(pair.first, pair.second) : std::make_pair(pair.second, pair.first);
          }
        }

        index = {best_pair.first, best_pair.second};
    }

    Eigen::VectorXd z_xyz(index.size() * 4);
    for (size_t i = 0; i < index.size(); i++) {
        z_xyz[i * 4] = armors_msg->armors[i].pose.position.x;
        z_xyz[i * 4 + 1] = armors_msg->armors[i].pose.position.y;
        z_xyz[i * 4 + 2] = armors_msg->armors[i].pose.position.z;
        z_xyz[i * 4 + 3] = orientationToYaw(armors_msg->armors[i].pose.orientation);
    }

    Eigen::VectorXd z_pyd(index.size() * 4);
    for (size_t i = 0; i < index.size(); i++) {
        Eigen::Vector3d xyz = z_xyz.segment(i * 4, 3);
        Eigen::Vector3d pyd;
        ceres_xyz_to_ypd(xyz, pyd);
        z_pyd.segment(i * 4, 3) << pyd;
        z_pyd[i * 4 + 3] = z_xyz[i * 4 + 3];
    }

    auto yaw_deviation = [&](double meas_yaw) {
        double xc = x[0];
        double yc = use_cv_model_ ? x[2] : x[3];
        return std::abs(normalize_angle(meas_yaw - std::atan2(yc, xc))) * 180.0 / M_PI;
    };

    std::vector<double> abs_yaws;
    for (size_t i = 0; i < index.size(); ++i) {
        abs_yaws.push_back(yaw_deviation(z_pyd[i * 4 + 3]));
    }

    // 通用滤波器更新 lambda
    auto do_predict = [&]() {
        auto Q = get_q(dt);
        if (use_ekf_) {
            if (use_cv_model_) ekf.predict_forward(cv_model::Predict(dt), Q);
            else ekf.predict_forward(singer_model::Predict(dt, tau_singer_), Q);
        } else {
            if (use_cv_model_) ukf.predict_forward(cv_model::Predict(dt), Q);
            else ukf.predict_forward(singer_model::Predict(dt, tau_singer_), Q);
        }
    };

    auto do_update = [&](auto& measure_obj, Eigen::VectorXd& obs, Eigen::MatrixXd& R) {
        if (use_ekf_) {
            auto result = ekf.measure(measure_obj);
            for (int i = 0; i < obs.size() / 4; ++i) {
                obs[i * 4] = get_closest_angle(obs[i * 4], result.z_pri[i * 4]);
                obs[i * 4 + 3] = get_closest_angle(obs[i * 4 + 3], result.z_pri[i * 4 + 3]);
            }
            ekf.update_forward(measure_obj, obs, R);
        } else {
            auto z_pri = ukf.measure(measure_obj);
            for (int i = 0; i < obs.size() / 4; ++i) {
                obs[i * 4] = get_closest_angle(obs[i * 4], z_pri[i * 4]);
                obs[i * 4 + 3] = get_closest_angle(obs[i * 4 + 3], z_pri[i * 4 + 3]);
            }
            ukf.update_forward(measure_obj, obs, R);
        }
    };

    //根据匹配到的装甲板的数量选择不同的观测方程
    switch (index.size())
    {
    case 2: {
        if (use_cv_model_) {
            cv_model::MeasureDouble measure_double(index[0], index[1]);
            auto R = get_r_with_abs_yaw(z_pyd, abs_yaws);
            do_predict();
            do_update(measure_double, z_pyd, R);
        } else {
            singer_model::MeasureDouble measure_double(index[0], index[1]);
            auto R = get_r_with_abs_yaw(z_pyd, abs_yaws);
            do_predict();
            do_update(measure_double, z_pyd, R);
        }
        break;
    }

    default:
        if (use_cv_model_) {
            cv_model::MeasureSingle measure_single(index[0]);
            auto R = get_r_with_abs_yaw(z_pyd, abs_yaws);
            do_predict();
            do_update(measure_single, z_pyd, R);
        } else {
            singer_model::MeasureSingle measure_single(index[0]);
            auto R = get_r_with_abs_yaw(z_pyd, abs_yaws);
            do_predict();
            do_update(measure_single, z_pyd, R);
        }
        break;
    }

    if (use_ekf_) {
        x = ekf.getState();
    } else {
        x = ukf.getState();
    }

    // 状态约束: r1, r2, dz clamp
    if (use_cv_model_) {
        // cv_model: r1 at index 8, r2 at index 9, dz at index 10
        x[8] = std::clamp(x[8], 0.15, 0.4);   // r1
        x[9] = std::clamp(x[9], 0.15, 0.4);   // r2
        x[10] = std::clamp(x[10], -0.1, 0.1); // dz
    } else {
        // singer_model: r1 at index 12, r2 at index 13, dz at index 14
        x[12] = std::clamp(x[12], 0.15, 0.4);  // r1
        x[13] = std::clamp(x[13], 0.15, 0.4);  // r2
        x[14] = std::clamp(x[14], -0.1, 0.1);  // dz
    }
    // 将约束后的状态设置回滤波器
    if (use_ekf_) {
        ekf.setState(x);
    } else {
        ukf.setState(x);
    }
    
    // cv_model: yaw at index 6, singer_model: yaw at index 9
    last_yaw = use_cv_model_ ? x[6] : x[9];
    
    Eigen::VectorXd result(11);
    if (use_cv_model_) {
        // cv_model: [xc, vxc, yc, vyc, za, vza, yaw, vyaw, r1, r2, dz]
        result << x[0], x[2], x[4], x[1], x[3], x[5], x[6], x[7], x[8], x[9], x[10];
    } else {
        // singer_model: [x, vx, ax, y, vy, ay, z, vz, az, yaw, vyaw, ayaw, r1, r2, dz]
        result << x[0], x[3], x[6], x[1], x[4], x[7], x[9], x[10], x[12], x[13], x[14];
    }
    last_result_ = result;
    return result;

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
    if (use_cv_model_) {
        return cv_model::predict_q(dt_, s2qxy_cv_, s2qz_cv_, s2qyaw_cv_, s2qr_cv_, s2qdz_cv_);
    } else {
        return singer_model::predict_q(dt_, s2qxy_singer_, s2qz_singer_, s2qyaw_singer_, s2qr_singer_, s2qdz_singer_, tau_singer_);
    }
};

void ArmorFilter::set_r_for_simulation(double r_pose, double r_distance, double r_yaw) {
    r_pose_ = r_pose;
    r_distance_ = r_distance;
    r_yaw_ = r_yaw;
    use_fixed_r_ = true;  // 固定R模式，无PnP误差
}

void ArmorFilter::set_r_for_detector(double r_pose, double r_distance, double r_yaw) {
    r_pose_ = r_pose;
    r_distance_ = r_distance;
    r_yaw_ = r_yaw;
    use_fixed_r_ = false;  // 距离相关R模式
}

Eigen::MatrixXd ArmorFilter::get_r(Eigen::VectorXd & z){
    if (use_cv_model_) {
        return cv_model::measure_r(z, r_pose_, r_distance_, r_yaw_, {}, use_fixed_r_);
    } else {
        return singer_model::measure_r(z, r_pose_, r_distance_, r_yaw_, {}, use_fixed_r_);
    }
};

Eigen::MatrixXd ArmorFilter::get_r_with_abs_yaw(Eigen::VectorXd & z, std::vector<double> & abs_yaws){
    if (use_cv_model_) {
        return cv_model::measure_r(z, r_pose_, r_distance_, r_yaw_, abs_yaws, use_fixed_r_);
    } else {
        return singer_model::measure_r(z, r_pose_, r_distance_, r_yaw_, abs_yaws, use_fixed_r_);
    }
};

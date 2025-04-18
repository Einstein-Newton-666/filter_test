#include "armor_tracker/tracker.hpp"

namespace rm_auto_aim {

// 查找优先级最高的敌人作为跟踪装甲板
bool Tracker::findTrackerArmor(const auto_aim_interfaces::msg::Armors::SharedPtr & armors_msg){
    if(armors_msg->armors.size() == 0){
        match_state = MATCH_NONE;
        return false;
    }
    int priority = 0;
    Armor tracked_armor = armors_msg->armors[0];
    for (const auto & armor : armors_msg->armors) {
        if(armor.priority >=priority){
            priority = armor.priority;
            tracked_armor = armor;
        }
    }
    if(trackered_enemy_id != tracked_armor.number){
        trackered_enemy_id = tracked_armor.number;
        changed_tracked_armor = true;
    }else{
        changed_tracked_armor = false;
    }
    
    enemy_armor_num = (tracked_armor.number == "outpost")? OUTPOST_3 : NORMAL_4;
    // TODO:优化装甲板切换
    // RCLCPP_INFO(rclcpp::get_logger("armor_tracker"), "跟踪目标：%s", trackered_enemy_id.c_str());
    return true;
}

bool Tracker::updateTrackerArmor(const auto_aim_interfaces::msg::Armors::SharedPtr & armors_msg){
    int count = 0;
    tracked_armors.clear();
    for(const auto & armor : armors_msg->armors){
        if(armor.number == trackered_enemy_id){
            count++;
            double orientation_yaw = orientationToYaw(armor.pose.orientation); //TODO：处理yaw连续性
            // TODO：在更新跟踪装甲板的时候便算出装甲板的ypd
            Eigen::Vector3d xyz = {armor.pose.position.x, armor.pose.position.y, armor.pose.position.z};
            Eigen::Vector3d pyd;
            ceres_xyz_to_ypd(xyz,pyd);
            TrackedArmor a = {xyz[0], xyz[1], xyz[2], orientation_yaw, pyd[0], pyd[1], pyd[2]};
            tracked_armors.push_back(a); // 根据跟踪装甲板的id，将装甲板加入跟踪列表
        }
    }
    std::cout<<"count:"<<count<<std::endl;
    // RCLCPP_INFO(rclcpp::get_logger("armor_tracker"), "识别装甲板数量：%d", count);
    if(count == 0){
        match_state = MATCH_NONE;
    }else if(count == 1){
        match_state = MATCH_SINGLE;
    }else if(count == 2){
        match_state = MATCH_DOUBLE;
    }else{// 推进跟踪器，找到最匹配的两个装甲板 or 不跟踪，清空装甲板（选择后者）
        match_state = MATCH_NONE;
        tracked_armors.clear();
        // RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "同时识别到相同兵种的三个装甲板，请检查识别算法！");
        // 如果采用前者，tracked_armors中可能存在3个及以上装甲板，使用定长数组时要避免越界访问
    }
    return true;
}

void Tracker::initArmorModel(){
    //无论识别到一个装甲板还是两个装甲板，都初始化两个跟踪器
    if(match_state == MATCH_SINGLE){
        Eigen::VectorXd x0(7);
        x0 << tracked_armors[0].x, 0, tracked_armors[0].y, 0, tracked_armors[0].z, 0, tracked_armors[0].orientation_yaw;
        armor_model.initFilter(x0, 1);
        armor_model.filters[2].tracker_state = LOST;
        std::cout<<"初始化一个装甲板模型"<<std::endl;
    }else if(match_state == MATCH_DOUBLE){
        Eigen::VectorXd x0(7), x1(7);
        x0 << tracked_armors[0].x, 0, tracked_armors[0].y, 0, tracked_armors[0].z, 0, tracked_armors[0].orientation_yaw;
        x1 << tracked_armors[1].x, 0, tracked_armors[1].y, 0, tracked_armors[1].z, 0, tracked_armors[1].orientation_yaw;
        armor_model.initFilter(x0, 1);
        armor_model.initFilter(x1, 2);
        std::cout<<"初始化两个装甲板模型"<<std::endl;
    }
    
}

void Tracker::updateArmorModel(){
    // 更新初始化跟踪器的先验估计值
    for(size_t i = 1; i <= 2; i++){
        if(armor_model.filters[i].tracker_state != LOST){
            // std::cout<<"更新跟踪器"<<i<<"的先验估计值"<<std::endl;
            Eigen::VectorXd x_post = armor_model.filters[i].kf.getState();
            std::cout<<x_post<<std::endl;
            armor_model.filters[i].pri_estimation = armor_model.filters[i].kf.predict(FuncA(dt), armor_model.update_Q(dt));
        }
        armor_model.filters[i].matched = false;
    }
    if(match_state != MATCH_NONE){
        // 遍历跟踪器装甲板，如果跟预测装甲板在阈值内，就更新跟踪器；
        // 如果不在阈值内，就重置跟踪器，将该装甲板作为跟踪器的初始状态
        for (const auto& armor : tracked_armors){
            Eigen::VectorXd measurement (7);
            measurement << armor.x, armor.y, armor.z, armor.orientation_yaw, armor.yaw, armor.pitch, armor.distance;
            // 找到最接近的跟踪器装甲板
            double min_diff = DBL_MAX;
            double position_diff = DBL_MAX;
            int best_index;
            for(size_t i = 1; i <= 2; i++){
                if(armor_model.filters[i].matched || armor_model.filters[i].tracker_state == LOST){ continue;} // 如果该跟踪器装甲板已经被匹配，则跳过
                position_diff = pow(armor.x - armor_model.filters[i].pri_estimation[0],2)
                                    + pow(armor.y - armor_model.filters[i].pri_estimation[2],2)
                                    + pow(armor.z - armor_model.filters[i].pri_estimation[4],2);
                if(position_diff < min_diff){
                    min_diff = position_diff;
                    best_index = i;
                }
            }
            // 判断装甲板与跟踪器装甲板是否在阈值内
            std::cout<<"position_diff: "<<position_diff<<std::endl;
            if(position_diff < armor_model.position_diff_thres){
                Eigen::Matrix<double, 4, 1> z = {armor.x, armor.y, armor.z, armor.orientation_yaw};
                armor_model.filters[best_index].measurement = measurement;
                armor_model.filters[best_index].kf.update(FuncH(), z, armor_model.update_R(z));
                armor_model.filters[best_index].matched = true;
                std::cout<<"更新跟踪器"<<best_index<<"的装甲板"<<std::endl;
            }else{
                // 如果超过阈值，同时另一个跟踪器处于非LOST状态，则重置该跟踪器
                // 如果另一个跟踪器处于LOST状态，则初始化另一个跟踪器
                int another_index = (best_index == 1)? 2 : 1;
                Eigen::VectorXd x0(7);
                x0 << armor.x, 0, armor.y, 0, armor.z, 0, armor.orientation_yaw;
                if(armor_model.filters[another_index].tracker_state != LOST){
                    armor_model.filters[best_index].kf.init(x0);
                    armor_model.filters[best_index].measurement = measurement;
                    armor_model.filters[best_index].tracker_state = LOST;
                    armor_model.filters[best_index].matched = true;
                    std::cout<<"重置跟踪器"<<best_index<<"的装甲板"<<std::endl;
        
                }else{
                    armor_model.filters[another_index].kf.init(x0);
                    armor_model.filters[another_index].measurement = measurement;
                    armor_model.filters[another_index].tracker_state = LOST;
                    armor_model.filters[another_index].matched = true;
                    std::cout<<"初始化跟踪器"<<another_index<<"的装甲板"<<std::endl;
                }
            }
        }
    }

    // 更新装甲板跟踪器状态
    for (auto& filter : armor_model.filters) {
        filter.second.post_estimation = filter.second.kf.getState();
        updateTrackerState(filter.second,filter.second.matched);
    }
}

void Tracker::initEnemyModel(){
    //TODO：考虑第一眼看到目标时敌人已经高速小陀螺时如何进行v_yaw初始化而不会跟丢？
    double init_r = 0.25; // 默认初始化半径为过洞车半径
    Eigen::VectorXd x(10);
    double orientation_yaw = tracked_armors[0].orientation_yaw;
    x << tracked_armors[0].x + init_r * cos(orientation_yaw), 0,
        tracked_armors[0].y + init_r * sin(orientation_yaw), 0,
        tracked_armors[0].z, tracked_armors[0].z,
        orientation_yaw, 0, init_r, init_r;
    // if(match_state == MATCH_SINGLE){// 只看到一个装甲板
    //     // RCLCPP_INFO(rclcpp::get_logger("armor_tracker"), "整车跟踪器初始装甲板为一！");
    //     std::cout<<"MATCH_SINGLE"<<std::endl;
    //     x << tracked_armors[0].x + init_r * cos(orientation_yaw), 0,
    //         tracked_armors[0].y + init_r * sin(orientation_yaw), 0,
    //         tracked_armors[0].z, tracked_armors[0].z,
    //         orientation_yaw, 0, init_r, init_r;
    //         std::cout<<"x:"<<x<<std::endl;
        
    // }else if(match_state == MATCH_DOUBLE){// 同时看到两个装甲板
    //     // 不考虑误识别的情况，直接拿前两个装甲板初始化
    //     // 选择yaw最小的装甲板的yaw作为初始yaw和初始r1
    //     // RCLCPP_INFO(rclcpp::get_logger("armor_tracker"), "整车跟踪器初始装甲板为二！");
    //     std::cout<<"MATCH_DOUBLE"<<std::endl;
    //     int index = tracked_armors[0].orientation_yaw < tracked_armors[1].orientation_yaw ? 0 : 1;
    //     double yaw_diff = angles::shortest_angular_distance(tracked_armors[0].orientation_yaw,tracked_armors[1].orientation_yaw);
    //     // 定义角度差的合理范围
    //     const double ANGLE_THRESHOLD = M_PI_2;
    //     const double TOLERANCE = 0.2;
    //     if (yaw_diff > ANGLE_THRESHOLD - TOLERANCE && yaw_diff < ANGLE_THRESHOLD + TOLERANCE) { // 两个装甲板的绝对角度差不能偏差太大
    //         double x1, y1, z1, yaw1, x2, y2, z2, yaw2;
    //         double init_r1, init_r2, xc, yc;
    //         if (index == 0) {
    //             x1 = tracked_armors[0].x; y1 = tracked_armors[0].y; z1 = tracked_armors[0].z; yaw1 = tracked_armors[0].orientation_yaw;
    //             x2 = tracked_armors[1].x; y2 = tracked_armors[1].y; z2 = tracked_armors[1].z; yaw2 = tracked_armors[1].orientation_yaw;
    //         } else {
    //             x1 = tracked_armors[1].x; y1 = tracked_armors[1].y; z1 = tracked_armors[1].z; yaw1 = tracked_armors[1].orientation_yaw;
    //             x2 = tracked_armors[0].x; y2 = tracked_armors[0].y; z2 = tracked_armors[0].z; yaw2 = tracked_armors[0].orientation_yaw;
    //         }
    //         init_r1 = ((y2 - y1)*cos(yaw2) - (x2 - x1)*sin(yaw2))/sin(yaw1-yaw2);
    //         init_r2 = ((y2 - y1)*cos(yaw1) - (x2 - x1)*sin(yaw1))/cos(yaw1-yaw2);
    //         xc = x1 + init_r1 * cos(yaw1);
    //         yc = y1 + init_r1 * sin(yaw1);
    //         x << xc, 0, yc, 0, z1, z2, yaw1, 0, init_r1, init_r2;
    //     }else{
    //         x << tracked_armors[index].x + init_r * cos(orientation_yaw), 0,
    //             tracked_armors[index].y + init_r * sin(orientation_yaw), 0,
    //             tracked_armors[index].z, tracked_armors[index].z,
    //             orientation_yaw, 0, init_r, init_r;
    //     }
    // }
    enemy_model.ekf.init(x);
    enemy_model.tracker_state = DETECTING;
    // RCLCPP_INFO(rclcpp::get_logger("armor_tracker"), "初始化敌人跟踪器！");

}


void Tracker::updateEnemyModel(){
    Eigen::VectorXd x(10);
    x = enemy_model.ekf.predict(ekfPredict(dt)).x_pri;
    enemy_model.pri_estimation = x;
    // std::cout<<x<<std::endl;

    if(match_state == MATCH_SINGLE || match_state == MATCH_DOUBLE){
        auto calculate_yaw_diff = [&](int idx, double observed_yaw) {
            const double predicted_yaw = x[6] + idx * M_PI_2;
            return std::abs(angles::shortest_angular_distance(observed_yaw, predicted_yaw));
        };


        std::vector<int> index;
        if (match_state == MATCH_SINGLE) {
            // RCLCPP_INFO(rclcpp::get_logger("armor_tracker"), "识别到一个装甲板！");
            const double obs_yaw = tracked_armors[0].orientation_yaw;
            // const Eigen::Vector3d pose = {
            //     tracked_armors[0].x,
            //     tracked_armors[0].y,
            //     tracked_armors[0].z
            // };
            int best_idx = 0;
            double min_diff = DBL_MAX;
            for (int i = 0; i < 4; ++i) {
                // double orientation_yaw = x[6] + i * M_PI_2;
                // double xa = x[0] - cos(orientation_yaw) * x[8 + i % 2];
                // double ya = x[2] - sin(orientation_yaw) * x[8 + i % 2];
                // double za = i % 2 == 0 ? x[4] : x[5];
                // TODO：优化匹配方式
                // double position_diff = (pose - Eigen::Vector3d(xa, ya, za)).norm();
                double yaw_diff = calculate_yaw_diff(i, obs_yaw);
                // double diff = 0.7*position_diff/std::max(Eigen::Vector3d(xa, ya, za).norm(),1e-6)+0.3*yaw_diff/std::max(abs(orientation_yaw),1e-6); //TODO：优化匹配方式
                double diff = yaw_diff;
                if (diff < min_diff) {
                    min_diff = diff;
                    best_idx = i;
                }
            }
            index.push_back(best_idx);
        }else if (match_state == MATCH_DOUBLE) {
            // RCLCPP_INFO(rclcpp::get_logger("armor_tracker"), "识别到两个装甲板！");
            // 仅允许相邻装甲板对
            constexpr std::array<std::pair<int, int>, 4> adjacent_pairs = {
                {{0,1}, {1,2}, {2,3}, {3,0}}};
            
            // 获取观测角度
            const double yaw1 = tracked_armors[0].orientation_yaw;
            const double yaw2 = tracked_armors[1].orientation_yaw;

            // 寻找最佳相邻对
            auto best_pair = adjacent_pairs[0];
            double min_total_diff = std::numeric_limits<double>::max();
            //TODO:优化匹配方式
            for (const auto& pair : adjacent_pairs) {
                // 考虑两种顺序匹配可能性
                // TODO：加一个旋转方向判断会不会更好？
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
        Eigen::VectorXd z_pyd(index.size() * 4);
        for(size_t i = 0; i < index.size(); ++i){
            // i = 0或1
            z_pyd[i * 4] = tracked_armors[i].yaw;
            z_pyd[i * 4 + 1] = tracked_armors[i].pitch;
            z_pyd[i * 4 + 2] = tracked_armors[i].distance;
            z_pyd[i * 4 + 3] = tracked_armors[i].orientation_yaw;
        }
        // std::cout<<"z_pyd"<<std::endl;
        // std::cout<<z_pyd<<std::endl;
        //根据匹配到的装甲板的数量选择不同的观测方程
        switch (match_state)
        {
        case MATCH_DOUBLE:
            enemy_model.ekf.update(ekfMeasureDouble(index[0], index[1]),
                                    ekfPredict(dt),
                                    z_pyd,
                                    enemy_model.update_Q(dt),
                                    enemy_model.update_R(z_pyd));
            break;
        
        default:
            enemy_model.ekf.update(ekfMeasureSingle(index[0]),
                                    ekfPredict(dt),
                                    z_pyd,
                                    enemy_model.update_Q(dt),
                                    enemy_model.update_R(z_pyd));
            break;
        }
        x = enemy_model.ekf.getState();
    }
    preventRadiusSpread(enemy_model.ekf, x, 8);
    preventRadiusSpread(enemy_model.ekf, x, 9);
    enemy_model.post_estimation = x;

    // 更新跟踪状态
    if(match_state == MATCH_SINGLE || match_state == MATCH_DOUBLE){
        updateTrackerState(enemy_model, true);
        // std::cout<<"更新跟踪状态: "<<enemy_model.tracker_state<<std::endl;
    }else{
        updateTrackerState(enemy_model, false);
    }
    // std::cout<<"更新敌人跟踪状态"<<std::endl;
    double dz = enemy_model.post_estimation[5] - enemy_model.post_estimation[4];
    if(dz * last_dz < 0){
        RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "装甲板误匹配，当前匹配装甲板个数为%d",match_state);// TODO:优化判断误匹配的方式
    }
    last_dz = dz;
}


double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q){
    // Get armor yaw
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    // Make yaw change continuous (-pi~pi to -inf~inf)
    // yaw = last_yaw + angles::shortest_angular_distance(last_yaw, yaw);
    return yaw;
}

// 防止两个半径发散,将两个半径限制在合理范围内
template <typename T>
void Tracker::preventRadiusSpread(T& model,Eigen::VectorXd& x,const double& idx){
    if(x[idx] < 0.15 || x[idx] > 0.45){
        std::clamp(x[idx],0.15,0.45);
        model.setState(x);
    }
}

template <typename T>
void Tracker::updateTrackerState(T& model,bool matched){
    if(model.tracker_state == DETECTING){
        if(matched){
            model.detect_count++;
            if(model.detect_count > model.tracking_thres){
                model.tracker_state = TRACKING;
                model.detect_count = 0;
            }
        }else{
            model.detect_count = 0;
            model.tracker_state = LOST;

        }
    }else if(model.tracker_state == TRACKING){
        if(!matched){
            model.lost_count++;
            model.tracker_state = TEMP_LOST;
        }
    }else if(model.tracker_state == TEMP_LOST){
        if(!matched){
            model.lost_count++;
            if(model.lost_count > model.lost_thres){
                model.tracker_state = LOST;
                model.lost_count = 0;
            }
        }else{
            model.tracker_state = TRACKING;
            model.lost_count = 0;
        }
    }else if(model.tracker_state == LOST && matched){
        model.tracker_state = DETECTING;
    }
}


} // namespace rm_auto_aim
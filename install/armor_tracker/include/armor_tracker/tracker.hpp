#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

#include <string>
// ROS
#include "rclcpp/rclcpp.hpp"
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "armor_tracker/common.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "armor_tracker/motion/armor_model.hpp"
#include "armor_tracker/motion/enemy_model.hpp"

namespace rm_auto_aim
{

// using namespace rm_auto_aim::ArmorsNum;
class Tracker
{
public:
    using Armors = auto_aim_interfaces::msg::Armors;
    using Armor = auto_aim_interfaces::msg::Armor;
    Tracker(){
        this->enemy_model = EnemyModel();
        this->armor_model = ArmorModel();
    };

    void initArmorModel();
    void initEnemyModel();

    void updateArmorModel();
    void updateEnemyModel();

    bool findTrackerArmor(const auto_aim_interfaces::msg::Armors::SharedPtr & armors_msg);
    bool updateTrackerArmor(const auto_aim_interfaces::msg::Armors::SharedPtr & armors_msg);


    double dt;
    EnemyModel enemy_model; //TODO：是否需要用到智能指针？
    ArmorModel armor_model;
    MatchState match_state;
    std::string trackered_enemy_id; // 跟踪的装甲板id
    ArmorsNum enemy_armor_num; // 跟踪目标的装甲板数量
    std::vector<TrackedArmor> tracked_armors; // 追踪到的装甲板
private:
    double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

    template <typename T>
    void preventRadiusSpread(T& model,Eigen::VectorXd& x,const double& idx);

    template <typename T>
    void updateTrackerState(T& model,bool matched);


};

} // namespace rm_auto_aim
#endif // ARMOR_PROCESSOR__TRACKER_HPP_
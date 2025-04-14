#ifndef ARMOR_PROCESSOR__COMMON_HPP_
#define ARMOR_PROCESSOR__COMMON_HPP_

namespace rm_auto_aim{

enum TrackerState
{
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
};

enum ArmorsNum {  OUTPOST_3 = 3 ,NORMAL_4 = 4};

enum MatchState
{
    MATCH_NONE,
    MATCH_SINGLE,
    MATCH_DOUBLE
};

struct TrackedArmor
{
    double x, y, z, orientation_yaw;//xyz坐标系
    double yaw, pitch,distance;//陀螺仪坐标系
};


}

#endif // ARMOR_PROCESSOR__COMMON_HPP_
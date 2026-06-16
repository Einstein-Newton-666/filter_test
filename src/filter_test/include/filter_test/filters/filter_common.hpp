#pragma once

#include <cmath>

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


inline double normalize_angle(const double& angle)
{
    const double result = fmod(angle + M_PI, 2.0*M_PI);
    if(result <= 0.0) return result + M_PI;
    return result - M_PI;
}


inline double shortest_angular_distance(const double& from,const double& to)
{
    return normalize_angle(to-from);
}

// 限制到 -pi ~ pi
inline double reduced_angle(const double& x) {
    return std::atan2(std::sin(x), std::cos(x));
}

// 实数求余，符号为除数符号，以表示 0 ～ range 区间
inline double reduced(const double& x, const double& range) {
    double times = range / (2. * M_PI);
    return times * (reduced_angle(x / times - M_PI) + M_PI);
}

inline double get_closest(const double& cur, const double& tar, const double& period) {
    double reduced_value = reduced(cur, period);
    double possibles[3] = { reduced_value - period, reduced_value, reduced_value + period };
    double closest = possibles[0];
    for (double possible: possibles) {
        if (std::fabs(tar - possible) < std::fabs(tar - closest)) {
            closest = possible;
        }
    }
    return closest;
}

// 获取 tar 最近的 cur。获取结果可能不在 -pi 到 pi 之间
inline double get_closest_angle(const double& cur, const double& tar) {
    const double delta = reduced_angle(cur - tar);
    return tar + delta; // tar + cur - tar
}

template<typename T>
inline void ceres_xyz_to_ypd(const T& xyz, T& ypd) {
    ypd[0] = ceres::atan2(xyz[1], xyz[0]); // yaw
    ypd[1] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1])); // pitch
    ypd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]); // distance
};
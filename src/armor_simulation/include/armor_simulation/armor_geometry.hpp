#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace armor_sim {

// RoboMaster标准装甲板尺寸（单位：米）
constexpr double SMALL_ARMOR_WIDTH = 0.135;   // 135mm
constexpr double SMALL_ARMOR_HEIGHT = 0.125;  // 125mm
constexpr double LARGE_ARMOR_WIDTH = 0.230;   // 230mm
constexpr double LARGE_ARMOR_HEIGHT = 0.125;  // 125mm

/**
 * 计算装甲板4个角点（odom系）
 *
 * 装甲板坐标系约定：
 * - X: 装甲板宽度方向（从左到右）
 * - Y: 装甲板法线方向（指向外）
 * - Z: 装甲板高度方向（从下到上）
 *
 * 返回：[左下, 左上, 右上, 右下]
 */
inline std::array<Eigen::Vector3d, 4> computeArmorCorners(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    double width, double height) {

    // 装甲板局部坐标系下的4个角点
    double hw = width / 2.0;
    double hh = height / 2.0;

    std::array<Eigen::Vector3d, 4> corners_local = {
        Eigen::Vector3d(-hw, 0, -hh),  // 左下
        Eigen::Vector3d(-hw, 0, hh),   // 左上
        Eigen::Vector3d(hw, 0, hh),    // 右上
        Eigen::Vector3d(hw, 0, -hh)    // 右下
    };

    // 变换到odom坐标系
    std::array<Eigen::Vector3d, 4> corners_odom;
    for (int i = 0; i < 4; i++) {
        corners_odom[i] = orientation * corners_local[i] + position;
    }

    return corners_odom;
}

/**
 * 从yaw角和装甲板索引计算装甲板位姿
 *
 * @param center_x 机器人中心x
 * @param center_y 机器人中心y
 * @param center_z 机器人中心z（基础高度）
 * @param yaw 机器人yaw角
 * @param radius 装甲板到中心的半径
 * @param dz 高度差（奇数装甲板的额外高度）
 * @param armor_index 装甲板索引（0-3）
 * @return {position, orientation}
 */
inline std::pair<Eigen::Vector3d, Eigen::Quaterniond> computeArmorPose(
    double center_x, double center_y, double center_z,
    double yaw, double radius, double dz, int armor_index) {

    // 装甲板相对于中心的角度
    double armor_yaw = yaw + armor_index * M_PI / 2.0;

    // 装甲板位置
    double armor_x = center_x - radius * std::cos(armor_yaw);
    double armor_y = center_y - radius * std::sin(armor_yaw);
    double armor_z = center_z + (armor_index % 2) * dz;

    Eigen::Vector3d position(armor_x, armor_y, armor_z);

    // 装甲板朝向（法线指向外）
    // 装甲板的roll固定为0，pitch固定为0.26rad（约15度），yaw为armor_yaw
    double pitch = 0.26;  // 装甲板倾斜角
    Eigen::Quaterniond orientation =
        Eigen::AngleAxisd(armor_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

    return {position, orientation};
}

/**
 * 计算装甲板的6个检测点（autoaim格式）
 *
 * 返回：[左灯条下, 左灯条上, 右灯条上, 右灯条下]
 */
inline std::array<Eigen::Vector3d, 4> computeDetectedPoints(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    double width, double height) {

    // 左右灯条的位置
    double light_offset = width * 0.4;  // 灯条距离装甲板边缘的比例

    std::array<Eigen::Vector3d, 4> points_local = {
        Eigen::Vector3d(-light_offset, 0, -height / 2.0),  // 左灯条下
        Eigen::Vector3d(-light_offset, 0, height / 2.0),   // 左灯条上
        Eigen::Vector3d(light_offset, 0, height / 2.0),    // 右灯条上
        Eigen::Vector3d(light_offset, 0, -height / 2.0)    // 右灯条下
    };

    // 变换到odom坐标系
    std::array<Eigen::Vector3d, 4> points_odom;
    for (int i = 0; i < 4; i++) {
        points_odom[i] = orientation * points_local[i] + position;
    }

    return points_odom;
}

}  // namespace armor_sim

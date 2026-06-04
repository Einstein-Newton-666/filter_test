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
// 检测器提取的灯条角点尺寸 (对齐 jlu_vision_26 ArmorPoints.hpp LIGHTBAR_LENGTH=56mm)
constexpr double LIGHTBAR_LENGTH = 0.056;

/**
 * 计算装甲板4个角点（odom系）
 *
 * 装甲板坐标系约定 (autoaim / jlu 对齐)：
 * - X: 装甲板法线方向（指向外）
 * - Y: 装甲板宽度方向（左/右）
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

    // autoaim 约定: X=前(法向), Y=左(宽度), Z=上(高度)
    std::array<Eigen::Vector3d, 4> corners_local = {
        Eigen::Vector3d(0, -hw, -hh),  // 左下: y=-w/2, z=-h/2
        Eigen::Vector3d(0, -hw,  hh),  // 左上: y=-w/2, z=+h/2
        Eigen::Vector3d(0,  hw,  hh),  // 右上: y=+w/2, z=+h/2
        Eigen::Vector3d(0,  hw, -hh)   // 右下: y=+w/2, z=-h/2
    };

    // 变换到odom坐标系
    std::array<Eigen::Vector3d, 4> corners_odom;
    for (int i = 0; i < 4; i++) {
        corners_odom[i] = orientation * corners_local[i] + position;
    }

    return corners_odom;
}

}  // namespace armor_sim

#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace armor_sim {

/**
 * 相机内参
 */
struct CameraIntrinsics {
    double fx = 1500.0;  // 焦距x
    double fy = 1500.0;  // 焦距y
    double cx = 640.0;   // 光心x
    double cy = 360.0;   // 光心y
    double k1 = -0.1;    // 径向畸变系数1
    double k2 = 0.01;    // 径向畸变系数2
    double p1 = 0.0;     // 切向畸变系数1
    double p2 = 0.0;     // 切向畸变系数2
    int image_width = 1280;
    int image_height = 720;
};

/**
 * 相机外参
 */
struct CameraExtrinsics {
    Eigen::Matrix3d R_camera_odom = Eigen::Matrix3d::Identity();  // odom到camera的旋转
    Eigen::Vector3d t_camera_odom = Eigen::Vector3d::Zero();      // odom到camera的平移
};

/**
 * 相机模型
 *
 * 坐标系约定：
 * - Camera frame: X-right, Y-down, Z-forward（标准计算机视觉约定）
 * - Odom frame: X-forward, Y-left, Z-up（标准机器人约定）
 */
class CameraModel {
public:
    CameraModel() = default;
    CameraModel(const CameraIntrinsics& intr, const CameraExtrinsics& extr);

    /**
     * 3D点投影到2D像素坐标（odom系输入）
     * @param point_3d_odom 3D点在odom坐标系下的坐标
     * @return 像素坐标 (u, v)，如果点在相机后方返回 (-1, -1)
     */
    Eigen::Vector2d project3Dto2D(const Eigen::Vector3d& point_3d_odom) const;

    /**
     * 检查像素是否在图像内
     */
    bool isInImage(const Eigen::Vector2d& pixel, double margin = 0.0) const;

    /**
     * 计算装甲板像素面积（鞋带公式）
     */
    double computeArea(const std::array<Eigen::Vector3d, 4>& corners_3d_odom) const;

    /**
     * 计算装甲板中心像素坐标
     */
    Eigen::Vector2d computeCenter(const std::array<Eigen::Vector3d, 4>& corners_3d_odom) const;

    /**
     * 计算到图像中心距离
     */
    double distanceToImageCenter(const Eigen::Vector2d& pixel) const;

    /**
     * 获取相机外参旋转矩阵 R_camera_odom
     */
    Eigen::Matrix3d getRotation() const {
        return T_camera_odom_.block<3, 3>(0, 0);
    }

    /**
     * 设置相机外参 (用于云台动态跟踪)
     */
    void setExtrinsics(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
        T_camera_odom_.block<3, 3>(0, 0) = R;
        T_camera_odom_.block<3, 1>(0, 3) = t;
    }

    /**
     * PnP 位姿估计
     *
     * 从带噪像素角点求解 3D 位姿，保证 pixel ↔ pose 一致性。
     * 噪声从像素自然传播到位姿，无需独立参数。
     *
     * 投影模型:
     *   s [u, v, 1]^T = K [R | t] X
     * 其中 X 是装甲板局部 3D 点, [u, v] 是像素角点.
     *
     * 注意: 装甲板 4 点共面, 平面 PnP 天然存在两个相近姿态候选。
     * IPPE (Infinitesimal Plane-based Pose Estimation) 会针对平面点给出
     * 重投影误差排序后的解；OpenCV solvePnP(IPPE) 返回其中一个,
     * solvePnPGeneric(IPPE) 可返回两个候选。噪声较大时两个候选误差接近,
     * 仍需要上层用 yaw 先验/连续性处理双解歧义。
     *
     * @param image_points  4个带噪像素角点
     * @param armor_width   装甲板宽度 (m)
     * @param armor_height  装甲板高度 (m)
     * @return odom 系下的位姿 {position, quaternion}
     */
    struct PnPResult {
        Eigen::Vector3d position_odom;
        Eigen::Quaterniond orientation_odom;
        bool success = false;
    };

    PnPResult estimatePose(
        const std::array<Eigen::Vector2d, 4>& image_points,
        double armor_width, double armor_height) const
    {
        // 装甲板 3D 模型点 (X=法向, Y=宽度, Z=高度, 对齐 autoaim)
        double hw = armor_width / 2.0;
        double hh = armor_height / 2.0;
        std::vector<cv::Point3d> obj_pts = {
            {0, -hw, -hh},  // 左下
            {0, -hw,  hh},  // 左上
            {0,  hw,  hh},  // 右上
            {0,  hw, -hh}   // 右下
        };

        // 图像点
        std::vector<cv::Point2d> img_pts;
        for (int i = 0; i < 4; i++) {
            img_pts.emplace_back(image_points[i].x(), image_points[i].y());
        }

        double image_area = 0.0;
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            image_area += image_points[i].x() * image_points[j].y();
            image_area -= image_points[j].x() * image_points[i].y();
        }
        image_area = std::abs(image_area) * 0.5;
        PnPResult result;
        if (image_area < 1.0) {
            result.position_odom.setZero();
            result.orientation_odom.setIdentity();
            result.success = false;
            return result;
        }

        // 内参矩阵
        cv::Mat K = (cv::Mat_<double>(3, 3) <<
            intr_.fx, 0, intr_.cx,
            0, intr_.fy, intr_.cy,
            0, 0, 1);

        // 畸变系数 [k1, k2, p1, p2, k3]
        cv::Mat dist = (cv::Mat_<double>(5, 1) <<
            intr_.k1, intr_.k2, intr_.p1, intr_.p2, 0);

        cv::Mat rvec, tvec;
        // IPPE: 专为共面点设计的解析平面 PnP. 它比 ITERATIVE 更适合装甲板平面,
        // 但不是“消除双解”：平面目标仍可能有两组姿态候选, 此处只取 OpenCV
        // 按重投影误差选择的一个解, 后续由 correctPlanarPnPAmbiguity() 用 yaw 先验修正.
        bool ok = cv::solvePnP(obj_pts, img_pts, K, dist, rvec, tvec,
                               false, cv::SOLVEPNP_IPPE);
        if (!ok || tvec.empty() || rvec.empty()) {
            result.position_odom.setZero();
            result.orientation_odom.setIdentity();
            result.success = false;
            return result;
        }

        // 相机系位置
        Eigen::Vector3d pos_cam(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        if (!pos_cam.allFinite()) {
            result.position_odom.setZero();
            result.orientation_odom.setIdentity();
            result.success = false;
            return result;
        }

        // 相机系朝向 (Rodrigues → rotation matrix → quaternion)
        cv::Mat rot_mat;
        cv::Rodrigues(rvec, rot_mat);
        Eigen::Matrix3d R_cam;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R_cam(i, j) = rot_mat.at<double>(i, j);
        Eigen::Quaterniond orient_cam(R_cam);

        // 转到 odom 系: p_odom = Rᵀ · (p_cam - t),  R_odom = Rᵀ · R_cam
        Eigen::Matrix3d R = getRotation();
        Eigen::Vector3d t(T_camera_odom_(0, 3), T_camera_odom_(1, 3), T_camera_odom_(2, 3));

        result.position_odom = R.transpose() * (pos_cam - t);
        result.orientation_odom = Eigen::Quaterniond(R.transpose() * R_cam);
        result.success = true;
        return result;
    }

    /**
     * 从欧拉角构建外参
     */
    static CameraExtrinsics fromEulerAngles(double roll, double pitch, double yaw,
                                             double tx, double ty, double tz);

private:
    CameraIntrinsics intr_;
    Eigen::Matrix4d T_camera_odom_ = Eigen::Matrix4d::Identity();

    /**
     * 应用Brown-Conrady畸变模型
     */
    Eigen::Vector2d distort(const Eigen::Vector2d& normalized) const;
};

// 实现

inline CameraModel::CameraModel(const CameraIntrinsics& intr, const CameraExtrinsics& extr)
    : intr_(intr) {
    T_camera_odom_.block<3, 3>(0, 0) = extr.R_camera_odom;
    T_camera_odom_.block<3, 1>(0, 3) = extr.t_camera_odom;
}

inline Eigen::Vector2d CameraModel::project3Dto2D(const Eigen::Vector3d& point_3d_odom) const {
    // 1. 从odom坐标系变换到相机坐标系
    Eigen::Vector4d p_odom(point_3d_odom.x(), point_3d_odom.y(), point_3d_odom.z(), 1.0);
    Eigen::Vector4d p_cam = T_camera_odom_ * p_odom;

    // 2. 检查点是否在相机前方
    if (p_cam.z() <= 0.01) {
        return Eigen::Vector2d(-1, -1);
    }

    // 3. 归一化
    double x_n = p_cam.x() / p_cam.z();
    double y_n = p_cam.y() / p_cam.z();

    // 4. 应用畸变
    Eigen::Vector2d distorted = distort(Eigen::Vector2d(x_n, y_n));

    // 5. 转换到像素坐标
    double u = intr_.fx * distorted.x() + intr_.cx;
    double v = intr_.fy * distorted.y() + intr_.cy;

    return Eigen::Vector2d(u, v);
}

inline bool CameraModel::isInImage(const Eigen::Vector2d& pixel, double margin) const {
    return pixel.x() >= margin && pixel.x() < intr_.image_width - margin &&
           pixel.y() >= margin && pixel.y() < intr_.image_height - margin;
}

inline double CameraModel::computeArea(const std::array<Eigen::Vector3d, 4>& corners_3d_odom) const {
    // 投影4个角点到2D
    std::array<Eigen::Vector2d, 4> pixels;
    for (int i = 0; i < 4; i++) {
        pixels[i] = project3Dto2D(corners_3d_odom[i]);
    }

    // 鞋带公式计算面积
    double area = 0.0;
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        area += pixels[i].x() * pixels[j].y();
        area -= pixels[j].x() * pixels[i].y();
    }
    return std::abs(area) / 2.0;
}

inline Eigen::Vector2d CameraModel::computeCenter(const std::array<Eigen::Vector3d, 4>& corners_3d_odom) const {
    Eigen::Vector2d center(0, 0);
    for (int i = 0; i < 4; i++) {
        Eigen::Vector2d pixel = project3Dto2D(corners_3d_odom[i]);
        center += pixel;
    }
    return center / 4.0;
}

inline double CameraModel::distanceToImageCenter(const Eigen::Vector2d& pixel) const {
    double dx = pixel.x() - intr_.cx;
    double dy = pixel.y() - intr_.cy;
    return std::sqrt(dx * dx + dy * dy);
}

inline CameraExtrinsics CameraModel::fromEulerAngles(double roll, double pitch, double yaw,
                                                       double tx, double ty, double tz) {
    CameraExtrinsics extr;

    // 构建旋转矩阵（ZYX欧拉角）
    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

    extr.R_camera_odom = (yaw_angle * pitch_angle * roll_angle).matrix();
    extr.t_camera_odom = Eigen::Vector3d(tx, ty, tz);

    return extr;
}

inline Eigen::Vector2d CameraModel::distort(const Eigen::Vector2d& normalized) const {
    double x = normalized.x();
    double y = normalized.y();
    double r2 = x * x + y * y;
    double r4 = r2 * r2;

    // Brown-Conrady畸变模型
    double radial = 1.0 + intr_.k1 * r2 + intr_.k2 * r4;
    double x_d = x * radial + 2.0 * intr_.p1 * x * y + intr_.p2 * (r2 + 2.0 * x * x);
    double y_d = y * radial + intr_.p1 * (r2 + 2.0 * y * y) + 2.0 * intr_.p2 * x * y;

    return Eigen::Vector2d(x_d, y_d);
}

}  // namespace armor_sim

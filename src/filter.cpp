#include "filter_test/filter.hpp"

template<typename T>
void ceres_xyz_to_ypd(const T xyz[3], T ypd[3]) {
    ypd[0] = ceres::atan2(xyz[1], xyz[0]); // yaw
    ypd[1] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1])); // pitch
    ypd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]); // distance
};

// EKF
// xa = x_armor, xc = x_robot_center
// state: xc, v_xc, yc, v_yc, za1, za2, orient, v_yaw, r1, r2
// index: 0,  1,    2,  3,    4,  5,    6,      7,     8,   9,
// - yaw 需要在切换时 set?
// - 对于只有单独半径的装甲板， 不更新多余的 r 和 z
// measurement: xa, ya, za, yaw
// measurement2: yaw（相机）, pitch, distance, yaw(orient)
// f - Process function

struct Predict {
public:
    explicit Predict(const double& delta_t): delta_t(delta_t) {}
    template<typename T>
    void operator()(const T x_pre[10], T x_cur[10]) const {
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

private:
    double delta_t = 0.;
};


struct Measure {
    public:
        // z 用于接收. 由 x 推导到 z
        // z: 当前需求比较的装甲板，rotate: super_yaw 到需求装甲的 rotate
        // ekf 之与测量转观测量
        // 用装甲板进行更新J
        // N_Y 观测到的装甲板的个数 I 匹配到的装甲板的索引
        template<typename T, int I>
        void operator()(const T x[10], T z[4]) const {
            // x[6] 是 super 板的 yaw
            const T xyz_armor[3] = { 
                x[0] + ceres::cos(x[6] + M_PI_2 * I) * x[8 + I % 2],
                x[2] + ceres::sin(x[6] + M_PI_2 * I) * x[8 + I % 2],
                x[4 + I % 2] 
            };
            T ypd[3];
            ceres_xyz_to_ypd(xyz_armor, ypd);
            for (int i = 0; i < 3; i++) {
                z[i] = ypd[i];
            }
            // orien_yaw = orien_yaw
            z[3] = x[6] + M_PI_2 * I;
        }
        // N_Y 观测到的装甲板的个数 I J 匹配到的装甲板的索引
        template<typename T, int I, int J>
        void operator()(const T x[10], T z[8]) const {
            int idx = 0;
            for (const auto &i : {I, J})
            {
                // x[6] 是 super 板的 yaw
                T xyz_armor[3] = { 
                    x[0] + ceres::cos(x[6] + M_PI_2 * i) * x[8 + i % 2],
                    x[2] + ceres::sin(x[6] + M_PI_2 * i) * x[8 + i % 2],
                    x[4 + i % 2] 
                };
                T ypd[3];
                ceres_xyz_to_ypd(xyz_armor, ypd);
                for (int j = 0; j < 3; j++) {
                    z[idx + j] = ypd[j];
                }
                // orien_yaw = orien_yaw
                z[idx + 3] = x[6] + M_PI_2 * i;
                idx += 4;
            }
        }
    
    private:
};

ArmorFilter::ArmorFilter(/* args */)
{
}


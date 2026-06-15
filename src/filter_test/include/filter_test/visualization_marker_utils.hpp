#pragma once

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

namespace filter_test {

inline tf2::Quaternion observedArmorMarkerQuaternion(double yaw, bool is_outpost = false) {
    tf2::Quaternion q;
    const double pitch = is_outpost ? -0.26 : 15.0 * M_PI / 180.0;
    q.setRPY(0.0, pitch, yaw);
    return q;
}

}  // namespace filter_test

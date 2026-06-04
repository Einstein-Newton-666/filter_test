#pragma once

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

namespace filter_test {

inline tf2::Quaternion observedArmorMarkerQuaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 15.0 * M_PI / 180.0, yaw);
    return q;
}

}  // namespace filter_test

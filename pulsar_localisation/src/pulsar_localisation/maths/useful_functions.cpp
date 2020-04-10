#include "maths/useful_functions.hpp"

#include <cmath>

double quat_to_yaw(const geometry_msgs::Quaternion& q) {
    return 2*acos(q.w) * (q.z < 0 ? -1 : 1);
}

geometry_msgs::Quaternion yaw_to_quat(double yaw) {
    geometry_msgs::Quaternion q;
    q.w = cos(yaw/2);
    q.z = sin(yaw/2);
    return q;
}

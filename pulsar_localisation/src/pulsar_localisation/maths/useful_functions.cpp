#include "maths/useful_functions.hpp"

#include <cmath>
#include <random>

double quat_to_yaw(const geometry_msgs::Quaternion& q) {
    return 2*acos(q.w) * (q.z < 0 ? -1 : 1);
}

geometry_msgs::Quaternion yaw_to_quat(double yaw) {
    geometry_msgs::Quaternion q;
    q.w = cos(yaw/2);
    q.z = sin(yaw/2);
    return q;
}

double prob_normal_distribution(double a, double b2) {
    return (1.0/sqrt(2*M_PI*b2))*exp(-0.5 * a*a/b2);
}

double sample_normal_distribution(double b2) {
    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    std::normal_distribution<double> dist(0, sqrt(b2));
    return dist(gen);
}

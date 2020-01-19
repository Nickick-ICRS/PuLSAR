#include "odometry/odometry.hpp"

Odometry::Odometry(float cpr, float radius, float separation) 
    :cpr(cpr), wheel_separation(separation), 
     wheel_circumference(3.1415926*radius*2), theta_vel(0), forwards_vel(0),
     l_w_c(0), r_w_c(0)
{
    // ctor
}

Odometry::~Odometry() {
    // dtor
}

void Odometry::update(float dt) {
    float thl = l_w_c;
    float thr = r_w_c;
    l_w_c = 0;
    r_w_c = 0;

    thl *= 2*3.1415926/cpr;
    thr *= 2*3.1415926/cpr;

    forwards_vel =  (thr + thl) / (2*wheel_circumference);
    theta_vel = (thr - thl) / (wheel_separation*wheel_circumference);
}

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
    // This could be running with interrupts so quickly store the current 
    // pulse count and reset the pulse counts
    float thl = l_w_c;
    float thr = r_w_c;
    l_w_c = 0;
    r_w_c = 0;

    r_vel = thl * 2 * 3.1415926 / (cpr * dt);
    l_vel = thr * 2 * 3.1415926 / (cpr * dt);

    forwards_vel =  (r_vel + l_vel) / (2*wheel_circumference);
    theta_vel = (r_vel - l_vel) / (wheel_separation*wheel_circumference);
}

#include "odometry/odometry.hpp"

Odometry::Odometry(float cpr, float radius, float separation) 
    :cpr(cpr), wheel_separation(separation), 
     wheel_circumference(3.1415926*radius*2), l_w_c(0), r_w_c(0)
{
    for(int i = 0; i < COV_SAMPLES; i++) {
        theta_vel[i] = 0;
        forwards_vel[i] = 0;
    }
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

    r_vel = thr * 2 * 3.1415926 / (cpr * dt);
    l_vel = thl * 2 * 3.1415926 / (cpr * dt);

    for(int i = COV_SAMPLES-1; i != 0; i--) {
        forwards_vel[i] = forwards_vel[i-1];
        theta_vel[i] = theta_vel[i-1];
    }

    forwards_vel[0] =  wheel_circumference * (r_vel + l_vel) / 2.f;
    theta_vel[0] = wheel_circumference * (r_vel - l_vel) / wheel_separation;
}

void Odometry::calculate_wheel_speeds(
    float *left, float *right, float vx, float vth)
{
    *left = (vx - wheel_separation * vth / 2.f) / wheel_circumference;
    *right = (vx + wheel_separation * vth / 2.f) / wheel_circumference;
}

float Odometry::calculate_cov_forwards_vel() {
    float mean = 0;
    for(float val : forwards_vel) 
        mean += val;
    mean /= COV_SAMPLES;

    float var = 0;
    for(float val : forwards_vel)
        var += (val - mean) * (val - mean);
    var /= COV_SAMPLES - 1;

    return sqrt(var);
}

float Odometry::calculate_cov_theta_vel() {
    float mean = 0;
    for(float val : theta_vel) 
        mean += val;
    mean /= COV_SAMPLES;

    float var = 0;
    for(float val : theta_vel)
        var += (val - mean) * (val - mean);
    var /= COV_SAMPLES - 1;

    return sqrt(var);
}

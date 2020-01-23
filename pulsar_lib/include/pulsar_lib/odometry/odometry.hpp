#ifndef __ODOMETRY_HPP__
#define __ODOMETRY_HPP__

#include <cmath>

// Number of samples used for determining the covariance
#define COV_SAMPLES 5

class Odometry {
public:
    // cpr: encoder counts per full rotation
    // radius: radius of the wheels
    // separation: distance between the centers of the wheels
    Odometry(float cpr, float radius, float separation);
    virtual ~Odometry();

    // Update the odometry calculation
    // dt: amount of time since last update
    void update(float dt);
    // Update the count of a given wheel
    // dir: true if the wheel moved forwards by one increment, false if it
    //      move backwards
    void update_left_wheel_count(bool dir) { dir ? l_w_c++ : l_w_c--; };
    void update_right_wheel_count(bool dir) { dir ? r_w_c++ : r_w_c--; };

    // Get the current forward and angular velocities of the robot
    const float& get_forwards_vel() { return forwards_vel[0]; };
    const float& get_theta_vel() { return theta_vel[0]; };
    const float& get_left_vel() { return l_vel; };
    const float& get_right_vel() { return r_vel; };

    // Calculate the wheel velocities required to move at a given velocity
    // left: [output] left wheel speed
    // right: [output] right wheel speed
    // vx: target forwards velocity
    // vth: target angular velocity
    void calculate_wheel_speeds(
        float *left, float *right, float vx, float vth);

    // Calculate uncertainty in recent measurements
    float calculate_cov_forwards_vel();
    float calculate_cov_theta_vel();
private:
    // theta and forwards (x) velocities
    float theta_vel[COV_SAMPLES];
    float forwards_vel[COV_SAMPLES];

    // Wheel velocities
    float l_vel;
    float r_vel;

    // left and right wheel counts - these probably will be modified during
    // interrupts, hence the volatility
    volatile int l_w_c;
    volatile int r_w_c;

    // constants
    const int cpr;
    const int wheel_circumference;
    const int wheel_separation;
};

#endif // __ODOMETRY_HPP__

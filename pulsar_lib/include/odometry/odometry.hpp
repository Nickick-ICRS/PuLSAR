#ifndef __ODOMETRY_HPP__
#define __ODOMETRY_HPP__

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
    const float& get_forwards_vel() { return forwards_vel; };
    const float& get_theta_vel() { return theta_vel; };
private:
    // theta and forwards (x) velocities
    float theta_vel;
    float forwards_vel;

    // left and right wheel counts
    int l_w_c;
    int r_w_c;

    // constants
    const int cpr;
    const int wheel_circumference;
    const int wheel_separation;
};

#endif // __ODOMETRY_HPP__

#include "motor_control/motor_control.hpp"

MotorControl::MotorControl(float kp, float ki) :kp(kp), ki(ki) {
    // ctor
}

MotorControl::~MotorControl() {
    // dtor
}

void MotorControl::update(float dt) {
    float err = target_vel - current_vel;
    // Prevent excessive time-jumps from screwing over i
    if(dt > 0.1)
        dt = 0.1;
    int_err += dt * err;
    output_power = kp * err + ki * int_err;
    if(output_power > 1) output_power = 1;
    if(output_power < -1) output_power = -1;
}

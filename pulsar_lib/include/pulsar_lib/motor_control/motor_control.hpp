#ifndef __MOTOR_CONTROL_HPP__
#define __MOTOR_CONTROL_HPP__

class MotorControl {
public:
    // This is a PI controller for a motor
    MotorControl(float kp, float ki);
    virtual ~MotorControl();

    // Update the controller
    void update(float dt);
    // Get the current % power that should be applied to the motor
    const float& get_power_percentage() { return output_power; };

    void set_target_vel(float target) { target_vel = target; };
    void set_current_vel(float current) { current_vel = current; };
private:
    const float kp;
    const float ki;

    float current_vel;
    float target_vel;
    float output_power;

    float int_err;
};

#endif // __MOTOR_CONTROL_HPP__

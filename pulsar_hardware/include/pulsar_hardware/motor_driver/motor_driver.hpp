#ifndef __MOTOR_DRIVER_HPP__
#define __MOTOR_DRIVER_HPP__

#include "driver/mcpwm.h"
#include "driver/gpio.h"

class MotorDriver {
public:
    MotorDriver(
        mcpwm_unit_t pwm_unit, mcpwm_io_signals_t io_signal, gpio_num_t dir,
        gpio_num_t sig, mcpwm_timer_t timer, float freq);
    virtual ~MotorDriver();
    // pc - percent duty cycle from -1 (100% reverse) to 1 (100% forwards)
    void update_duty_cycle(float pc);

    void stop();
private:
    gpio_num_t dir_;
    gpio_num_t sig_;
    mcpwm_timer_t timer_;
    mcpwm_unit_t unit_;
};

#endif // __MOTOR_DRIVER_HPP__

#include "motor_driver/motor_driver.hpp"

MotorDriver::MotorDriver(
    mcpwm_unit_t pwm_unit, mcpwm_io_signals_t io_signal, gpio_num_t dir,
    gpio_num_t sig, mcpwm_timer_t timer, float freq)
    :unit_(pwm_unit), dir_(dir), sig_(sig), timer_(timer)
{
    // Initialise the PWM unit
    mcpwm_gpio_init(unit_, io_signal, sig_);

    // Set the config settings
    mcpwm_config_t config;
    config.frequency = freq;
    config.cmpr_a = 0;
    config.cmpr_b = 0;
    config.counter_mode = MCPWM_UP_COUNTER;
    config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(unit_, timer_, &config);

    // Initialise the gpio direction pin
    gpio_pad_select_gpio(dir_);
    gpio_set_direction(dir_, GPIO_MODE_OUTPUT);
}

MotorDriver::~MotorDriver() {
    // dtor
}

void MotorDriver::update_duty_cycle(float pc) {
    pc *= 100;
    if(pc >= 0) {
        mcpwm_set_duty(unit_, timer_, MCPWM_OPR_A, pc);
        gpio_set_level(dir_, 0);
    }
    else {
        mcpwm_set_duty(unit_, timer_, MCPWM_OPR_A, 100 + pc);
        gpio_set_level(dir_, 1);
    }
}

void MotorDriver::stop() {
    this->update_duty_cycle(0);
}

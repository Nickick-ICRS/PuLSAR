#ifndef __LED_HPP__
#define __LED_HPP__

#include "driver/gpio.h"

class Led {
public:
    Led(gpio_num_t gpio);
    virtual ~Led();

    // Toggle the status of the LED
    void toggle();

    // Assignment operator
    Led& operator=(bool value);
private:
    gpio_num_t gpio_;
    bool status_;

    // Comparison operators
    friend bool operator==(const Led& lhs, bool rhs);
    friend bool operator==(bool lhs, const Led& rhs);
};

bool operator==(const Led& lhs, bool rhs);
bool operator==(bool lhs, const Led& rhs);

#endif // __LED_HPP__

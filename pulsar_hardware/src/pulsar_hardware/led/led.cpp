#include "led/led.hpp"

Led::Led(gpio_num_t gpio) : gpio_(gpio), status_(false) {
    gpio_pad_select_gpio(gpio_);
    gpio_set_direction(gpio_, GPIO_MODE_OUTPUT);
}

Led::~Led() {
    // dtor
}

void Led::toggle() {
    status_ = !status_;
    gpio_set_level(gpio_, status_);
}

Led& Led::operator=(bool status) {
    status_ = status;
    gpio_set_level(gpio_, status_);
    return *this;
}

bool operator==(const Led& lhs, bool rhs) {
    return lhs.status_ == rhs;
}

bool operator==(bool lhs, const Led& rhs) {
    return lhs == rhs.status_;
}

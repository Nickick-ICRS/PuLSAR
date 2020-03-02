#include "led/led.hpp"

#include "driver/gpio.h"

#include <chrono>
#include <thread>

// This is necessary to allow the esp (C) library to link to my main
// function
extern "C" {
    void app_main();
}

void app_main()
{
    Led green(GPIO_NUM_16);
    Led blue_1(GPIO_NUM_19);
    Led blue_2(GPIO_NUM_33);
    while(1) {
        green = 0;
        blue_1 = 0;
        blue_2 = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        green = 1;
        blue_1 = 0;
        blue_2 = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        green = 0;
        blue_1 = 1;
        blue_2 = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        green = 1;
        blue_1 = 1;
        blue_2 = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        green = 0;
        blue_1 = 0;
        blue_2 = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        green = 1;
        blue_1 = 0;
        blue_2 = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

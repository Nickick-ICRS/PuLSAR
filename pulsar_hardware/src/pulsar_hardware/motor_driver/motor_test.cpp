#include "motor_driver/motor_driver.hpp"

#include <chrono>
#include <thread>

extern "C" {
    void app_main(void);
}

void app_main() {
    MotorDriver left(
        MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_27, GPIO_NUM_26, MCPWM_TIMER_0,
        10000);
    MotorDriver right(
        MCPWM_UNIT_1, MCPWM1A, GPIO_NUM_23, GPIO_NUM_25, MCPWM_TIMER_1,
        10000);
    float speed;
    while(true) {
        printf("Spinning left forwards...\n");
        for(speed = 0; speed < 1; speed += 0.01) {
            left.update_duty_cycle(speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        for(speed = 1; speed > 0; speed -= 0.01) {
            left.update_duty_cycle(speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        printf("Spinning left backwards...\n");
        for(speed = 0; speed > -1; speed -= 0.01) {
            left.update_duty_cycle(speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        for(speed = -1; speed < 0; speed += 0.01) {
            left.update_duty_cycle(speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        printf("Spinning right forwards...\n");
        for(speed = 0; speed < 1; speed += 0.01) {
            right.update_duty_cycle(speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        for(speed = 1; speed > 0; speed -= 0.01) {
            right.update_duty_cycle(speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        printf("Spinning right backwards...\n");
        for(speed = 0; speed > -1; speed -= 0.01) {
            right.update_duty_cycle(speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        for(speed = -1; speed < 0; speed += 0.01) {
            right.update_duty_cycle(speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

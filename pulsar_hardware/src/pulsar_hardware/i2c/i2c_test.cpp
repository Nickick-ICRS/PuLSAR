#include "led/led.hpp"
#include "i2c/i2c.hpp"

#include "driver/gpio.h"

#include <stdio.h>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>

// This is necessary to allow the esp (C) library to link to my main
// function
extern "C" {
    void app_main();
}

void app_main() {
    Led led(GPIO_NUM_33);
    I2C *i2c;
    // 100 kHz i2c
    try {
        i2c = new I2C(100000);
    }
    catch(int err) {
        std::stringstream ss;
        ss << err;
        std::string s;
        ss >> s;
        s = "Failed to initialise i2c class. Error code: '" + s + "'.\n";
        printf(s.c_str());
        return;
    }

    std::stringstream ss;
    while(true) {
        std::string addresses;
        try {
            addresses = i2c->scan();
        }
        catch(int err) {
            std::stringstream ss;
            ss << err;
            std::string s;
            ss >> s;
            s = "Failed to scan i2c bus. Error code: '" + s + "'.\n";
            printf(s.c_str());
            continue;
        }
        led.toggle();
        if(addresses.size()) {
            std::string msg = "Found the following addresses: ";
            for(uint32_t i = 0; i < addresses.size(); i++) {
                ss << std::hex;
                ss << std::setw(2) << std::setfill('0') 
                   << (int)(addresses[i]);
                msg += ss.str();
                msg += ", ";
                ss.clear();
            }
            msg = msg.substr(0, msg.size() - 2);
            msg += "\r\n";
            printf(msg.c_str(), msg.size());
        }
        else {
            printf("No i2c devices were found.\n");
        }
    }
}

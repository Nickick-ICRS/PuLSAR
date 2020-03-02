#ifndef __I2C_HPP__
#define __I2C_HPP__

#include "driver/gpio.h"
#include "driver/i2c.h"

#include <string>

// i2c pins
#define I2C_SDA_IO 22
#define I2C_SCL_IO 21

class I2C {
public:
    I2C(uint32_t baud_hz);
    virtual ~I2C();

    // Reads data from an address. Returns the bytes read as a string.
    std::string read(uint8_t address, uint16_t len);
    // Writes data to an address. Data is read in bytes by bytes.
    void write(uint8_t address, std::string data);

    // Scan the i2c network. Returns the addresses of anything found as a 
    // string.
    std::string scan();
private:
    i2c_port_t port;
};

#endif // __I2C_HPP__

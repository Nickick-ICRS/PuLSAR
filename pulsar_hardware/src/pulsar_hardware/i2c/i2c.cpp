#include "i2c/i2c.hpp"

I2C::I2C(uint32_t baud_hz) {
    // i2c protocol documentation here: https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/i2c.html
    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = I2C_SDA_IO;
    config.scl_io_num = I2C_SCL_IO;
    config.sda_pullup_en = GPIO_PULLUP_DISABLE; // pullups exist on the PCB
    config.scl_pullup_en = GPIO_PULLUP_DISABLE;
    config.master.clk_speed = baud_hz;
    i2c_param_config(0, &config);

    esp_err_t err = i2c_driver_install(0, config.mode, 0, 0, 0);
    if(err)
        throw(err);
}

I2C::~I2C() {
    // dtor
}

std::string I2C::read(uint8_t address, uint16_t len) {
    // i2c protocol documentation here: https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/i2c.html
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(
        cmd, (address << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
    uint8_t data[len];
    i2c_master_read(cmd, data, len-1, I2C_MASTER_ACK);
    i2c_master_read(cmd, data + len-1, 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(
        0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(err)
        throw(err);
    return std::string((char*)data, len);
}

void I2C::write(uint8_t address, std::string data) {
    // i2c protocol documentation here: https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/i2c.html
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(
        cmd, (address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write(
        cmd, (uint8_t*)data.c_str(), data.size(), I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(
        0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(err)
        throw(err);
}

std::string I2C::scan() {
    std::string addresses;
    for(uint8_t addr = 0; addr != 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(
            cmd, (addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(
            0, cmd, 50 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if(err == ESP_OK) {
            addresses += (char)addr;
        }
    }
    return addresses;
}

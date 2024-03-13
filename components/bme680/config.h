#pragma once
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define I2C_PORT 0
#define GLITCH_IGNORE_COUNT 7
#define I2C_DEVICE_ADDRESS 0b1110111
#define I2C_CLOCK_HZ (1 * 100)

static const i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .scl_io_num = 0,
    .sda_io_num = 0,
    .glitch_ignore_cnt = GLITCH_IGNORE_COUNT,
    .flags.enable_internal_pullup = true,
};

static const i2c_device_config_t i2c_device_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = I2C_DEVICE_ADDRESS,
    .scl_speed_hz = I2C_CLOCK_HZ,
};
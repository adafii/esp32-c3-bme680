#pragma once
#include "driver/i2c_master.h"

// #define OSRS_1 0b001

typedef enum oversampling_t : uint8_t {
    OS_1 = 0b001,
    OS_2 = 0b010,
    OS_4 = 0b011,
    OS_8 = 0b100,
    OS_16 = 0b101,
} oversampling_t;

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t device_handle;
} bme680_device_t;

esp_err_t init_bme680_device(bme680_device_t* bme680_device, gpio_num_t scl_gpio, gpio_num_t sda_gpio);

esp_err_t set_oversampling(bme680_device_t* bme680_device,
                           oversampling_t temperature,
                           oversampling_t pressure,
                           oversampling_t humidity);
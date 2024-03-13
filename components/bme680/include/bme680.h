#pragma once
#include "driver/i2c_master.h"

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t device_handle;
} bme680_device_t;

esp_err_t init_bme680_device(gpio_num_t scl_gpio, gpio_num_t sda_gpio, bme680_device_t* bme680_device);
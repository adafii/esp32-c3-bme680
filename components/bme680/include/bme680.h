#pragma once
#include "driver/i2c_master.h"

// Oversampling values
typedef enum : uint8_t {
    OS_1 = 0b001,
    OS_2 = 0b010,
    OS_4 = 0b011,
    OS_8 = 0b100,
    OS_16 = 0b101,
} oversampling_t;

// IIR filter coefficients
typedef enum : uint8_t {
    COEFF_0 = 000,
    COEFF_1 = 001,
    COEFF_3 = 010,
    COEFF_7 = 011,
    COEFF_15 = 100,
    COEFF_31 = 101,
    COEFF_63 = 110,
    COEFF_127 = 111,
} coefficient_t;

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t device_handle;
} bme680_device_t;

typedef struct {
    double temperature;
    double pressure;
    double humidity;
    double gas;
} measurement_t;

esp_err_t init_bme680_device(bme680_device_t* bme680_device, gpio_num_t scl_gpio, gpio_num_t sda_gpio);

esp_err_t set_oversampling(bme680_device_t* bme680_device,
                           oversampling_t temperature,
                           oversampling_t pressure,
                           oversampling_t humidity);

esp_err_t set_iir_filter(bme680_device_t* bme680_device, coefficient_t coefficient);

esp_err_t disable_gas(bme680_device_t* bme680_device);

esp_err_t measure(bme680_device_t* bme680_device, measurement_t* measurement);
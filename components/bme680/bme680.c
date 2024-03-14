#include "bme680.h"
#include "config.h"
#include "esp_check.h"

#define BUS_ERROR_MSG "Error when creating I2C bus"
#define PROBE_ERROR_MSG "BME680 was not found"
#define DEVICE_ERROR_MSG "Error when adding I2C device"
#define SENSOR_INIT_ERROR_MSG "Error when trying to init sensor"
#define TRANSMIT_ERROR_MSG "I2C transmit error"

static const char* TAG = "BME860";

esp_err_t init_bme680_device(bme680_device_t* bme680_device, gpio_num_t scl_gpio, gpio_num_t sda_gpio) {
    i2c_master_bus_config_t user_bus_config = i2c_bus_config;
    user_bus_config.scl_io_num = scl_gpio;
    user_bus_config.sda_io_num = sda_gpio;

    i2c_master_bus_handle_t* bus = &bme680_device->bus_handle;
    i2c_master_dev_handle_t* device = &bme680_device->device_handle;

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&user_bus_config, bus), TAG, BUS_ERROR_MSG);
    ESP_RETURN_ON_ERROR(i2c_master_probe(*bus, I2C_DEVICE_ADDRESS, -1), TAG, PROBE_ERROR_MSG);
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(*bus, &i2c_device_config, device), TAG, DEVICE_ERROR_MSG);

    ESP_RETURN_ON_ERROR(set_oversampling(bme680_device, DEFAULT_OS, DEFAULT_OS, DEFAULT_OS), TAG,
                        SENSOR_INIT_ERROR_MSG);

    // TODO: Set gas on by default
    ESP_RETURN_ON_ERROR(disable_gas(bme680_device), TAG, SENSOR_INIT_ERROR_MSG);

    return ESP_OK;
}

esp_err_t set_oversampling(bme680_device_t* bme680_device,
                           oversampling_t temperature,
                           oversampling_t pressure,
                           oversampling_t humidity) {
    i2c_master_dev_handle_t* device = &bme680_device->device_handle;
    uint8_t ctrl_hum = 0;
    uint8_t ctrl_meas = 0;

    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(*device, &REGISTER_CTRL_HUM, sizeof(REGISTER_CTRL_HUM), &ctrl_hum,
                                                    sizeof(ctrl_hum), I2C_MAX_WAIT),
                        TAG, TRANSMIT_ERROR_MSG);

    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(*device, &REGISTER_CTRL_MEAS, sizeof(REGISTER_CTRL_MEAS),
                                                    &ctrl_meas, sizeof(ctrl_meas), I2C_MAX_WAIT),
                        TAG, TRANSMIT_ERROR_MSG);

    ctrl_meas = (ctrl_meas & 0b00000011) | temperature << 5 | pressure << 2;
    ctrl_hum = (ctrl_hum & 0b11111000) | humidity;

    uint8_t write_buffer[4] = {REGISTER_CTRL_HUM, ctrl_hum, REGISTER_CTRL_MEAS, ctrl_meas};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(*device, write_buffer, sizeof(write_buffer), I2C_MAX_WAIT), TAG,
                        TRANSMIT_ERROR_MSG);

    return ESP_OK;
}

esp_err_t set_iir_filter(bme680_device_t* bme680_device, coefficient_t coefficient) {
    i2c_master_dev_handle_t* device = &bme680_device->device_handle;
    uint8_t config = 0;

    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(*device, &REGISTER_CONFIG, sizeof(REGISTER_CONFIG), &config,
                                                    sizeof(config), I2C_MAX_WAIT),
                        TAG, TRANSMIT_ERROR_MSG);

    config = (config & 0b11100011) | coefficient << 2;

    uint8_t write_buffer[2] = {REGISTER_CONFIG, config};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(*device, write_buffer, sizeof(write_buffer), I2C_MAX_WAIT), TAG,
                        TRANSMIT_ERROR_MSG);

    return ESP_OK;
}

esp_err_t disable_gas(bme680_device_t* bme680_device) {
    i2c_master_dev_handle_t* device = &bme680_device->device_handle;
    uint8_t ctrl_gas_0 = 0;
    uint8_t ctrl_gas_1 = 0;

    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(*device, &REGISTER_CTRL_GAS_0, sizeof(REGISTER_CTRL_GAS_0),
                                                    &ctrl_gas_0, sizeof(ctrl_gas_0), I2C_MAX_WAIT),
                        TAG, TRANSMIT_ERROR_MSG);

    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(*device, &REGISTER_CTRL_GAS_1, sizeof(REGISTER_CTRL_GAS_1),
                                                    &ctrl_gas_1, sizeof(ctrl_gas_1), I2C_MAX_WAIT),
                        TAG, TRANSMIT_ERROR_MSG);

    ctrl_gas_0 |= 1 << 3;
    ctrl_gas_1 &= ~(1 << 4);

    uint8_t write_buffer[4] = {REGISTER_CTRL_GAS_0, ctrl_gas_0, REGISTER_CTRL_GAS_1, ctrl_gas_1};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(*device, write_buffer, sizeof(write_buffer), I2C_MAX_WAIT), TAG,
                        TRANSMIT_ERROR_MSG);

    return ESP_OK;
}
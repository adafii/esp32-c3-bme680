#include "bme680.h"
#include "config.h"
#include "esp_check.h"

#define BUS_ERROR_MSG "Error when creating I2C bus"
#define PROBE_ERROR_MSG "BME680 was not found"
#define DEVICE_ERROR_MSG "Error when adding I2C device"
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

    return ESP_OK;
}

esp_err_t set_oversampling(bme680_device_t* bme680_device,
                           oversampling_t temperature,
                           oversampling_t pressure,
                           oversampling_t humidity) {

    i2c_master_dev_handle_t* device = &bme680_device->device_handle;
    uint8_t ctrl_meas_address = REGISTER_CTRL_MEAS;
    uint8_t ctrl_hum_address = REGISTER_CTRL_HUM;
    uint8_t ctrl_meas = 0;
    uint8_t ctrl_hum = 0;

    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(*device, &ctrl_meas_address, sizeof(uint8_t), &ctrl_meas, sizeof(uint8_t), -1), TAG,
        TRANSMIT_ERROR_MSG);

    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(*device, &ctrl_hum_address, sizeof(uint8_t), &ctrl_hum, sizeof(uint8_t), -1), TAG,
        TRANSMIT_ERROR_MSG);

    ctrl_meas |= temperature << 5 | pressure << 2;
    ctrl_hum |= humidity;

    uint8_t write_buffer[4] = {REGISTER_CTRL_HUM, ctrl_hum, REGISTER_CTRL_MEAS, ctrl_meas};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(*device, write_buffer, sizeof(write_buffer), -1), TAG, TRANSMIT_ERROR_MSG);

    return ESP_OK;
}
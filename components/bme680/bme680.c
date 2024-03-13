#include "bme680.h"
#include "config.h"
#include "esp_check.h"

static const char* TAG = "BME860";

esp_err_t init_bme680_device(gpio_num_t scl_gpio, gpio_num_t sda_gpio, bme680_device_t* bme680_device) {
    i2c_master_bus_config_t user_bus_config = i2c_bus_config;
    user_bus_config.scl_io_num = scl_gpio;
    user_bus_config.sda_io_num = sda_gpio;

    i2c_master_bus_handle_t* bus = &bme680_device->bus_handle;
    i2c_master_dev_handle_t* device = &bme680_device->device_handle;

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&user_bus_config, bus), TAG, "Error when creating I2C bus");
    ESP_RETURN_ON_ERROR(i2c_master_probe(*bus, I2C_DEVICE_ADDRESS, -1), TAG, "BME680 not found");
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(*bus, &i2c_device_config, device), TAG, "Error when adding device");

    return ESP_OK;
}

#include "setup.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

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

int8_t bme68x_interface_init(struct bme68x_dev* bme, uint8_t intf) {
    if (!bme) {
        return BME68X_E_NULL_PTR;
    }

    if (intf == BME68X_SPI_INTF) {
        return BME68X_E_DEV_NOT_FOUND;
    }

    int8_t result = BME68X_OK;

    bme->read = bme68x_i2c_read;
    bme->write = bme68x_i2c_write;
    bme->intf = BME68X_I2C_INTF;

    i2c_master_bus_config_t bus_config = i2c_bus_config;
    bus_config.scl_io_num = SCL_GPIO;
    bus_config.sda_io_num = SDA_GPIO;

    i2c_interface_t* intf_ptr = calloc(1, sizeof(i2c_interface_t));

    i2c_master_bus_handle_t* bus = &intf_ptr->bus_handle;
    i2c_master_dev_handle_t* device = &intf_ptr->device_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus, &i2c_device_config, device));

    bme->delay_us = bme68x_delay_us;
    bme->intf_ptr = intf_ptr;
    bme->amb_temp = AMBIENT_TEMPERATURE;

    return result;
}

esp_err_t bme68x_interface_deinit(void* intf_ptr) {
    if(!intf_ptr) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_bus_handle_t bus = ((i2c_interface_t*)intf_ptr)->bus_handle;
    i2c_master_dev_handle_t device = ((i2c_interface_t*)intf_ptr)->device_handle;

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(device));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus));

    free(intf_ptr);

    return ESP_OK;
}

BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr) {
    if(!intf_ptr || !reg_data) {
        return BME68X_E_NULL_PTR;
    }

    i2c_master_dev_handle_t device = ((i2c_interface_t*)intf_ptr)->device_handle;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(device, &reg_addr, sizeof(reg_addr), reg_data, len, I2C_MAX_WAIT_MS));
    return BME68X_OK;
}

BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr) {
    if(!intf_ptr || !reg_data) {
        return BME68X_E_NULL_PTR;
    }

    i2c_master_dev_handle_t device = ((i2c_interface_t*)intf_ptr)->device_handle;
    ESP_ERROR_CHECK(i2c_master_transmit(device, reg_data, len, I2C_MAX_WAIT_MS));
    return BME68X_OK;
}

void bme68x_delay_us(uint32_t period, void* intf_ptr) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void bme68x_check_rslt(const char api_name[], int8_t rslt) {
    switch (rslt) {
        case BME68X_OK:
            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}
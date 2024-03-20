#include "setup.h"
#include "esp_pthread.h"
#include "esp_timer.h"
#include <stdio.h>

void delay_timer_callback(void* delay_signal);

int8_t bme68x_interface_init(struct bme68x_dev* bme, uint8_t intf) {
    if (!bme) {
        return BME68X_E_NULL_PTR;
    }

    if (intf == BME68X_SPI_INTF) {
        return BME68X_E_DEV_NOT_FOUND;
    }

    i2c_master_bus_config_t bus_config = i2c_bus_config;
    bus_config.scl_io_num = SCL_GPIO;
    bus_config.sda_io_num = SDA_GPIO;

    chip_interface_t* intf_ptr = calloc(1, sizeof(chip_interface_t));

    i2c_master_bus_handle_t* bus = &intf_ptr->bus;
    i2c_master_dev_handle_t* device = &intf_ptr->device;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus, &i2c_device_config, device));

    esp_timer_create_args_t timer_args = delay_timer_args;
    timer_args.callback = delay_timer_callback;
    timer_args.arg = &intf_ptr->delay_signal;

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &intf_ptr->delay_timer));

    intf_ptr->delay_signal.delay_mutex = PTHREAD_MUTEX_INITIALIZER;
    intf_ptr->delay_signal.stop_wait = PTHREAD_COND_INITIALIZER;
    intf_ptr->delay_signal.wait_status = WAIT_STATUS_UNKNOWN;

    bme->read = bme68x_i2c_read;
    bme->write = bme68x_i2c_write;
    bme->intf = BME68X_I2C_INTF;
    bme->delay_us = bme68x_delay_us;
    bme->intf_ptr = intf_ptr;
    bme->amb_temp = AMBIENT_TEMPERATURE;

    return BME68X_OK;
}

esp_err_t bme68x_interface_deinit(struct bme68x_dev* bme) {
    if (!bme) {
        return ESP_ERR_INVALID_ARG;
    }

    chip_interface_t* interface = bme->intf_ptr;

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(interface->device));
    ESP_ERROR_CHECK(i2c_del_master_bus(interface->bus));
    ESP_ERROR_CHECK(esp_timer_delete(interface->delay_timer));

    pthread_mutex_destroy(&interface->delay_signal.delay_mutex);
    pthread_cond_destroy(&interface->delay_signal.stop_wait);

    free(bme->intf_ptr);

    return ESP_OK;
}

BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr) {
    if (!intf_ptr || !reg_data) {
        return BME68X_E_NULL_PTR;
    }

    i2c_master_dev_handle_t device = ((chip_interface_t*)intf_ptr)->device;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(device, &reg_addr, sizeof(reg_addr), reg_data, len, I2C_MAX_WAIT_MS));
    return BME68X_OK;
}

BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr) {
    if (!intf_ptr || !reg_data) {
        return BME68X_E_NULL_PTR;
    }

    if(len > 16) {
        return BME68X_E_INVALID_LENGTH;
    }

    uint8_t write_buffer[32] = {};
    for (uint32_t offset = 0; offset < len; ++offset) {
        write_buffer[offset * 2] = reg_addr + offset;
        write_buffer[offset * 2 + 1] = reg_data[offset];
    }

    i2c_master_dev_handle_t device = ((chip_interface_t*)intf_ptr)->device;
    ESP_ERROR_CHECK(i2c_master_transmit(device, write_buffer, len * 2, I2C_MAX_WAIT_MS));
    return BME68X_OK;
}

void bme68x_delay_us(uint32_t period, void* intf_ptr) {
    uint64_t start = esp_timer_get_time();

    if (!intf_ptr) {
        return;
    }

    esp_timer_handle_t delay_timer = ((chip_interface_t*)intf_ptr)->delay_timer;
    delay_signal_t* delay_signal = &((chip_interface_t*)intf_ptr)->delay_signal;
    pthread_mutex_t* delay_mutex = &delay_signal->delay_mutex;
    pthread_cond_t* stop_wait = &delay_signal->stop_wait;
    wait_status_t* wait_status = &delay_signal->wait_status;

    pthread_mutex_lock(delay_mutex);

    *wait_status = WAIT_STATUS_WAITING;

    uint64_t delay_us = period - (esp_timer_get_time() - start);
    ESP_ERROR_CHECK(esp_timer_start_once(delay_timer, delay_us));

    while (*wait_status == WAIT_STATUS_WAITING) {
        pthread_cond_wait(stop_wait, delay_mutex);
    }

    pthread_mutex_unlock(delay_mutex);
}

void delay_timer_callback(void* delay_signal) {
    delay_signal_t* signal = delay_signal;
    pthread_mutex_t* delay_mutex = &signal->delay_mutex;
    pthread_cond_t* stop_wait = &signal->stop_wait;
    wait_status_t* wait_status = &signal->wait_status;

    pthread_mutex_lock(delay_mutex);

    *wait_status = WAIT_STATUS_DONE;
    pthread_cond_signal(stop_wait);

    pthread_mutex_unlock(delay_mutex);
}

void bme68x_check_rslt(const char api_name[], int8_t rslt) {
    switch (rslt) {
        case BME68X_OK:
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
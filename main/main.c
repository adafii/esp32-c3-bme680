#include "bme680.h"
#include "setup.h"
#include <stdio.h>

#define SAMPLE_COUNT       UINT8_C(50)

int app_main() {
    struct bme68x_dev bme = {};
    struct bme68x_conf conf = {};
    struct bme68x_data data = {};

    uint8_t n_fields = 0;
    uint16_t sample_count = 1;

    int8_t result = bme68x_interface_init(&bme, BME68X_I2C_INTF);
    bme68x_check_rslt("bme68x_interface_init", result);

    result = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", result);

    conf.filter = BME68X_FILTER_SIZE_127;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_16X;
    conf.os_temp = BME68X_OS_16X;
    result = bme68x_set_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_set_conf", result);

    printf("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status\n");

    while (sample_count <= SAMPLE_COUNT)
    {
        result = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        bme68x_check_rslt("bme68x_set_op_mode", result);

        uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme);
        bme.delay_us(del_period, bme.intf_ptr);

        uint64_t time_ms = esp_timer_get_time();

        result = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
        bme68x_check_rslt("bme68x_get_data", result);

        if (n_fields)
        {
            printf("%u, %lu, %.2f, %.2f, %.2f, %.2f, 0x%x\n",
                   sample_count,
                   (long unsigned int)time_ms,
                   data.temperature,
                   data.pressure,
                   data.humidity,
                   data.gas_resistance,
                   data.status);

            sample_count++;
        }
    }

    ESP_ERROR_CHECK(bme68x_interface_deinit(&bme));

    return result;
}

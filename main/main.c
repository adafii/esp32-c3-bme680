#include "bme680.h"

#define SDA_GPIO GPIO_NUM_8
#define SCL_GPIO GPIO_NUM_9

void app_main() {
    bme680_device_t bme680 = {};
    ESP_ERROR_CHECK(init_bme680_device(&bme680, SCL_GPIO, SDA_GPIO));

    ESP_ERROR_CHECK(set_oversampling(&bme680, OS_4, OS_4, OS_4));
    ESP_ERROR_CHECK(set_iir_filter(&bme680, COEFF_31));
}

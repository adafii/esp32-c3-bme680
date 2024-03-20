#pragma once
#include "bme680.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "pthread.h"

#define SDA_GPIO GPIO_NUM_8
#define SCL_GPIO GPIO_NUM_9
#define I2C_PORT (-1)
#define GLITCH_IGNORE_COUNT 7
#define I2C_CLOCK_HZ (400 * 1000)
#define I2C_MAX_WAIT_MS (-1)
#define I2C_DEVICE_ADDRESS BME68X_I2C_ADDR_HIGH
#define AMBIENT_TEMPERATURE 25

// Delay timer status for ignoring spurious wake-ups
typedef enum : int8_t {
    WAIT_STATUS_DONE = 0,
    WAIT_STATUS_WAITING = 1,
    WAIT_STATUS_UNKNOWN,
} wait_status_t;

// Used to signal ESP timer alarm to main task
typedef struct {
    pthread_mutex_t delay_mutex;
    pthread_cond_t stop_wait;
    wait_status_t wait_status;
} delay_signal_t;

// Basic intf_ptr type for ESP32
typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t device;
    esp_timer_handle_t delay_timer;
    delay_signal_t delay_signal;
} chip_interface_t;

// I2C bus default configs
static const i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .scl_io_num = 0,
    .sda_io_num = 0,
    .glitch_ignore_cnt = GLITCH_IGNORE_COUNT,
    .trans_queue_depth = 0,
    .flags.enable_internal_pullup = false,
};

// I2C device default configs
static const i2c_device_config_t i2c_device_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = I2C_DEVICE_ADDRESS,
    .scl_speed_hz = I2C_CLOCK_HZ,
};

// ESP timer default settings
static const esp_timer_create_args_t delay_timer_args = {.callback = NULL,
                                                         .arg = NULL,
                                                         .dispatch_method = ESP_TIMER_TASK,
                                                         .name = "delay_timer",
                                                         .skip_unhandled_events = true};

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *
 *  @param[in] bme      : Structure instance of bme68x_dev
 *  @param[in] intf     : Interface selection parameter
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bme68x_interface_init(struct bme68x_dev* bme, uint8_t intf);

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] len          : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BME68X_INTF_RET_SUCCESS -> Success
 *  @retval != BME68X_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);

/*!
 *  @brief Deinitializes the interface
 *
 *  @param[in] bme      : Structure instance of bme68x_dev
 *
 *  @return Status of execution
 */
esp_err_t bme68x_interface_deinit(struct bme68x_dev* bme);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose value is to be written.
 *  @param[in] len          : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = BME68X_INTF_RET_SUCCESS -> Success
 *  @retval != BME68X_INTF_RET_SUCCESS  -> Failure Info
 *
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period       : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 *
 */
void bme68x_delay_us(uint32_t period, void* /*intf_ptr*/);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bme68x_check_rslt(const char api_name[], int8_t rslt);

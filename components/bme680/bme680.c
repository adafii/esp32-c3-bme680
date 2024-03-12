#include "bme680.h"
#include "driver/spi_master.h"

const uint16_t WRITE_DATA = 0;
const uint16_t READ_DATA = 1;

void test() {
    spi_bus_config_t bus_config = {
        .mosi_io_num = 6,
        .miso_io_num = 5,
        .sclk_io_num = 4,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_DISABLED));

    spi_device_interface_config_t bme680_config = {
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
        .mode = 0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .clock_speed_hz = 1000 * 1000 * 1,
        .spics_io_num = 7,
        .flags = 0,
        .queue_size = 1,
    };

    spi_device_handle_t bme680_handle = 0;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &bme680_config, &bme680_handle));

    spi_transaction_t read_id = {
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .cmd = READ_DATA,
        .addr = 0x50,
        .length = 8,
        .rxlength = 8,
        .tx_data = {},
        .rx_data = {},
    };

    ESP_ERROR_CHECK(spi_device_transmit(bme680_handle, &read_id));

    printf("\nChip id: 0x%x\n", read_id.rx_data[0]);
}

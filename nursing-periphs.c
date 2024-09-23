#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// Accelerometer SPI Stuff
#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

#include "driver/gpio.h"

#include "hal/i2c_types.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "driver/i2c_slave.h"
#include "drivers/src/adxl345-i2c.c"

#include "inttypes.h"

#define ADXL_IO_INT1 GPIO_NUM_5
#define ADXL_IO_INT2 GPIO_NUM_6


bool yellow_led_val = 1;
bool green_led_val = 1;

i2c_master_dev_handle_t master_dev_handle;

/**
 * SINGLE_TAP event on INT1 (yellow)
 * FREE_FALL event on INT2 (green)
 */
static void IRAM_ATTR adxl_int1_isr_handler(void *arg)
{
    gpio_set_level(GPIO_NUM_1, yellow_led_val);
    yellow_led_val = !yellow_led_val;
    // Read out and clear the SINGLE_TAP interrupt
    adxl345_int_tap_clear(master_dev_handle);
}

static void IRAM_ATTR adxl_int2_isr_handler(void *arg)
{
    gpio_set_level(GPIO_NUM_2, green_led_val);
    green_led_val = !green_led_val;


}

void app_main()
{

    /*** Configure GPIO Pins for ADXL INT1 and INT 2 ***/
    gpio_reset_pin(ADXL_IO_INT1);
    gpio_reset_pin(ADXL_IO_INT2);

    gpio_config_t adxl_int1_config = {
        .pin_bit_mask = (1 << GPIO_NUM_5),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
        .intr_type = GPIO_INTR_POSEDGE, // rising edge interrupt
    };
    gpio_config(&adxl_int1_config);

    gpio_config_t adxl_int2_config = {
        .pin_bit_mask = (1 << GPIO_NUM_6),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
        .intr_type = GPIO_INTR_POSEDGE, // rising edge interrupt
    };
    gpio_config(&adxl_int2_config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ADXL_IO_INT1, adxl_int1_isr_handler, NULL);
    gpio_isr_handler_add(ADXL_IO_INT2, adxl_int2_isr_handler, NULL);

    gpio_set_level(ADXL_IO_INT1, 0);
    gpio_set_level(ADXL_IO_INT2, 0);

    gpio_intr_enable(ADXL_IO_INT1);
    gpio_intr_enable(ADXL_IO_INT2);

    /* TEMP<^> - LED gpio pins initialization*/
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(GPIO_NUM_1, GPIO_PULLUP_PULLDOWN);

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(GPIO_NUM_2, GPIO_PULLUP_PULLDOWN);

    /*** Configure I2C ***/
    i2c_master_bus_config_t master_bus_config = {
        .i2c_port = -1,
        .scl_io_num = GPIO_NUM_15,
        .sda_io_num = GPIO_NUM_16,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .trans_queue_depth = 20,
        .flags = {1} // enable pull up
    };

    i2c_master_bus_handle_t master_bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&master_bus_config, &master_bus_handle));

    /* Configure device */
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = 0x53, // address when SDO cnctd to GND
        .scl_speed_hz = 100000  // ADXL either 100kHz or 400kHz
    };

    i2c_master_bus_add_device(master_bus_handle, &device_config, &master_dev_handle);

    adxl345_init(master_dev_handle);

    // adxl345_int_tap_clear(master_dev_handle);

    adxl345_interrupt_init(master_dev_handle, 2000, 100, 800, 200);

     /* Take readings */
    uint8_t read_data[1] = {
        0x32, // DATA_X0 register
    };
    size_t read_data_len = 1;
    uint8_t readings_buf[6];
    size_t readings_buf_len = 6;

    while (1)
    {
        gpio_set_level(ADXL_IO_INT1, 0);

        vTaskDelay(20);
        gpio_set_level(ADXL_IO_INT1, 1);
        vTaskDelay(20);

        // i2c_master_transmit_receive(master_dev_handle, read_data, read_data_len,
        //                             readings_buf, readings_buf_len, 100);

        // int16_t x_out, y_out, z_out;
        // x_out = (readings_buf[0] | (readings_buf[1] << 8));
        // y_out = (readings_buf[2] | (readings_buf[3] << 8));
        // z_out = (readings_buf[4] | (readings_buf[5] << 8));
        // printf("x_out: %d, y_out: %d, z_out: %d\n", x_out, y_out, z_out);

        // vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

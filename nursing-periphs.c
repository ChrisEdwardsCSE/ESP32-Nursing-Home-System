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

#include "inttypes.h"

#include "drivers/Src/adxl345-i2c.c"
#include "drivers/Src/max30102-i2c.c"
#include "drivers/Src/algorithm_by_RF.c"

#define ADXL_IO_INT1    GPIO_NUM_5
#define ADXL_IO_INT2    GPIO_NUM_6
#define MAX_IO_INT      GPIO_NUM_7


bool yellow_led_val = 1;
bool green_led_val = 1;

i2c_master_dev_handle_t adxl_master_dev_handle;
i2c_master_bus_handle_t adxl_master_bus_handle;

i2c_master_dev_handle_t max_master_dev_handle;


#define ADXL_TESTING 1
#define MAX_TESTING 0

TaskHandle_t ISR = NULL;

uint8_t read_data[1] = {
    0x32, // DATA_X0 register
};
size_t read_data_len = 1;
uint8_t readings_buf[6];
size_t readings_buf_len = 6;
int16_t x, y, z;

void task1(void* arg)
{
    while(1)
    {
        i2c_master_transmit_receive(adxl_master_dev_handle, read_data, 1, 
                        readings_buf, 6, 100);
        x = readings_buf[0] | readings_buf[1] << 8;
        y = readings_buf[2] | readings_buf[3] << 8;
        z = readings_buf[4] | readings_buf[5] << 8;
        printf("x: %d, y: %d, z: %d\n", x,y,z);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }

}

/**
 * SINGLE_TAP event on INT1 (yellow)
 * FREE_FALL event on INT2 (green)
 */
static void IRAM_ATTR adxl_int1_isr_handler(void *arg)
{
    // printf("int1 triggered\n");
    gpio_set_level(GPIO_NUM_1, green_led_val);
    green_led_val = !green_led_val;
    // Read out and clear the SINGLE_TAP interrupt
    adxl345_int_tap_clear(adxl_master_dev_handle, adxl_master_bus_handle);
}

static void IRAM_ATTR adxl_int2_isr_handler(void *arg)
{
    // printf("int2 triggered\n");
    // gpio_set_level(GPIO_NUM_2, yellow_led_val);
    // yellow_led_val = !yellow_led_val;
    adxl345_int_tap_clear(adxl_master_dev_handle, adxl_master_bus_handle);
}


void app_main()
{
    /*** Configure GPIO Pins for ADXL INT1 and INT 2 ***/
    gpio_reset_pin(ADXL_IO_INT1);
    gpio_reset_pin(ADXL_IO_INT2);

    gpio_config_t adxl_int1_gpio_config = {
        .pin_bit_mask = (1 << ADXL_IO_INT1),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
        .intr_type = GPIO_INTR_POSEDGE, // rising edge interrupt
    };
    gpio_config(&adxl_int1_gpio_config);

    gpio_config_t adxl_int2_gpio_config = {
        .pin_bit_mask = (1 << ADXL_IO_INT2),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
        .intr_type = GPIO_INTR_POSEDGE, // rising edge interrupt
    };
    gpio_config(&adxl_int2_gpio_config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ADXL_IO_INT1, adxl_int1_isr_handler, NULL);
    gpio_isr_handler_add(ADXL_IO_INT2, adxl_int2_isr_handler, NULL);

    gpio_set_level(ADXL_IO_INT1, 0);
    gpio_set_level(ADXL_IO_INT2, 0);

    gpio_intr_enable(ADXL_IO_INT1);
    gpio_intr_enable(ADXL_IO_INT2);

    /* *********** TESTING ********* LED gpio pins initialization*/
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(GPIO_NUM_1, GPIO_PULLUP_PULLDOWN);

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(GPIO_NUM_2, GPIO_PULLUP_PULLDOWN);
    /* ***** TESTING ******* */

    /*** Configure I2C for ADXL345 ***/
    i2c_master_bus_config_t adxl_master_bus_config = {
        .i2c_port = -1,
        .scl_io_num = GPIO_NUM_15,
        .sda_io_num = GPIO_NUM_16,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .trans_queue_depth = 20,
        .flags = {1} // enable pull up
    };

    
    ESP_ERROR_CHECK(i2c_new_master_bus(&adxl_master_bus_config, &adxl_master_bus_handle));

    /* Configure device */
    i2c_device_config_t adxl_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = 0x53, // address when SDO cnctd to GND
        .scl_speed_hz = 100000  // ADXL either 100kHz or 400kHz
    };

    i2c_master_bus_add_device(adxl_master_bus_handle, &adxl_device_config, &adxl_master_dev_handle);

    adxl345_init(adxl_master_dev_handle);

    adxl345_interrupt_init(adxl_master_dev_handle, adxl_master_bus_handle, 2500, 100, 800, 200);

    adxl345_int_tap_clear(adxl_master_dev_handle, adxl_master_bus_handle);


    xTaskCreate(task1, "task1", 4096, NULL, 10, &ISR);

}

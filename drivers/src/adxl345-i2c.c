#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "hal/spi_types.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ADXL_POWER_CTL_REG          0x2d
#define ADXL_DATA_FORMAT_REG        0x21
#define ADXL_DATA_XOUT_REG          0x32
#define ADXL_INT_ENABLE_REG         0x2e
#define ADXL_INT_MAP_REG            0x2f
#define ADXL_INT_THRESH_TAP_REG     0x1d
#define ADXL_INT_THRESH_DUR_REG     0x21
#define ADXL_INT_THRESH_FF_REG      0x28
#define ADXL_INT_TIME_FF_REG        0x29
#define ADXL_TAP_AXES               0x2a
#define ADXL_INT_SOURCE             0x30

void adxl345_int_tap_clear(i2c_master_dev_handle_t master_dev_handle)
{
    /* Clear the interrupt source register */
    uint8_t int_source_buf[1] = {
        ADXL_INT_SOURCE
    };
    uint8_t read_buf[1];
    i2c_master_transmit_receive(master_dev_handle, int_source_buf, 1,
                                read_buf, 1, 100);
}

void adxl345_init(i2c_master_dev_handle_t master_dev_handle)
{
    /* Power up and put into Measure Mode */
    uint8_t init_seq[2] = {
        ADXL_POWER_CTL_REG, // POWER_CTL register write
        0x08  // Put in Measure Mode
    };
    i2c_master_transmit(master_dev_handle, init_seq, 2, 100);

    vTaskDelay(1);

    /* Format Data */
    uint8_t format_data[2] = {
        ADXL_DATA_FORMAT_REG, // DATA_FORMAT register address
        0x01 // 4g detection
    };
    i2c_master_transmit(master_dev_handle, format_data, 2, 100);
    vTaskDelay(10);
}

/**
 * Set up the interrupts on SINGLE_TAP and FREE_FALL occurrences.
 * SINGLE_TAP on INT1 IO pin; FREE_FALL on INT2 IO pin
 * 
 * @param master_dev_handle I2C master device handler instance
 * @param int_tap_thresh treshold minimum acceleration magnitude (mg) for 
 * TAP interrupt to occur. Range 0 to 16000mg
 * @param int_thresh_dur treshold maximum duration of acceleration (ms) for 
 * TAP interrupt to trigger. Range 0 to 160ms
 * @param int_thresh_ff threshold maximum acceleration magnitude (mg) for 
 * FF interrupt to trigger. Recommended 300mg - 600mg
 * @param int_time_ff threshold minimum time (ms) for FF interrupt to trigger.
 *  Recommended 100ms - 350ms
 */
void adxl345_interrupt_init(i2c_master_dev_handle_t master_dev_handle, uint16_t int_thresh_tap,
                            uint8_t int_thresh_dur, uint16_t int_thresh_ff, uint16_t int_time_ff)
{
    uint8_t error_check_buf[1];
    uint8_t temp_reg = ADXL_INT_THRESH_TAP_REG;

    // Set up interrupt IO mapping
    uint8_t int_mapping_buf[2] = {
        ADXL_INT_MAP_REG,
        (uint8_t)( (1 << 2) & ~(1 << 6)) // SINGLE_TAP on INT1, FREE_FALL on INT2, 
    };
    i2c_master_transmit(master_dev_handle, int_mapping_buf, 2, 100);
    vTaskDelay(2);
    temp_reg = ADXL_INT_MAP_REG;
    i2c_master_transmit_receive(master_dev_handle, &temp_reg, 
                                1, error_check_buf, 1, 100);
    printf("INT_MAP: 0x%X\n", error_check_buf[0]);
    vTaskDelay(1);
    
    /*** TAP Interrupt Configuration ***/
    // Set threshold acceleration in TAP interrupt
    int_thresh_tap /= 62.5; // .0625g/bit
    uint8_t int_thresh_tap_buf[2] = {
        ADXL_INT_THRESH_TAP_REG,
        int_thresh_tap 
    };
    i2c_master_transmit(master_dev_handle, int_thresh_tap_buf, 2, 100);
    vTaskDelay(2);

    i2c_master_transmit_receive(master_dev_handle, &temp_reg, 
                                1, error_check_buf, 1, 100);
    printf("INT_TRESH_TAP: 0x%X\n", error_check_buf[0]);
    vTaskDelay(1);

    // Set threshold maximum duration of TAP acceleration 
    int_thresh_dur /= 0.625; // 625us/bit
    uint8_t int_thresh_dur_buf[2] = {
        ADXL_INT_THRESH_DUR_REG,
        int_thresh_dur
    };
    i2c_master_transmit(master_dev_handle, int_thresh_dur_buf, 2, 100);
    vTaskDelay(2);
    temp_reg = ADXL_INT_THRESH_DUR_REG;
    i2c_master_transmit_receive(master_dev_handle, &temp_reg, 
                                1, error_check_buf, 1, 100);
    printf("INT_THRESH_DUR: 0x%X\n", error_check_buf[0]);
    vTaskDelay(1);

    /*** Free Fall Interrupt Configuration ***/
    // Set treshold minimum acceleration to register as a free fall
    int_thresh_ff /= 62.5; // 62.5mg/bit
    uint8_t int_thresh_ff_buf[2] = {
        ADXL_INT_THRESH_FF_REG,
        int_thresh_ff
    };
    i2c_master_transmit(master_dev_handle, int_thresh_ff_buf, 2, 100);
    vTaskDelay(2);
    temp_reg = ADXL_INT_THRESH_FF_REG;
    i2c_master_transmit_receive(master_dev_handle, &temp_reg, 
                                1, error_check_buf, 1, 100);
    printf("INT_THRESH_FF: 0x%X\n", error_check_buf[0]);
    vTaskDelay(1);

    // Set threshold minimnum time of free fall occurrence for an interrupt to trigger
    int_time_ff /= 5; // 5ms/bit
    uint8_t int_time_ff_buf[2] = {
        ADXL_INT_TIME_FF_REG,
        int_time_ff
    };
    i2c_master_transmit(master_dev_handle, int_time_ff_buf, 2, 100);
    vTaskDelay(2);
    temp_reg = ADXL_INT_TIME_FF_REG;
    i2c_master_transmit_receive(master_dev_handle, &temp_reg, 
                                1, error_check_buf, 1, 100);
    printf("INT_TIME_FF: 0x%X\n", error_check_buf[0]);
    vTaskDelay(1);

    uint8_t int_enable_tap_axes_buf[2] = {
        ADXL_TAP_AXES,
        (uint8_t)( (1 << 2) | (1 << 1) | (1 << 0)) // Enable x, y, and z for TAP events
    };
    i2c_master_transmit(master_dev_handle, int_enable_tap_axes_buf, 2, 100);
    vTaskDelay(2);
    temp_reg = ADXL_TAP_AXES;
    i2c_master_transmit_receive(master_dev_handle, &temp_reg, 
                                1, error_check_buf, 1, 100);
    printf("INT_ENABLE_TAP_AXES: 0x%X\n", error_check_buf[0]);
    vTaskDelay(1);

    // Enable SINGLE_TAP and FREE_FALL interrupts
    uint8_t int_enable_buf[2] = {
        ADXL_INT_ENABLE_REG,
        (uint8_t)((1 << 6) | (1 << 2))
    };
    i2c_master_transmit(master_dev_handle, int_enable_buf, 2, 100);
    vTaskDelay(2);
    temp_reg = ADXL_INT_ENABLE_REG;
    i2c_master_transmit_receive(master_dev_handle, &temp_reg, 
                            1, error_check_buf, 1, 100);
    printf("INT_ENABLE: 0x%X\n", error_check_buf[0]);
    vTaskDelay(1);
}
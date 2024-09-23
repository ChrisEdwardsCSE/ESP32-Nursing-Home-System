#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

float x_out, y_out, z_out;        // Outputs of accelerometer
int x_offset, y_offset, z_offset; // Offset values

void __adxl345_calibrate(spi_device_handle_t spi_handler);
uint8_t *__adxl345_send_cmd(spi_device_handle_t spi_handler, void *tx_buf,
                            uint16_t tx_buf_len);

void adxl345_init(spi_device_handle_t spi_handler)
{

    uint8_t init_sequence[4] =
        {
            0x00,
            0x2d, // POWER_CTL Register
            0x08  // Set Enable Measurement Bit
        };

    __adxl345_send_cmd(spi_handler, init_sequence, 4);
}

/**
 * Send command to the ADXL345.
 * @param spi_handler - the spi_handler_t of the peripheral
 * @param tx_buf - the buffer to send
 * @param tx_buf_len - the length of tx_buf in BYTES
 *
 * @return the received data
 */
uint8_t *__adxl345_send_cmd(spi_device_handle_t spi_handler, void *tx_buf,
                            uint16_t tx_buf_len)
{
    uint8_t rx_buf[tx_buf_len];

    tx_buf_len *= 8; // Bytes -> bits

    spi_transaction_t spi_transaction;
    memset(&spi_transaction, 0, sizeof(spi_transaction));
    spi_transaction = (spi_transaction_t){
        .length = tx_buf_len, // 8 bit transactions
        .tx_buffer = tx_buf,
        .rxlength = tx_buf_len,
        .rx_buffer = rx_buf};

    spi_device_transmit(spi_handler, &spi_transaction);

    // printf("%s\n", rx_buf);
    return rx_buf;
}

uint8_t *__adxl345_send_cmd2(spi_device_handle_t spi_handler)
{
    uint8_t rx_buf[40];

    uint8_t tx_buf[5] = {0x32, 0x33, 0x34, 0x35, 0x36};
    uint16_t tx_buf_len = 5;

    tx_buf_len *= 8; // Bytes -> bits

    spi_transaction_t spi_transaction;
    memset(&spi_transaction, 0, sizeof(spi_transaction));
    spi_transaction = (spi_transaction_t){
        .length = tx_buf_len, // 8 bit transactions
        .tx_buffer = tx_buf,
        .rxlength = tx_buf_len,
        .rx_buffer = rx_buf};

    spi_device_transmit(spi_handler, &spi_transaction);

    // printf("%s\n", rx_buf);
    return rx_buf;
}

// static void __adxl345_format_data(spi_device_handle_t spi_handler)
// {
//     uint8_t format_data = {
//         0x31,      // reg address
//         0b00000010 // 8g data range
//     };

//     spi_transaction_t spi_transaction;
//     memset(&spi_transaction, 0, sizeof(spi_transaction)); // Reset the transaction memory
//     spi_transaction = {
//         .length = 8, // 8 bit transactions
//         .tx_buffer = init_sequence,
//     };
// }

// static void __adxl345_calibrate(spi_device_handle_t spi_handler)
// {
//     uint16_t num_readings = 500;

//     float z_sum = 0;

//     char x_out_reg[1] = {0x32}; // x_out register

//     spi_transaction_t spi_transaction;
//     memset(&spi_transaction, 0, sizeof(spi_transaction)); // Reset the transaction memory
//     spi_transaction = {
//         .length = 8, // 8 bit transactions
//         .rx_length = 8,
//         .tx_buffer = x_out_reg,
//     };
// }
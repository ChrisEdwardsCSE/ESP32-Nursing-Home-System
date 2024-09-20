// NimBLE Client - Scan

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "sdkconfig.h"

// Temperature I2C Stuff
#include "driver/i2c.h"
#include "driver/i2c_master.h"

// Accelerometer SPI Stuff
#include "hal/spi_types.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

char *TAG = "BLE Client Scan";
uint8_t ble_addr_type;
void ble_app_scan(void);
void ble_app_advertise(void);

/* *************** SCANNING ****************** */
/* BLE event handling for scanning - the cb associated with discovery event.
receives the data and does stuff with it */
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
  struct ble_hs_adv_fields fields;

  // the different connection events we could have
  switch (event->type)
  {
  // discovery event case
  case BLE_GAP_EVENT_DISC:
    ESP_LOGI("GAP", "GAP EVENT DISCOVERY");
    // parse the adv data fields from a source buffer into fields`
    ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
    // ble_hs_adv_parse();

    if (fields.name_len > 0)
    {
      printf("Name: %.*s, data: %s\n", fields.name_len, fields.name, fields.mfg_data);
    }
    break;
  default:
    break;
  }
  return 0;
}

// SIMS
void ble_app_scan(void)
{
  printf("Start scanning ...\n");

  /* define GAP connectivity */
  struct ble_gap_disc_params disc_params;
  disc_params.filter_duplicates = 1; // no duplicates
  disc_params.passive = 0;
  disc_params.itvl = 0;
  disc_params.window = 0;
  disc_params.filter_policy = 0;
  disc_params.limited = 0;

  /* handles discovery procedure, so does all the parts of configuring the RF receiver,
  and setting channel, how long to scan for, what address should it call itself.
  Also calls the cb on different GAP events (notably discovery/received adv);
  ble_gap_event is the handler callback */
  ble_gap_disc(ble_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
}

/* ************** ADVERTISING ***************** */

// advertise on sync (the application) from innovate yrslf
void ble_app_advertise(void)
{
  /* https://github.com/apache/mynewt-nimble/blob/master/nimble/host/include/host/ble_hs_adv.h#L60
  advertising fields struct. has the 16b UUIDs, 32b UUIDs, 12b UUIDS, local
  names, TX power level, public target address, adv interval, broadcast name */
  struct ble_hs_adv_fields fields;
  const char *device_name;
  memset(&fields, 0, sizeof(fields));
  device_name = ble_svc_gap_device_name(); // read device name
  fields.name = (uint8_t *)device_name;    // local names
  fields.name_len = strlen(device_name);
  fields.name_is_complete = 1; // indicates list of local names is complete

  // This'll be the data of the heartrate, temperature, & fall detection that we're sending
  const uint8_t the_data[9] = "the_data";
  fields.mfg_data = the_data;
  ble_gap_adv_set_fields(&fields); // configures fields for advertisements

  /* advertising param: connection mode, discoverable mode, min & max adv interval*/
  struct ble_gap_adv_params adv_params;
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_NON; // unconnectable
  adv_params.disc_mode = BLE_GAP_DISC_MODE_NON; // undiscoverable, we don't need scan responses

  // starts the advertising w/ the params
  ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// void ble_gap_scan_response(void)
// {
//   ble_gap_adv_rsp_set_fields();
//   ble_gap_adv_rsp_set_data();
// }

// our actual application
void ble_app_on_sync(void)
{
  /* Determines the best address type, sets ble_addr_type with it*/
  int res = ble_hs_id_infer_auto(0, &ble_addr_type);
  if (res != 0)
  {
    printf("Device Address Set Error: %d", res);
  }
  ble_app_advertise();
  // ble_app_scan();
}

// The infinite task
void host_task(void *param)
{
  nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

void app_main()
{
  nvs_flash_init();                               // 1 - Initialize NVS flash using
                                                  // esp_nimble_hci_and_controller_init();      // 2 - Initialize ESP controller - not needed anymore
  nimble_port_init();                             // 3 - Initialize the controller stack
  ble_svc_gap_device_name_set("ESP32-Scan-ched"); // 4 - Set device name characteristic
  ble_svc_gap_init();                             // 4 - Initialize GAP service
  ble_hs_cfg.sync_cb = ble_app_on_sync;           // 5 - Set application
  nimble_port_freertos_init(host_task);           // 6 - Set infinite task

  /* Configure I2C */
  i2c_master_bus_config_t i2c_master_bus_conf = {
    // i2c_port_num_t i2c_port;              /*!< I2C port number, `-1` for auto selecting */
    // gpio_num_t sda_io_num;                /*!< GPIO number of I2C SDA signal, pulled-up internally */
    // gpio_num_t scl_io_num;                /*!< GPIO number of I2C SCL signal, pulled-up internally */
    // i2c_clock_source_t clk_source;        /*!< Clock source of I2C master bus, channels in the same group must use the same clock source */
    // uint8_t glitch_ignore_cnt;            /*!< If the glitch period on the line is less than this value, it can be filtered out, typically value is 7 (unit: I2C module clock cycle)*/
    // int intr_priority;                    /*!< I2C interrupt priority, if set to 0, driver will select the default priority (1,2,3). */
    // size_t trans_queue_depth;             /*!< Depth of internal transfer queue, increase this value can support more transfers pending in the background, only valid in asynchronous transaction. (Typically max_device_num * per_transaction)*/
    // struct {
    //     uint32_t enable_internal_pullup:1;   /*!< Enable internal pullups. Note: This is not strong enough to pullup buses under high-speed frequency. Recommend proper external pull-up if possible */
    // } flags;
    .i2c_port = -1,
    .sda_io_num = GPIO_NUM_4,
    .scl_io_num = GPIO_NUM_5,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7
  };
  i2c_device_config_t i2c_device_conf = {
    // i2c_addr_bit_len_t dev_addr_length;         /*!< Select the address length of the slave device. */
    // uint16_t device_address;                    /*!< I2C device raw address. (The 7/10 bit address without read/write bit) */
    // uint32_t scl_speed_hz;                      /*!< I2C SCL line frequency. */
    // uint32_t scl_wait_us;                      /*!< Timeout value. (unit: us). Please note this value should not be so small that it can handle stretch/disturbance properly. If 0 is set, that means use the default reg value*/
    // struct {
    //     uint32_t disable_ack_check:      1;     /*!< Disable ACK check. If this is set false, that means ack check is enabled, the transaction will be stoped and API returns error when nack is detected. */
    // } flags;                                    /*!< I2C device config flags */
    .dev_addr_length = 7,
    .device_address = 0x27,
    .scl_speed_hz = 10000,
  };


  /* Configure SPI for Accelerometer Reading */
  spi_bus_config_t spi_bus_config = {
      //   union {
      //     int mosi_io_num;    ///< GPIO pin for Master Out Slave In (=spi_d) signal, or -1 if not used.
      //     int data0_io_num;   ///< GPIO pin for spi data0 signal in quad/octal mode, or -1 if not used.
      //   };
      //   union {
      //     int miso_io_num;    ///< GPIO pin for Master In Slave Out (=spi_q) signal, or -1 if not used.
      //     int data1_io_num;   ///< GPIO pin for spi data1 signal in quad/octal mode, or -1 if not used.
      //   };
      //   int sclk_io_num;      ///< GPIO pin for SPI Clock signal, or -1 if not used.
      //   union {
      //     int quadwp_io_num;  ///< GPIO pin for WP (Write Protect) signal, or -1 if not used.
      //     int data2_io_num;   ///< GPIO pin for spi data2 signal in quad/octal mode, or -1 if not used.
      //   };
      //   union {
      //     int quadhd_io_num;  ///< GPIO pin for HD (Hold) signal, or -1 if not used.
      //     int data3_io_num;   ///< GPIO pin for spi data3 signal in quad/octal mode, or -1 if not used.
      //   };
      //   int data4_io_num;     ///< GPIO pin for spi data4 signal in octal mode, or -1 if not used.
      //   int data5_io_num;     ///< GPIO pin for spi data5 signal in octal mode, or -1 if not used.
      //   int data6_io_num;     ///< GPIO pin for spi data6 signal in octal mode, or -1 if not used.
      //   int data7_io_num;     ///< GPIO pin for spi data7 signal in octal mode, or -1 if not used.
      //   int max_transfer_sz;  ///< Maximum transfer size, in bytes. Defaults to 4092 if 0 when DMA enabled, or to `SOC_SPI_MAXIMUM_BUFFER_SIZE` if DMA is disabled.
      //   uint32_t flags;       ///< Abilities of bus to be checked by the driver. Or-ed value of ``SPICOMMON_BUSFLAG_*`` flags.
      //   esp_intr_cpu_affinity_t  isr_cpu_id;    ///< Select cpu core to register SPI ISR.
      //   int intr_flags;       /**< Interrupt flag for the bus to set the priority, and IRAM attribute, see
      //                          *  ``esp_intr_alloc.h``. Note that the EDGE, INTRDISABLED attribute are ignored
      //                          *  by the driver. Note that if ESP_INTR_FLAG_IRAM is set, ALL the callbacks of
      //                          *  the driver, and their callee functions, should be put in the IRAM.
      //                          */
      .sclk_io_num = GPIO_NUM_15,
      .mosi_io_num = GPIO_NUM_16,
      .miso_io_num = GPIO_NUM_17,
      .max_transfer_sz = 32,
      .data4_io_num = -1,
      .data5_io_num = -1,
      .data6_io_num = -1,
      .data7_io_num = -1,
  };

  spi_device_interface_config_t spi_dev_conf = {
      .clock_speed_hz = 1000000, // max of accel. is 5M
      .mode = 0,                 //
      .spics_io_num = GPIO_NUM_18,
      .queue_size = 7,
  };

  // Initialize SPI peripheral with configurations
  spi_bus_initialize(SPI2_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);

  // Set up SPI for use
  spi_device_handle_t spi_handler;
  spi_bus_add_device(SPI2_HOST, &spi_dev_conf, &spi_handler);

  char tx_buf[8] = {1,2,3,4,8,7,6,5};
  char rx_buf[8];
  spi_transaction_t spi_transaction;
  memset(&spi_transaction, 0, sizeof(spi_transaction)); // Reset the transaction memory
  spi_transaction = {
    .length = 8, // 8 bit transactions
    .rxlength = 8, // receive 8 bits worth
    .tx_buffer = tx_buf,
    .rx_buffer = rx_buf,
  };

  spi_device_transmit(spi_handler, )
}

/**
 * Task for getting temperature reading
 */
void get_temperature()
{
  for (;;)
  {
  }
}

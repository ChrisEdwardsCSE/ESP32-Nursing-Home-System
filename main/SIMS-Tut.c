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

char *TAG = "BLE Client Scan";
uint8_t ble_addr_type;
void ble_app_scan(void);

/* BLE event handling - the cb associated with discovery event. receives the data
and does stuff with it */
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
    if (fields.name_len > 0)
    {
      printf("Name: %.*s\n", fields.name_len, fields.name);
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
  disc_params.passive = 0;           // want to use active scan - finds better
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

// advertise on sync (the application) from innovate yrslf
void ble_gap_advertise(void)
{
  /* https://github.com/apache/mynewt-nimble/blob/master/nimble/host/include/host/ble_hs_adv.h#L60 
  advertising fields struct. has the 16b UUIDs, 32b UUIDs, 12b UUIDS, local
  names, TX power level, public target address, adv interval, broadcast name */
  struct ble_hs_adv_fields fields;
  const char *device_name;
  memset(&fields, 0, sizeof(fields));
  device_name = ble_svc_gap_device_name(); // read device name
  fields.name = (uint8_t*)device_name; // local names
  fields.name_len = strlen(device_name);
  fields.name_is_complete = 1; // indicates list of local names is complete
  ble_gap_adv_set_fields(&fields); // configures fields for advertisements

  /* advertising param . connection mode, discoverable mode, min & max adv interval*/
  struct ble_gap_adv_params adv_params;
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable undirected
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // general discoverable
  // starts the advertising w/ the params
  ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);

}

// our actual application
void ble_app_on_sync(void)
{
  ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
  ble_app_scan();`
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
}
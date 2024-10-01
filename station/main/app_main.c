/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"

#include "sdkconfig.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

#include "mqtt_client.h"

#include "driver/gpio.h"
#include "hal/i2c_types.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "driver/i2c_slave.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define ESP_WIFI_SSID "chris-iphone-12"
#define ESP_WIFI_PASS "ull-nvr-guess-this-69"

// event for status information
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_SUCCESS    1 << 0
#define WIFI_FAIL       1 << 1
#define TCP_SUCCESS 1 << 0
#define TCP_FAIL 1 << 1
#define MAX_FAILURES 10

static const char *BLE_TAG = "BLE";
static const char *WIFI_TAG = "WIFI";

static int s_retry_num = 0; // counts number of connection attempts

TaskHandle_t mqtt_app_start_task_handle;
TaskHandle_t wifi_init_start_task_handle;

uint8_t ble_addr_type;

esp_mqtt_client_handle_t mqtt_client;
void ble_app_scan(void);
void ble_app_advertise(void);

static void mqtt_app_start(void);

// configTASK_NOTIFICATION_ARRAY_ENTRIES

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(WIFI_TAG, "Last error %s: 0x%x", message, error_code);
    }
}


/* ******************** WiFi ******************** */

/**
 * WiFi event handler
 * @param event_base - base type of event, should be WIFI_EVENT
 * @param event_id - the event that occurred
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    // If WiFi-related event & it's a station start
    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect(); // Connect to WiFi, eventually calls ip_event_handler
        ESP_LOGI(WIFI_TAG, "Connecting to AP...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_FAILURES) { // Retry to connect MAX_FAILURES times
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(WIFI_TAG, "Reconnecting to AP...");
        } else { // If number of tries > MAX_FAILURES, then connection failed. Set event group
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL);
            ESP_LOGI(WIFI_TAG, "Connection to AP failed");
        }
    }
    else {
        ESP_LOGI(WIFI_TAG, "Wifi Event: %ld", event_id);
    }
}

/**
 * IP event handler
 * @param event_base - base type of event, should be IP_EVENT
 * @param event_id - the IP event that occurred
 */
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_SUCCESS);
    }
    else {
        ESP_LOGI(WIFI_TAG, "IP Event: %ld", event_id);
    }
}

esp_netif_t* esp_netif;
uint8_t first_time = 1; 
void wifi_init_sta(void*)
{

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init()); // Initialize TCP/IP stack

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    

    for(;;) {
        esp_netif = esp_netif_create_default_wifi_sta(); // Create default WiFi station

    
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg)); 

        /* Establish wifi event event handler if see WIFI_EVENT event of ESP_ANY_EVENT_ID 
        (any wifi event at all), then call wifi_event_handler. */
        esp_event_handler_instance_t wifi_event_handler_instance;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            &wifi_event_handler_instance));

        /* Establish IP event event handler; if IP_EVENT_STA_GOT_IP (we, the STA=station, got an
        IP) then call ip_event_handler. the ip_event_handler gets called sometime after we've connected
        to wifi */
        esp_event_handler_instance_t ip_event_handler_instance;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP,
                                                            &ip_event_handler,
                                                            NULL,
                                                            &ip_event_handler_instance));

        wifi_config_t wifi_config = {
            .sta = {
                .ssid = ESP_WIFI_SSID,
                .password = ESP_WIFI_PASS,
                /* Setting a password implies station will connect to all security modes including WEP/WPA.
                * However these modes are deprecated and not advisable to be used. Incase your Access point
                * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

                .pmf_cfg = {
                    .capable = true,
                    .required = false
                },
            },
        };

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // Set WiFi to be a station
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config)); // Configure WiFi


        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_ERROR_CHECK(esp_wifi_start()); // Start the WiFi driver

        ESP_LOGI(WIFI_TAG, "wifi_init_sta finished.");


        /* Wait in BLOCKED for either WIFI_SUCCESS or WIFI_FAIL events to occur. 
        * These get set by wifi_event_handler() */
        EventBits_t wifi_result_bits = xEventGroupWaitBits(
                s_wifi_event_group,         // Event group
                WIFI_SUCCESS | WIFI_FAIL,   // Bits to wait for
                pdFALSE,                    // Don't clear bits on exit
                pdFALSE,                    // Any of the specified bits unblock the task 
                portMAX_DELAY);             // Wait forever


        if (wifi_result_bits & WIFI_SUCCESS) {
            ESP_LOGI(WIFI_TAG, "connected to ap SSID:%s password:%s",
                    ESP_WIFI_SSID, ESP_WIFI_PASS);
        } else if (wifi_result_bits & WIFI_FAIL) {
            ESP_LOGI(WIFI_TAG, "Failed to connect to SSID:%s, password:%s",
                    ESP_WIFI_SSID, ESP_WIFI_PASS);
        } else {
            ESP_LOGE(WIFI_TAG, "UNEXPECTED EVENT");
        }

        /* The event will not be processed after unregister */
        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_event_handler_instance));
        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler_instance));
        vEventGroupDelete(s_wifi_event_group);

        mqtt_app_start();
        

        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        ESP_LOGI(WIFI_TAG, "Stopping Scanning");
        esp_mqtt_client_stop(mqtt_client);
        esp_wifi_stop();
        esp_wifi_deinit();
        esp_netif_destroy_default_wifi((void*)esp_netif);
        esp_wifi_clear_default_wifi_driver_and_handlers((void*)esp_netif);
        vTaskDelay(pdMS_TO_TICKS(10));
        ble_app_scan();
        ESP_LOGI(BLE_TAG, "BLE scanning starting back up again");
    }
}


/* ************ MQTT ****************** */
uint8_t mqtt_published_flag = 0;
/*
 * MQTT event handler
 *
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(WIFI_TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(WIFI_TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "residents", "data_3", 0, 1, 0);
        ESP_LOGI(WIFI_TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        if (!mqtt_published_flag) {
            ESP_LOGI(WIFI_TAG, "Disconnected without publishing");
            // Do some guarantee that the message is sent once reconnecting
        }
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(WIFI_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        mqtt_published_flag = 1;
        xTaskNotifyGiveIndexed(wifi_init_start_task_handle, 1);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(WIFI_TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(WIFI_TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(WIFI_TAG, "Other event id:%d", event->event_id);
        break;
    }
}

/**
 * Creates and starts the client as well as event handler
 */
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg); // Create MQTT client

    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL); // Register MQTT event handler cb

    esp_mqtt_client_start(mqtt_client);

}

/* ********** BLE Scanning ********** */

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
        // ESP_LOGI("GAP", "GAP EVENT DISCOVERY");
        // parse the adv data fields from a source buffer into fields`
        ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);

        if (fields.name_len > 0)
        {
            ESP_LOGI(BLE_TAG, "Name: %.*s, data: %.*s\n", fields.name_len, fields.name, fields.mfg_data_len,fields.mfg_data);
            char received_data[fields.mfg_data_len];
            strncpy(received_data, (char*)fields.mfg_data, fields.mfg_data_len);
            if (!strncmp(received_data, "the_data", 8)) {
                ESP_LOGI(BLE_TAG, "GOT ITTT\n");
                ble_gap_disc_cancel();
                vTaskDelay(pdMS_TO_TICKS(10));
                xTaskNotifyGive(wifi_init_start_task_handle);
            }
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

// our actual application
void ble_app_on_sync(void)
{
    /* Determines the best address type, sets ble_addr_type with it*/
    int res = ble_hs_id_infer_auto(0, &ble_addr_type);
    if (res != 0)
    {
        printf("Device Address Set Error: %d", res);
    }
    ble_app_scan();
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

void app_main(void)
{
    ESP_LOGI(WIFI_TAG, "[APP] Startup..");
    ESP_LOGI(WIFI_TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(WIFI_TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());

    nimble_port_init();                             // 3 - Initialize the controller stack
    ble_svc_gap_device_name_set("ESP32-Scan-ched"); // 4 - Set device name characteristic
    ble_svc_gap_init();                             // 4 - Initialize GAP service
    ble_hs_cfg.sync_cb = ble_app_on_sync;           // 5 - Set application as callback
    nimble_port_freertos_init(host_task);           // 6 - Set infinite task


    xTaskCreate(wifi_init_sta, "WiFi Init",
                4096, NULL, configMAX_PRIORITIES-1,
                &wifi_init_start_task_handle);

}





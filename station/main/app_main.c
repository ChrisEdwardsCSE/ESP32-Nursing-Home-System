/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* pretty sure this is SIMS + LLL for wifi not 100%*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "WIFI";


/************ LLL WIFI **************/

/** DEFINES **/
#define WIFI_SUCCESS 1 << 0
#define WIFI_FAILURE 1 << 1
#define TCP_SUCCESS 1 << 0
#define TCP_FAILURE 1 << 1
#define MAX_FAILURES 10

/** GLOBALS **/

// event group to contain status information
static EventGroupHandle_t wifi_event_group;

// retry tracker
static int s_retry_num = 0;



//event handler for wifi events
/**
 * Gets called on wifi events (cfg'd for now to be on any wifi events)
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
  // if wifi event & the event is a station_start
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	{
		ESP_LOGI(TAG, "Connecting to AP...");
		esp_wifi_connect(); // connects to the wifi, think this will eventually call ip_event_handler
  // else if wifi even but it's station disconnected
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
    // track the failures we've had, 
		if (s_retry_num < MAX_FAILURES)
		{
			ESP_LOGI(TAG, "Reconnecting to AP...");
			esp_wifi_connect();
			s_retry_num++;
    // if num tries is above MAX_FAILURES, then set in status handler (wifi_event_group) that we've failed
		} else {
			xEventGroupSetBits(wifi_event_group, WIFI_FAILURE);
		}
	}
}

//event handler for ip events
/**
 * Gets called sometime after we connect to wifi (esp_wifi_connect in wifi_event_handler)
 * 
 */
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
  // if it's an IP event and station got an IP from the network
	if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data; // idk just get the event data
        ESP_LOGI(TAG, "STA IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_SUCCESS); // note in status tracker that we've had a WIFI SUCCESS
    }

}

/**
 * Connect to Wifi. cfgs wifi connect stuff. runs the inf wifi event loop.
 * Sets the event handlers for wifi/ip_event_handler and waits for them to execute
 * with a result. Stashes the result in status (WIFI_SUCCESS/FAILURE)
 */
esp_err_t connect_wifi()
{
	int status = WIFI_FAILURE;

	/** INITIALIZE ALL THE THINGS **/
	//initialize the esp network interface
	ESP_ERROR_CHECK(esp_netif_init());

	// initialize inf default running event loop
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	// create wifi station in the wifi driver "we're going to do wifi, get ready"
	esp_netif_create_default_wifi_sta();

  /* set up wifi cfg for things like ssid, password, type of auth, etc.
  this one is default just for beginnings */
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  /** EVENT LOOP CRAZINESS **/
  // this is what will hold the output of whatever outcomes/event happens (wifi success/failure)
	wifi_event_group = xEventGroupCreate();

  /* establish wifi event event handler if see WIFI_EVENT event of ESP_ANY_EVENT_ID 
  (any wifi event at all), then call wifi_event_handler. */
  esp_event_handler_instance_t wifi_handler_event_instance;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      &wifi_handler_event_instance));

  /* establish IP event event handler; if IP_EVENT_STA_GOT_IP (we, the STA=station, got an
  IP) then call ip_event_handler. the ip_event_handler gets called sometime after we've connected
  to wifi */
  esp_event_handler_instance_t got_ip_event_instance;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &ip_event_handler,
                                                      NULL,
                                                      &got_ip_event_instance));

  /** START THE WIFI DRIVER **/
  wifi_config_t wifi_config = {
      .sta = {
          .ssid = "chris' iphone 12",
          .password = "fatbitchfromthe6",
      .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          .pmf_cfg = {
              .capable = true,
              .required = false
          },
      },
  };

  // set the wifi controller to be a station
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );

  // set the wifi config
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );

  // start the wifi driver; event loop kicks off
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "STA initialization complete");

  /** NOW WE WAIT, blocks waiting for the wifi_event_group (event output)
   * to be filled. and it's gonna be filled with either WIFI_SUCCESS or WIFI_FAILURE.
   * so we're waiting for wifi_event_handler to call esp_wifi_connect, which will then call
   * ip_event_handler and if successful, this will return a WIFI_SUCCESS bc we've cncted to wifi **/
  EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
          WIFI_SUCCESS | WIFI_FAILURE,
          pdFALSE,
          pdFALSE,
          portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits (either WIFI_SUCCESS or WIFI_FAILURE) */
  if (bits & WIFI_SUCCESS) {
      ESP_LOGI(TAG, "Connected to ap");
      status = WIFI_SUCCESS;
  } else if (bits & WIFI_FAILURE) {
      ESP_LOGI(TAG, "Failed to connect to ap");
      status = WIFI_FAILURE;
  } else {
      ESP_LOGE(TAG, "UNEXPECTED EVENT");
      status = WIFI_FAILURE;
  }

  /* unregister the wifi & ip events b/c we've cncted already, don't want to keep trying to connect to more wifi */
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip_event_instance));
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler_event_instance));
  vEventGroupDelete(wifi_event_group); // delete the status container for the event loop

  return status;
}

// connect to the server and return the result
esp_err_t connect_tcp_server(void)
{
	struct sockaddr_in serverInfo = {0};
	char readBuffer[1024] = {0};

	serverInfo.sin_family = AF_INET;
	serverInfo.sin_addr.s_addr = 0x0100007f; // ip address
	serverInfo.sin_port = htons(12345); // hard-coded port


	int sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0)
	{
		ESP_LOGE(TAG, "Failed to create a socket..?");
		return TCP_FAILURE;
	}

  // connect to server using ip addr & port
	if (connect(sock, (struct sockaddr *)&serverInfo, sizeof(serverInfo)) != 0)
	{
		ESP_LOGE(TAG, "Failed to connect to %s!", inet_ntoa(serverInfo.sin_addr.s_addr));
		close(sock);
		return TCP_FAILURE;
	}

	ESP_LOGI(TAG, "Connected to TCP server.");
	bzero(readBuffer, sizeof(readBuffer));
    int r = read(sock, readBuffer, sizeof(readBuffer)-1);
    for(int i = 0; i < r; i++) {
        putchar(readBuffer[i]);
    }

    if (strcmp(readBuffer, "HELLO") == 0)
    {
    	ESP_LOGI(TAG, "WE DID IT!");
    }

    return TCP_SUCCESS;
}

/************ LLL WIFI END **************/

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA: // received data
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    // set up client config
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    // sets the event handler on any mqtt event
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    // these get called in wifi connection process
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    /** Connect to Wifi & get IP **/
    esp_err_t status = WIFI_FAILURE;

      // connect to wireless AP, kicks off all the stuff. returns WIFI_SUCCESS/FAILURE
	status = connect_wifi();
	if (WIFI_SUCCESS != status)
	{
    // we've failed > MAX_FAILURES times so just fail
		ESP_LOGI(TAG, "Failed to associate to AP, dying...");
		return;
	}
	
	status = connect_tcp_server();
	if (TCP_SUCCESS != status)
	{
		ESP_LOGI(TAG, "Failed to connect to remote server, dying...");
		return;
	}
    /** Start the MQTT **/
    mqtt_app_start();
}

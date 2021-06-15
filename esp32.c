/* ESP HTTP Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_tls.h"
#include "esp_http_client.h"

//includes for WiFi Station
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

//includes for UART
#include <unistd.h>
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#define BLINK_GPIO 2

//defines for WiFi Station
#define EXAMPLE_ESP_WIFI_SSID      "karel"
#define EXAMPLE_ESP_WIFI_PASS      "KarelNetwork"

#define EXAMPLE_ESP_MAXIMUM_RETRY  20
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

//statics for WiFi Station
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG_wifi = "wifi station";
static int s_retry_num = 0;

//defines for UART
#define TXD_PIN 17
#define RXD_PIN 16

//variables for UART
static const int RX_BUF_SIZE = 1024;
uint8_t temp_buf[30];
uint8_t hum_buf[30];
uint8_t freq_buf[30];
uint8_t mic_buf[30];
uint8_t ccs_buf[30];

//end of includes, defines and statics for WiFi Station

#define MAX_HTTP_RECV_BUFFER 512
static const char *TAG = "HTTP_CLIENT";

//IP and PORT numbers for HTTP connection
//These numbers must match with the IP of the receiver device that runs receiver.py
static const char *IP_ADDR = "192.168.225.49";
static const int PRT_NO = 8003;

/* Root cert for howsmyssl.com, taken from howsmyssl_com_root_cert.pem

   The PEM file was extracted from the output of this command:
   openssl s_client -showcerts -connect www.howsmyssl.com:443 </dev/null

   The CA root cert is the last cert given in the chain of certs.

   To embed it in the app binary, the PEM file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/
extern const char howsmyssl_com_root_cert_pem_start[] asm("_binary_howsmyssl_com_root_cert_pem_start");
extern const char howsmyssl_com_root_cert_pem_end[]   asm("_binary_howsmyssl_com_root_cert_pem_end");

void wifi_init(void);
void uart_init(void);
static void AHT_Humidity(uint8_t* data_aht);
static void AHT_Temperature(uint8_t* data_aht);
static void read_UART(uint8_t* data);
static void post_to_socket(char* data);


//FUNCTIONS FOR WIFI STATION
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_wifi, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG_wifi,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_wifi, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
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
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG_wifi, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_wifi, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_wifi, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG_wifi, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

void wifi_init(void){
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG_wifi, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
}
//END OF FUNCTIONS FOR WIFI STATION

//START OF UART FUNCTIONS
void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

    //AHT Data Handling
void AHT_Humidity(uint8_t* data_aht)   
{	
    uint32_t humid_c = data_aht[1];
    humid_c <<= 8;
    humid_c |= data_aht[2];
    humid_c <<= 4;
    humid_c |= data_aht[3] >> 4;

    sprintf((char*)hum_buf,
            "hum=%u.%u\n",
            ((int)humid_c * 100 / 0x100000),
            ((int)humid_c % 100));
}

void AHT_Temperature(uint8_t* data_aht)   
{	
    uint32_t temp_c = data_aht[3] & 0x0F;
    temp_c <<= 8;
    temp_c |= data_aht[4];
    temp_c <<= 8;
    temp_c |= data_aht[5];
    sprintf((char*)temp_buf, "temp=%u.%u\n", (((int)temp_c * 200 / 0x100000) - 50), ((int)temp_c % 100));
}

    //ADXL Data Handling
void ADXL_Frequency(uint8_t* data)
{
    uint8_t freq_data[4];

    for (int i = 6; i < 10; i++)
    {
        freq_data[i - 6] = data[i];
    }

    float freq = *(float *) & freq_data;
    sprintf((char*)freq_buf, "acc=%f\n", freq);
}

    //MAX Data Handling
void MAX_Mic(uint8_t* data)
{
    uint8_t mic_data[4];
    for (int i = 10; i < 14; i++)
    {
        mic_data[i - 10] = data[i];
    }

    uint32_t mic = mic_data[0] | (mic_data[1] << 8) | (mic_data[2] << 16) | (mic_data[3] << 24);
    sprintf((char*)mic_buf, "mic=%u\n", mic);
}

    //CCS Data Handling
void CCS_data(uint8_t* data)
{
    uint8_t data_ccs[4];
    
    for (int i = 14; i < 18; i++)
    {
        data_ccs[i - 14] = data[i];
    }
    
    uint16_t eCO2 = ((uint16_t)data_ccs[0] << 8) | ((uint16_t)data_ccs[1]);
    uint16_t TVOC = ((uint16_t)data_ccs[2] << 8) | ((uint16_t)data_ccs[3]);

    sprintf((char*)ccs_buf, "co2=%u\ntvoc=%u\n", eCO2, TVOC);
}

void read_UART(uint8_t* data)
{
    int size = 0;
    uart_flush(UART_NUM_1);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);

    if (rxBytes > 0) {
        data[rxBytes] = 0;    
        ESP_LOG_BUFFER_HEXDUMP("read: ", data, rxBytes, ESP_LOG_INFO);
        AHT_Humidity(data);
        AHT_Temperature(data); 
        ADXL_Frequency(data); 
        MAX_Mic(data);
        CCS_data(data);
    }  
}
//END OF UART FUNCTIONS

//FUNCTIONS FOR HTTP CONNECTION
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write out data
                // printf("%.*s", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            break;
    }
    return ESP_OK;
}

void post_to_socket(char* data){
                
        esp_http_client_config_t config = {
            .host = IP_ADDR,
            .port = PRT_NO,
            .path = "/sensordata",
            .event_handler = _http_event_handler,
        };

        esp_http_client_handle_t client = esp_http_client_init(&config);
        esp_http_client_set_post_field(client, data, strlen(data));
        esp_http_client_set_method(client, HTTP_METHOD_POST);
        esp_err_t err = esp_http_client_perform(client);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                    esp_http_client_get_status_code(client),
                    esp_http_client_get_content_length(client));
        } else {
            ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
        }
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
}
//END OF FUNCTIONS FOR HTTP Connection

void app_main(void)
{
    //Initialize NVS
    wifi_init();
    uart_init();

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    

    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    sleep(1);
    while(1)
    {    
        
        //read data from UART    
        read_UART(data);
        
        //Read counter value
        time_t rawtime;
        struct tm * timeinfo;

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );

        //RSSI data
        wifi_ap_record_t wifidata;
        esp_wifi_sta_get_ap_info(&wifidata);
    
        //Concatenate 
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        char postdata[200];
        sprintf(postdata, "%s%s%s%s%s%s%d\n%s%s%s", 
                            "AP=3\n", "time=", asctime(timeinfo), 
                            temp_buf, 
                            hum_buf, 
                            "rssi=", wifidata.rssi,
                            freq_buf,
                            mic_buf,
                            ccs_buf); // puts string into buffer
        printf(postdata);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        post_to_socket(postdata);
    }
}

#include <time.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_modem_api.h"
#include "esp_sntp.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

/*defines*/
#define GSM_APN                 "ntnet"
#define GSM_UART_PORT           UART_NUM_1 /*router over GPIO43/44 via GPIO matrix*/
#define GSM_UART_TX_PIN         43 /*COM_UART_RX : EG91 RXD */
#define GSM_UART_RX_PIN         44 /*COM_UART_TX : EG91 TXD */
#define GSM_UART_RTS_PIN        -1 
#define GSM_UART_CTS_PIN        -1 
#define GSM_MODEM_BAUD          115200
#define GSM_PWRKEY_PIN          38 /*Q1 (2N7002) gate */
#define GSM_PWRKEY_ACTIVE_LOW   0 /* gate HIGH -> Q1 pulls EG91 PWRKEY LOW */
#define GSM_SIM_SLOT            0 /* SIM tray: 0 = CN7 (module default) | 1 = CN8 (AT+QDSIM sent)*/

/*states*/
static const char *TAG = "GSM";

static EventGroupHandle_t s_conn_event_group;
#define GSM_CONNECTED_BIT       BIT0
#define GSM_LOST_BIT            BIT1

static esp_netif_t *s_ppp_netif;
static esp_modem_dce_t *s_dce;


/*helpers*/
static void gsm_modem_power_on(void)
{
    gpio_config_t io_conf = 
    {
        .pin_bit_mask = (1ULL << GSM_PWRKEY_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };

    gpio_config(&io_conf);

    gpio_set_level(GSM_PWRKEY_PIN, 0); /*idle: Q1 off*/
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(GSM_PWRKEY_PIN, 1); /*800ms: past EG91's ~500ms on and ~650ms off thresholds either way - unambiguous toggle, not just a power-on pulse*/
    vTaskDelay(pdMS_TO_TICKS(800));
    gpio_set_level(GSM_PWRKEY_PIN, 0); /*idle: Q1 off*/


    ESP_LOGI(TAG, "PWRKEY pulsed, waiting for modem to boot...");
    vTaskDelay(pdMS_TO_TICKS(5000)); 
}

static void gsm_ip_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if(id==IP_EVENT_PPP_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "PPP got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupClearBits(s_conn_event_group, GSM_LOST_BIT);
        xEventGroupSetBits(s_conn_event_group, GSM_CONNECTED_BIT);
    }
    else if(id==IP_EVENT_PPP_LOST_IP)
    {
        ESP_LOGI(TAG, "PPP disconnected");
         xEventGroupClearBits(s_conn_event_group, GSM_CONNECTED_BIT);
        xEventGroupSetBits(s_conn_event_group, GSM_LOST_BIT);
    }
}


/*auto-redial task*/
/*Cellular drops far more than wifi so this is what keeps the gateway posting without a manual reboot*/
static void gsm_link_monitor_task(void *arg)
{
    while(1)
    {
        xEventGroupWaitBits(s_conn_event_group, GSM_LOST_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Redialing...");
        esp_modem_set_mode(s_dce, ESP_MODEM_MODE_COMMAND);
        vTaskDelay(pdMS_TO_TICKS(2000)); /*wait for modem to settle*/
        esp_err_t err = esp_modem_set_mode(s_dce, ESP_MODEM_MODE_DATA);
        if(err!=ESP_OK)
        {
            ESP_LOGE(TAG, "Redial failed: %s, will retry", esp_err_to_name(err));
            xEventGroupSetBits(s_conn_event_group, GSM_LOST_BIT);
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

static void gsm_sntp_sync_time(void)
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    time_t now = 0;
    int retry = 0;
    const int retry_count = 15;

    while(now<1700000000 && ++retry<retry_count){
        vTaskDelay(pdMS_TO_TICKS(1000));
        time(&now);
    }

    if (now < 1700000000) ESP_LOGE(TAG, "SNTP sync failed, timestamps will be wrong");
    else ESP_LOGI(TAG, "Time synced: %lld", (long long)now);
}

void gsm_conn_init(void)
{
    s_conn_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, &gsm_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_LOST_IP, &gsm_ip_event_handler, NULL));

    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();
    s_ppp_netif = esp_netif_new(&netif_ppp_config);
    assert(s_ppp_netif); //halt if false :: diagnose


    /*DTE/DCE config and bringup*/
    /*DTE = ESP32 (dte_config), DCE = MODEM EG91 (dce_config)*/
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(GSM_APN);
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_config.uart_config.port_num = GSM_UART_PORT;
    dte_config.uart_config.baud_rate = GSM_MODEM_BAUD;
    dte_config.uart_config.tx_io_num = GSM_UART_TX_PIN;
    dte_config.uart_config.rx_io_num = GSM_UART_RX_PIN;
    dte_config.uart_config.rts_io_num = GSM_UART_RTS_PIN;
    dte_config.uart_config.cts_io_num = GSM_UART_CTS_PIN;

    gsm_modem_power_on();

    for(int attempt = 1; ; attempt++)
    {
        ESP_LOGI(TAG, "Modem init attempt %d", attempt);
        s_dce = esp_modem_new(&dte_config, &dce_config, s_ppp_netif);
        if(s_dce!=NULL) break;
        ESP_LOGW(TAG, "Modem not responding, re-pulsing PWRKEY and retrying...");
        gsm_modem_power_on();
    }

    /* AT sync, escalating: plain sync -> force COMMAND mode and retry -> as a
       last resort, toggle PWRKEY (a power-on pulse alone is a no-op on a
       modem that's already running but stuck) and start the cycle over. */
    esp_err_t sync_err = ESP_FAIL;
    for(int cycle = 1; cycle <= 3; cycle++)
    {
        sync_err = esp_modem_sync(s_dce);
        if(sync_err==ESP_OK) break;

        esp_modem_set_mode(s_dce, ESP_MODEM_MODE_COMMAND);
        sync_err = esp_modem_sync(s_dce);
        if(sync_err==ESP_OK) break;

        ESP_LOGW(TAG, "Modem unresponsive (cycle %d), toggling PWRKEY", cycle);
        gsm_modem_power_on();
    }
    if(sync_err!=ESP_OK)
    {
        /* a full ESP32 reboot has, empirically, so last way */
        ESP_LOGE(TAG, "Modem still not responding after PWRKEY toggling, restarting");
        esp_restart();
    }
    esp_modem_at(s_dce, "ATH\r", NULL, 2000); /* hang up any half-open call left over from a prior session */

#if GSM_SIM_SLOT != 0
    esp_modem_at(s_dce, "AT+QDSIM=1\r", NULL, 5000); /* select SIM slot */
    vTaskDelay(pdMS_TO_TICKS(2000)); /* module reinitializes its SIM interface after this - unresponsive until it settles */
#endif

    esp_err_t echo_err = esp_modem_set_echo(s_dce, false); /* echo was polluting the response parser */
    if(echo_err!=ESP_OK)
    {
        ESP_LOGW(TAG, "Disabling echo failed: %s", esp_err_to_name(echo_err));
    }
    vTaskDelay(pdMS_TO_TICKS(500));   /* let the parser settle before the next AT exchange */

    /* Bounded retry to actually get an IP */
#define GSM_CONNECT_TIMEOUT_MS  30000

    for(int connect_attempt = 1; ; connect_attempt++)
    {
        int rssi, ber;
        esp_err_t csq_err = esp_modem_get_signal_quality(s_dce, &rssi, &ber);
        if(csq_err==ESP_OK)
        {
            ESP_LOGI(TAG, "Modem signal quality: RSSI=%d dBm, BER=%d", rssi, ber);
        }
        else
        {
            ESP_LOGW(TAG, "esp_modem_get_signal_quality failed: %s", esp_err_to_name(csq_err));
        }

        esp_err_t err = esp_modem_set_mode(s_dce, ESP_MODEM_MODE_DATA);
        if(err!=ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set modem to DATA mode: %s", esp_err_to_name(err));
        }

        ESP_LOGI(TAG, "Waiting for PPP IP address (connect attempt %d)...", connect_attempt);
        EventBits_t bits = xEventGroupWaitBits(s_conn_event_group, GSM_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(GSM_CONNECT_TIMEOUT_MS));
        if(bits & GSM_CONNECTED_BIT)
        {
            break;   /* got an IP, fall through to starting the monitor/sntp */
        }

        ESP_LOGW(TAG, "No IP after %dms, returning to command mode and redialing", GSM_CONNECT_TIMEOUT_MS);
        esp_modem_set_mode(s_dce, ESP_MODEM_MODE_COMMAND);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    xTaskCreatePinnedToCore(gsm_link_monitor_task, "gsm_link_monitor_task", 4096, NULL, 4, NULL, 1);

    gsm_sntp_sync_time();
}
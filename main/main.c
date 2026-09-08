#include "main.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "nimble/hci_common.h"
#include "services/gap/ble_svc_gap.h"
#include "nvs_flash.h"
#include "nimble/ble.h"
#include "gap_scan.h"
#include "server_comm.h"
#include "gatt_client.h"
#include "wifi_conn.h"
#include "seedlink.h"
#include "wifi_conn.h"
#include "gsm_conn.h"
#include "seedlink.h"

#define TAG "BLE_APP"

/*
 * ble_hs_id_set_rnd() takes the address in host (little-endian) order,
 * Displayed MSB-first this is C0:52:54:53:4C:47.
 *   - 0xC0: top two bits must be 11 for a valid static-random address
 *     (BLE spec)
 *   - 52:54:53:4C:47 spells "RTSLG" in ASCII (RTS Landslide Gateway).
 */
static const uint8_t s_fixed_bd_addr[6] = { 0x47, 0x4C, 0x53, 0x54, 0x52, 0xC0 };

/* own_addr_type as determined in ble_app_on_sync_cb, reused so a GATT
   session can resume scanning once it ends */
static uint8_t s_own_addr_type;

/* prototypes */
static void on_gatt_session_end(void);

void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
}

void ble_app_on_reset_cb(int reason)
{
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

void ble_app_on_sync_cb(void)
{
    ESP_LOGI(TAG, "BLE synchronized");

    int rc = ble_hs_id_set_rnd(s_fixed_bd_addr);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set fixed BD address: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "This device's BD address set: %02x:%02x:%02x:%02x:%02x:%02x",
             s_fixed_bd_addr[5], s_fixed_bd_addr[4], s_fixed_bd_addr[3],
             s_fixed_bd_addr[2], s_fixed_bd_addr[1], s_fixed_bd_addr[0]);

    s_own_addr_type = BLE_OWN_ADDR_RANDOM;

    seedlink_init();  
    server_comm_init();

    gatt_client_init(s_own_addr_type);
    gatt_client_set_session_end_cb(on_gatt_session_end);

    gap_scan_start(s_own_addr_type);
}

static void on_gatt_session_end(void)
{
    gap_scan_start(s_own_addr_type);
}

void app_main(void)
{
    esp_log_level_set("esp-x509-crt-bundle", ESP_LOG_WARN);
    esp_log_level_set("esp_http_client", ESP_LOG_WARN);
    esp_log_level_set("mbedtls", ESP_LOG_WARN);
    esp_log_level_set("TRANS_TCP", ESP_LOG_WARN);

    //ble controller reads calibration data from nvs on every boot
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS and retrying");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

#if USE_WIFI_INSTEAD_OF_GSM
    wifi_conn_init();
#else
    gsm_conn_init();
#endif

    esp_err_t ret = nimble_port_init();
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return;
    }

    //configuring the BLE host
    ble_hs_cfg.reset_cb = ble_app_on_reset_cb;
    ble_hs_cfg.sync_cb = ble_app_on_sync_cb;

    //setting device name
    ble_svc_gap_device_name_set("S3-Gateway-LEWS");

    nimble_port_freertos_init(ble_host_task);

}

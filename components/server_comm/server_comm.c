#include "server_comm.h"
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "seedlink.h"

static const char *TAG = "SERVER_COMM";

#define TSS_IMPORT_URL      "https://alpha.wscada.net/import"
#define TSS_ORIGIN_CODE_DEFAULT      "600"

#define TSS_PARAM_ACCEL_X    "ACC_X"
#define TSS_PARAM_ACCEL_Y    "ACC_Y"
#define TSS_PARAM_ACCEL_Z    "ACC_Z"

#define TSS_PARAM_GYRO_X      "GYRO_X"
#define TSS_PARAM_GYRO_Y      "GYRO_Y"
#define TSS_PARAM_GYRO_Z      "GYRO_Z"

#define TSS_PARAM_DEVIATION    "Base_Ang_Dev"
#define TSS_PARAM_RATE         "rate_deg_per_h"
#define TSS_PARAM_STATUS       "LS_ALARM"
#define TSS_PARAM_TRIGGER      "ALARM_T"



static const sensor_id_t s_known_sensors[] = {
    { {0x34, 0x12, 0x2A, 0xE1, 0x08, 0x00}, "DMG37", "600" },   // node1: CFG_PUBLIC_BD_ADDRESS = 0x0008E12A1234
    { {0x35, 0x12, 0x2A, 0xE1, 0x08, 0x00}, "DMG38", "601" }, 
    //{ {0x36, 0x12, 0x2A, 0xE1, 0x08, 0x00}, "DMG39", "602" },
    /* add one row per deployed sensor - fill all needed */
};
#define NUM_KNOWN_SENSORS (sizeof(s_known_sensors) / sizeof(s_known_sensors[0]))

size_t server_comm_known_sensor_count(void)
{
    return NUM_KNOWN_SENSORS;
}

const sensor_id_t *server_comm_sensor_id_lookup(const uint8_t addr[6])
{
    for (size_t i = 0; i < NUM_KNOWN_SENSORS; i++) {
        if (memcmp(s_known_sensors[i].addr, addr, 6) == 0) {
            return &s_known_sensors[i];
        }
    }
    return NULL;
}


bool hide_gatt_logs = false;  

bool g_gatt_connect_requested = true;

bool training_mode = false; /* for turning on seedlink server and raw data logs */

bool restart_sensor_node = true; 

/* TSS post (esp_http_client + TLS) must never run on the nimble_host task, producer (BLE callback) just enqueues, a dedicated
   task with its own stack does the actual blocking network I/O. */
typedef struct {
    const char *origin_code;
    uint8_t status, trigger;
    uint16_t dev_x100, vel_x100;
    int16_t ax, ay, az, gx, gy, gz;
} tss_snapshot_t;

static QueueHandle_t s_tss_queue = NULL;

static void tss_post_task(void *arg);

#define TSS_RESP_BUF_SIZE 512
static char s_tss_resp_buf[TSS_RESP_BUF_SIZE];
static int s_tss_resp_len;

static esp_err_t tss_http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA && !esp_http_client_is_chunked_response(evt->client)) {
        int copy_len = evt->data_len;
        if (s_tss_resp_len + copy_len >= TSS_RESP_BUF_SIZE) {
            copy_len = TSS_RESP_BUF_SIZE - 1 - s_tss_resp_len;
        }
        if (copy_len > 0) {
            memcpy(s_tss_resp_buf + s_tss_resp_len, evt->data, copy_len);
            s_tss_resp_len += copy_len;
            s_tss_resp_buf[s_tss_resp_len] = '\0';
        }
    }
    return ESP_OK;
}

void server_comm_init(void)
{
    /*
     * TODO(server): call server_comm_handle_command() with the command string whenever
     * one arrives.
     */
    s_tss_queue = xQueueCreate(4, sizeof(tss_snapshot_t));
    if (s_tss_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create TSS post queue");
        return;
    }
    if (xTaskCreatePinnedToCore(tss_post_task, "tss_post", 8192, NULL, 5, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TSS post task");
    }
}

void server_comm_handle_command(const char *cmd_str)
{
    if (cmd_str == NULL) {
        return;
    }
    if (strcmp(cmd_str, "GATT_Connect") == 0) {
        g_gatt_connect_requested = true;
    }
}

void server_comm_log_json(const char *json_str)
{
    ESP_LOGI(TAG, "\n [gateway -> server] %s \n", json_str);
}

static void add_observation(cJSON *arr, const char *origin_code, const char *param_code, const char *time_str, int value)
{
    cJSON *obs = cJSON_CreateObject();
    cJSON_AddStringToObject(obs, "origin_code", origin_code);
    cJSON_AddStringToObject(obs, "parameter_code", param_code);
    cJSON_AddStringToObject(obs, "time", time_str);
    cJSON_AddNumberToObject(obs, "value", value);
    cJSON_AddItemToArray(arr, obs);
}

/* Per-axis tilt angle in degrees, matching the "Degrees" unit already set on
   the Accelerometer_X/Y/Z parameters in TSS */
static int accel_lsb_to_deg(int16_t raw)
{
    float g = (float)raw * ACCEL_G_PER_LSB;
    if (g > 1.0f)  g = 1.0f;
    if (g < -1.0f) g = -1.0f;
    float deg = asinf(g) * (180.0f / (float)M_PI);
    return (int)lroundf(deg);
}

/* dps, rounded to the nearest whole degree/s */
static int gyro_lsb_to_dps(int16_t raw)
{
    float dps = (float)raw * GYRO_DPS_PER_LSB;
    return (int)lroundf(dps);
}

void server_comm_post_snapshot(const char *origin_code, uint8_t status, uint8_t trigger,
                                uint16_t dev_x100, uint16_t vel_x100,
                                int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz)
{
    if (s_tss_queue == NULL)
    {
        ESP_LOGW(TAG, "TSS post skipped: server_comm_init() not called yet");
        return;
    }

    tss_snapshot_t item =
    {
        .origin_code = origin_code,
        .status = status, .trigger = trigger,
        .dev_x100 = dev_x100, .vel_x100 = vel_x100,
        .ax = ax, .ay = ay, .az = az,
        .gx = gx, .gy = gy, .gz = gz,
    };

    if (xQueueSend(s_tss_queue, &item, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "TSS post skipped: queue full");
    }
}

void seedlink_send(imu_payload_t *payload, uint16_t *idx, uint32_t *sequence,
                                const sensor_id_t *sensor_id,
                                int16_t accel_x, int16_t accel_y, int16_t accel_z)
{
    if (*idx == 0)
    {
        /* Nepal Standard Time = UTC+5:45 - mseed_record() runs gmtime() on
           this value, so shifting it here keeps the SeedLink/SeisComP side
           showing correct local time. */
        payload->timestamp = time(NULL) + (5 * 3600 + 45 * 60);
        payload->sequence_number = (*sequence)++;
        strlcpy(payload->station, sensor_id ? sensor_id->station : "DMG37", sizeof(payload->station));
    }

    payload->ax[*idx] = accel_x / 16384.0f;
    payload->ay[*idx] = accel_y / 16384.0f;
    payload->az[*idx] = accel_z / 16384.0f;
    (*idx)++;

    if (*idx >= IMU_MAX_SAMPLES)
    {
        xQueueSend(seedlink_get_queue(), payload, 0);
        *idx = 0;
    }
}

static void tss_post_task(void *arg)
{
    (void)arg;
    tss_snapshot_t s;

    esp_http_client_config_t config = {
        .url = TSS_IMPORT_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = tss_http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "TSS post: http client init failed, task exiting");
        vTaskDelete(NULL);
        return;
    }
    esp_http_client_set_header(client, "Content-Type", "application/json");

    while (1) 
    {
        if (xQueueReceive(s_tss_queue, &s, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        int16_t ax = s.ax, ay = s.ay, az = s.az;
        int16_t gx = s.gx, gy = s.gy, gz = s.gz;
        uint16_t dev_x100 = s.dev_x100, vel_x100 = s.vel_x100;
        uint8_t status = s.status, trigger = s.trigger;
        const char *origin_code = s.origin_code ? s.origin_code : TSS_ORIGIN_CODE_DEFAULT;

        time_t now = time(NULL) + (5 * 3600 + 45 * 60); /* Nepal Standard Time = UTC+5:45 */
        struct tm tm_utc;
        gmtime_r(&now, &tm_utc);
        char time_str[24];
        strftime(time_str, sizeof(time_str), "%Y-%m-%dT%H:%M:%S", &tm_utc);

        cJSON *arr = cJSON_CreateArray();
        add_observation(arr, origin_code, TSS_PARAM_ACCEL_X, time_str, accel_lsb_to_deg(ax));
        add_observation(arr, origin_code, TSS_PARAM_ACCEL_Y, time_str, accel_lsb_to_deg(ay));
        add_observation(arr, origin_code, TSS_PARAM_ACCEL_Z, time_str, accel_lsb_to_deg(az));
        add_observation(arr, origin_code, TSS_PARAM_GYRO_X, time_str, gyro_lsb_to_dps(gx));
        add_observation(arr, origin_code, TSS_PARAM_GYRO_Y, time_str, gyro_lsb_to_dps(gy));
        add_observation(arr, origin_code, TSS_PARAM_GYRO_Z, time_str, gyro_lsb_to_dps(gz));
        add_observation(arr, origin_code, TSS_PARAM_DEVIATION, time_str, (int)lroundf((float)dev_x100 / 100.0f));
        add_observation(arr, origin_code, TSS_PARAM_RATE, time_str, (int)lroundf((float)vel_x100 / 100.0f));
        add_observation(arr, origin_code, TSS_PARAM_STATUS, time_str, (int)status);
        add_observation(arr, origin_code, TSS_PARAM_TRIGGER, time_str, (int)trigger);

        char *body = cJSON_PrintUnformatted(arr);
        cJSON_Delete(arr);
        if (body == NULL) {
            continue;
        }

        esp_http_client_set_post_field(client, body, (int)strlen(body));

        ESP_LOGI(TAG, "%s  request=%s", TSS_IMPORT_URL, body);

        s_tss_resp_len = 0;
        s_tss_resp_buf[0] = '\0';

        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            int http_status = esp_http_client_get_status_code(client);
            ESP_LOGI(TAG, "TSS post: status=%d  response=%s", http_status,
                     s_tss_resp_len > 0 ? s_tss_resp_buf : "(empty)");
        } else {
            ESP_LOGW(TAG, "TSS post failed: %s", esp_err_to_name(err));
        }

        esp_http_client_close(client);

        cJSON_free(body);
    }
}

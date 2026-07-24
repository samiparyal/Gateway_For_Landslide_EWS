/*****************************************************************************************************************
 * seedlink.c
 *
 * Created on: July 16, 2025
 *     Author: Ravi Tamrakar
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_check.h"
#include "esp_board.h"

#include "system.h"
#include "setting.h"
#include "cm.h"
#include "seedlink.h"
#include "geophone.h"
#include "debug.h"
#include "tcp_client.h"
#include "sdcard.h"

#include "mseed.h"

/*****************************************************************************************************************
 * MACROS AND DEFINES
 */

/*****************************************************************************************************************
 * EXTERN PUBLIC VARIABLES
 */

/*****************************************************************************************************************
 * EXTERN PUBLIC FUNCTION DECLARATIONS
 */

void sensor_wait_for_data(bool wait);
struct tm *app_get_current_time(void);

/*****************************************************************************************************************
 * PRIVATE STRUCTURES
 */

enum
{
    FILE_READ,
    FILE_WRITE
};

typedef struct DATA_FILE_HEADER
{
    size_t read;
    size_t write;
} data_file_header_t;

typedef struct DATA_FILE
{
    data_file_header_t s_header;
    char path[32];
} data_file_t;

typedef struct SEEDLINK
{
    TaskHandle_t task;
    TaskHandle_t task_connect;
    TaskHandle_t task_recovery;
    QueueHandle_t queue;
    SemaphoreHandle_t lock;
    SemaphoreHandle_t sem_connect;
    data_file_t s_file[2];
    uint32_t cm_bit_val;
    void *vp_cm_handle;
    void *vp_conn;
    bool recover;
} seedlink_t;

/*****************************************************************************************************************
 * PRIVATE FUNCTION DECLARATIONS
 */

static void _task(void *vp_arg);
static void _task_connect(void *vp_arg);

static esp_err_t _header_init(void);

/*****************************************************************************************************************
 * PRIVATE VARIABLES
 */

static const char *LOG_TAG = "[seedlink]";
static const size_t g_size_payload = sizeof(geophone_payload_t);
static const size_t g_size_header = sizeof(data_file_header_t);
static seedlink_t g_s_self = {0};

/*****************************************************************************************************************
 * PRIVATE INLINE FUNCTION DEFINITIONS
 */

static inline char *_timestamp(uint64_t epoch_time, char *buffer, size_t buffer_size)
{
    time_t raw_time = (time_t)epoch_time;
    struct tm s_tm = {0};
    localtime_r(&raw_time, &s_tm);
    strftime(buffer, buffer_size, "%Y-%m-%d %H:%M:%S", &s_tm);
    return buffer;
}

static inline int _get_path(char **p_path)
{
    struct tm *sp_tm = app_get_current_time();
    int year = sp_tm->tm_year + 1900;
    int month = sp_tm->tm_mon + 1;
    int day = sp_tm->tm_mday;

    return asprintf(p_path, "/%04d/%02d/%02d.bin", year, month, day);
}

static inline esp_err_t _create_write_file(char *path)
{
    data_file_t *sp_file = &g_s_self.s_file[FILE_WRITE];
    data_file_header_t *sp_header = &sp_file->s_header;

    snprintf(sp_file->path, sizeof(sp_file->path), "%s", path);
    free(path);

    sp_header->read = 0;
    sp_header->write = 0;

    return sdcard_overwrite_file(sp_file->path, (uint8_t *)sp_header, g_size_header);
}

/*****************************************************************************************************************
 * PUBLIC FUNCTION DEFINITIONS
 */

esp_err_t seedlink_init(void)
{
    g_s_self.queue = xQueueCreateWithCaps(3600, sizeof(geophone_payload_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(g_s_self.queue, ESP_ERR_NO_MEM, LOG_TAG, "Failed to create geophone payload queue");

    g_s_self.lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(g_s_self.lock, ESP_ERR_NO_MEM, LOG_TAG, "Failed to create seedlink lock");

    g_s_self.sem_connect = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(g_s_self.sem_connect, ESP_ERR_NO_MEM, LOG_TAG, "Failed to create seedlink connect semaphore");

    ESP_RETURN_ON_ERROR(_header_init(), LOG_TAG, "Failed to initialize header for WRITE file");

    BaseType_t xRet = xTaskCreatePinnedToCore(_task, "seedlink", SYSTEM_STACK_SIZE(8), NULL, 5, &g_s_self.task, 1);
    ESP_RETURN_ON_FALSE(pdPASS == xRet, ESP_ERR_NO_MEM, LOG_TAG, "Failed to create seedlink task");

    return ESP_OK;
}

QueueHandle_t seedlink_get_queue(void)
{
    return g_s_self.queue;
}

uint32_t seedlink_get_stored(void)
{
    return uxQueueMessagesWaiting(g_s_self.queue);
}

/*****************************************************************************************************************
 * PRIVATE FUNCTION DEFINITIONS
 */

static esp_err_t _header_update(const char *info, const char *path, data_file_header_t *sp_header)
{
    esp_err_t ret = sdcard_write_to_file(path, (uint8_t *)sp_header, g_size_header, 0);

    if (ret == ESP_OK)
    {
        printf("%s: <read: %ld, write: %ld>\n",
               info, sp_header->read, sp_header->write);
        // ESP_LOGW(LOG_TAG, "%s: <read: %ld, write: %ld>",
        //          info, sp_header->read, sp_header->write);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Failed to update header: %s", path);
    }

    return ret;
}

static esp_err_t _header_load(const char *path, data_file_header_t *sp_header)
{
    esp_err_t ret = sdcard_read_from_file(path, (uint8_t *)sp_header, g_size_header, 0);

    if (ret == ESP_ERR_INVALID_SIZE)
    {
        if (ESP_OK == sdcard_delete_file(path))
        {
            ESP_LOGI(LOG_TAG, "Deleted invalid file: %s", path);
        }

        ret = ESP_ERR_NOT_FOUND;
    }

    return ret;
}

static esp_err_t _header_init(void)
{
    esp_err_t ret = ESP_OK;

    if (board_is_sd_card_inserted())
    {
        // #warning "formatting sdcard for test"
        // sdcard_format();
        sdcard_lock(portMAX_DELAY);

        /* Load file path for current date */
        char *path = NULL;
        int n = _get_path(&path);

        if (n < 0)
        {
            ESP_LOGE(LOG_TAG, "Failed to allocate memory for file path");
            sdcard_unlock();
            free(path);
            return ESP_ERR_NO_MEM;
        }

        /* Check if the file exists */
        long filesize = sdcard_get_filesize(path);

        if (filesize < 0)
        {
            ESP_LOGE(LOG_TAG, "Failed doesn't exist: %s", path);
            /* Create file if it doesn't exist */
            ret = _create_write_file(path);
        }
        else
        {
            data_file_t *sp_file = &g_s_self.s_file[FILE_WRITE];
            snprintf(sp_file->path, sizeof(sp_file->path), "%s", path);
            free(path);
        }

        sdcard_unlock();
    }

    return ret;
}

static esp_err_t _data_read(data_file_t *sp_file, uint8_t *p_data)
{
    data_file_header_t *sp_header = &sp_file->s_header;
    char *path = sp_file->path;

    size_t offset = g_size_header + (g_size_payload * sp_header->read);
    return sdcard_read_from_file(path, p_data, g_size_payload, offset);
}

static esp_err_t _data_write(data_file_t *sp_file, uint8_t *p_data)
{
    data_file_header_t *sp_header = &sp_file->s_header;
    char *path = sp_file->path;

    size_t offset = g_size_header + (g_size_payload * sp_header->write);
    return sdcard_write_to_file(path, p_data, g_size_payload, offset);
}

static uint8_t *_seed_create(float *p_data, mseed_config_t *sp_cfg, const char *channel)
{
    snprintf(sp_cfg->channel, sizeof(sp_cfg->channel), "%s", channel);
    uint8_t *p_record = mseed_record(p_data, sp_cfg);
    uint8_t *p_datalink = mseed_datalink(p_record, sp_cfg);
    return p_datalink;
}

static uint32_t _pkt_create(geophone_payload_t *sp_payload, uint8_t *p_buffer, uint32_t buffer_size)
{
    seedlink_info_t *sp_seedlink = &setting_get_device_info()->s_seedlink;

    if (!(strlen(sp_seedlink->station) > 0 && strlen(sp_seedlink->location) > 0 && strlen(sp_seedlink->network) > 0))
    {
        ESP_LOGE(LOG_TAG, "Seedlink station, location, or network not configured properly");
        return 0;
    }

    mseed_config_t s_cfg = {0};
    s_cfg.sample_count = GEOPHONE_MAX_SAMPLES;
    s_cfg.sequence_number = sp_payload->sequence_number;
    strlcpy(s_cfg.station, sp_seedlink->station, sizeof(s_cfg.station));
    strlcpy(s_cfg.location, sp_seedlink->location, sizeof(s_cfg.location));
    strlcpy(s_cfg.network, sp_seedlink->network, sizeof(s_cfg.network));
    s_cfg.start_time = sp_payload->timestamp;
    s_cfg.time_fract = 0;

    const uint32_t seed_size = 583;
    uint8_t *p_seed = NULL;

    struct channel
    {
        const char *name;
        float *data;
    } channels[] = {
        {"HNN", sp_payload->hnn},
        {"HNZ", sp_payload->hnz},
        {"HNE", sp_payload->hne},
    };

    uint8_t channel_count = sizeof(channels) / sizeof(channels[0]);

    for (int i = 0; i < channel_count; i++)
    {
        p_seed = _seed_create(channels[i].data, &s_cfg, channels[i].name);
        memcpy(p_buffer + (i * seed_size), p_seed, seed_size);
    }

    return seed_size * channel_count;
}

static esp_err_t _payload_send(geophone_payload_t *sp_payload)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t buffer[2048];

    uint32_t size = _pkt_create(sp_payload, buffer, sizeof(buffer));

    if (size)
    {
        if (ERR_OK != tcp_client_send(g_s_self.vp_conn, buffer, size))
        {
            tcp_client_deinit(g_s_self.vp_conn);
            g_s_self.vp_conn = NULL;

            if (g_s_self.vp_cm_handle)
            {
                cm_trigger_done(g_s_self.vp_cm_handle, g_s_self.cm_bit_val, FEATURE_DATA, COMM_ERROR_NET_DOWN);
                g_s_self.vp_cm_handle = NULL;
                g_s_self.cm_bit_val = 0;
            }

            xSemaphoreTake(g_s_self.sem_connect, portMAX_DELAY);
        }
        else
        {
            ret = ESP_OK;
        }
    }

    return ret;
}

static esp_err_t _payload_recover(geophone_payload_t *sp_payload)
{
    esp_err_t ret = ESP_FAIL;

    if (board_is_sd_card_inserted())
    {
        sdcard_lock(portMAX_DELAY);
        data_file_t *sp_file = &g_s_self.s_file[FILE_READ];
        data_file_header_t *sp_header = &sp_file->s_header;

        if (strlen(sp_file->path) == 0)
        {
            char *path = NULL;

            /* Find a data file for recovery */
            ret = sdcard_find_file(NULL, ".bin", &path);

            if (ret != ESP_OK)
            {
                ESP_LOGE(LOG_TAG, "Failed to find data file for recovery");
                sdcard_unlock();
                return ESP_ERR_NOT_FOUND;
            }

            snprintf(sp_file->path, sizeof(sp_file->path), "%s", path);
            free(path);
        }

        /* Load READ file header */
        data_file_header_t s_header = {0};
        ret = _header_load(sp_file->path, &s_header);

        if (ret != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "Failed to load header for recovery: %s", sp_file->path);
            sdcard_unlock();
            return ESP_ERR_NOT_FOUND;
        }
        else
        {
            if (sp_header->read < s_header.read)
            {
                sp_header->read = s_header.read;
            }

            sp_header->write = s_header.write;
        }

        /* Check if there are payloads to recover */
        if (sp_header->read < sp_header->write)
        {
            ret = _data_read(sp_file, (uint8_t *)sp_payload);
        }
        else
        {
            char *path = g_s_self.s_file[FILE_WRITE].path;

            /* Don't delete the file if it's the current WRITE file */
            if (strlen(path) && strcmp(path, sp_file->path) == 0)
            {
                sdcard_unlock();
                return ESP_ERR_NOT_ALLOWED;
            }
            else
            {
                ESP_LOGW(LOG_TAG, "No payloads to recover in file: %s, deleting file", sp_file->path);

                if (ESP_OK == sdcard_delete_file(sp_file->path))
                {
                    ESP_LOGI(LOG_TAG, "Deleted file: %s", sp_file->path);
                }

                memset(sp_file, 0, sizeof(data_file_t));
                ret = ESP_ERR_NOT_FINISHED;
            }
        }

        sdcard_unlock();
    }

    return ret;
}

static esp_err_t _payload_save(geophone_payload_t *sp_payload)
{
    esp_err_t ret = ESP_FAIL;

    if (board_is_sd_card_inserted())
    {
        sdcard_lock(portMAX_DELAY);
        data_file_t *sp_file = &g_s_self.s_file[FILE_WRITE];
        data_file_header_t *sp_header = &sp_file->s_header;

        /* Load file path for current date */
        char *path = NULL;
        int n = _get_path(&path);

        if (n < 0)
        {
            ESP_LOGE(LOG_TAG, "Failed to allocate memory for file path");
            sdcard_unlock();
            if (path)
            {
                free(path);
            }
            return ESP_ERR_NO_MEM;
        }

        /* Load WRITE file header */
        data_file_header_t s_header = {0};
        ret = _header_load(sp_file->path, &s_header);

        if (ret != ESP_OK)
        {
            if (ret == ESP_ERR_NOT_FOUND)
            {
                /* If WRITE file doesn't exist, create a new one with the current date */
                ret = _create_write_file(path);
            }
            else
            {
                ESP_LOGE(LOG_TAG, "Failed to load header from WRITE file: %s", sp_file->path);
            }
        }
        else
        {
            if (sp_header->read < s_header.read)
            {
                sp_header->read = s_header.read;
            }

            if (sp_header->write < s_header.write)
            {
                sp_header->write = s_header.write;
            }

            /* Check if date has changed */
            if (strcmp(path, sp_file->path) != 0)
            {
                /* If all data has been read */
                if (sp_header->read >= sp_header->write)
                {
                    /* Delete the WRITE file */
                    if (ESP_OK == sdcard_delete_file(sp_file->path))
                    {
                        ESP_LOGI(LOG_TAG, "Deleted file: %s", sp_file->path);
                    }
                }

                /* Create new WRITE file for the current date */
                ret = _create_write_file(path);
            }
        }

        if (ret == ESP_OK)
        {
            ret = _data_write(sp_file, (uint8_t *)sp_payload);

            if (ret == ESP_OK)
            {
                /* Update WRITE file header to reflect the new write position */
                data_file_header_t s_header = {0};
                s_header.read = sp_header->read;
                s_header.write = sp_header->write + 1;

                char timestamp[20] = {0};
                _timestamp(sp_payload->timestamp, timestamp, sizeof(timestamp));

                char info[64] = {0};
                snprintf(info, sizeof(info), "WRITE (%s)", timestamp);

                ret = _header_update(info, sp_file->path, &s_header);

                if (ret == ESP_OK)
                {
                    sp_header->write = s_header.write;
                }
                else
                {
                    ret = ESP_ERR_NOT_FINISHED;
                }
            }
        }

        sdcard_unlock();

        if (path)
        {
            free(path);
        }
    }

    return ret;
}

static void _task_connect(void *vp_arg)
{
    do
    {
        do
        {
            g_s_self.vp_cm_handle = cm_prepare_dynamic(0, FEATURE_DATA, xTaskGetCurrentTaskHandle(), NULL, &g_s_self.cm_bit_val);
            vTaskDelay(1000);
        } while (NULL == g_s_self.vp_cm_handle);

        if (g_s_self.vp_cm_handle)
        {
            uint32_t retry = 2;

            do
            {
                if (!g_s_self.vp_conn)
                {
                    g_s_self.vp_conn = tcp_client_init();
                }

                seedlink_info_t *sp_seedlink = &setting_get_device_info()->s_seedlink;

                err_t err = tcp_create_connection(g_s_self.vp_conn, sp_seedlink->server_name, sp_seedlink->tcp_port);

                if (err != ERR_OK)
                {
                    if (g_s_self.vp_conn)
                    {
                        tcp_client_deinit(g_s_self.vp_conn);
                        g_s_self.vp_conn = NULL;
                    }
                }
                else
                {
                    ESP_LOGW(LOG_TAG, "Connected to Seedlink server: %s:%d", sp_seedlink->server_name, sp_seedlink->tcp_port);
                    xSemaphoreGive(g_s_self.sem_connect);
                    break;
                }

                vTaskDelay(1000);
            } while (--retry);

            if (!retry)
            {
                cm_trigger_done(g_s_self.vp_cm_handle, g_s_self.cm_bit_val, FEATURE_DATA, COMM_ERROR_NET_DOWN);
                g_s_self.vp_cm_handle = NULL;
                g_s_self.cm_bit_val = 0;
                vTaskDelay(pdMS_TO_TICKS(15 * 60 * 1000)); /* Wait for 15 minutes before retrying */
            }
            else
            {
                break;
            }
        }
    } while (1);

    xSemaphoreTake(g_s_self.lock, portMAX_DELAY);
    g_s_self.task_connect = NULL;
    xSemaphoreGive(g_s_self.lock);

    vTaskDelete(NULL);
}

static void _task_recovery(void *vp_arg)
{
    data_file_t *sp_file = &g_s_self.s_file[FILE_READ];
    data_file_header_t *sp_header = &sp_file->s_header;

    do
    {
        if (!board_is_sd_card_inserted())
        {
            ESP_LOGE(LOG_TAG, "SD card not available for recovery");
            break;
        }

        geophone_payload_t s_payload = {0};

        esp_err_t e_err = _payload_recover(&s_payload);

        if (e_err != ESP_OK)
        {
            if (e_err == ESP_ERR_NOT_FOUND)
            {
                ESP_LOGE(LOG_TAG, "Recovery file not found");
                break;
            }
            else if (e_err == ESP_ERR_NOT_ALLOWED)
            {
                ESP_LOGE(LOG_TAG, "No payloads to recover, but file is locked: %s", sp_file->path);
                break;
            }
            else if (e_err == ESP_ERR_NOT_FINISHED)
            {
                ESP_LOGW(LOG_TAG, "Payload recovery not finished, will retry: %s", sp_file->path);
                vTaskDelay(1000);
                continue;
            }
            else
            {
                ESP_LOGE(LOG_TAG, "Failed to recover payload: %s", esp_err_to_name(e_err));
                vTaskDelay(1000);
                continue;
            }
        }

        char timestamp[20] = {0};
        _timestamp(s_payload.timestamp, timestamp, sizeof(timestamp));

        if (ESP_OK != _payload_send(&s_payload))
        {
            ESP_LOGE(LOG_TAG, "Failed to send recovered payload, stopping recovery");
            break;
        }
        else
        {
            sdcard_lock(portMAX_DELAY);
            /* Update header to mark payload as sent */
            sp_header->read++;

            char info[64] = {0};
            snprintf(info, sizeof(info), "READ  (%s)", timestamp);

            if (ESP_OK != _header_update(info, sp_file->path, sp_header))
            {
                ESP_LOGE(LOG_TAG, "Failed to update header: %s", sp_file->path);
            }

            ESP_LOGW(LOG_TAG, "Payload (%s) <sent>", timestamp);
            sdcard_unlock();
        }

        vTaskDelay(100);
    } while (1);

    memset(sp_file, 0, sizeof(data_file_t));
    g_s_self.recover = false;

    xSemaphoreTake(g_s_self.lock, portMAX_DELAY);
    g_s_self.task_recovery = NULL;
    xSemaphoreGive(g_s_self.lock);

    vTaskDelete(NULL);
}

static void _task(void *vp_arg)
{
    geophone_payload_t s_payload = {0};

    while (pdTRUE == xQueueReceive(g_s_self.queue, &s_payload, portMAX_DELAY))
    {
        bool sent = false;

        /* If not connected to SeedLink server */
        if (pdFALSE == xSemaphoreTake(g_s_self.sem_connect, 0))
        {
            g_s_self.recover = true;

            xSemaphoreTake(g_s_self.lock, portMAX_DELAY);
            if (!g_s_self.task_connect)
            {
                if (pdPASS != xTaskCreatePinnedToCore(_task_connect, "connect", SYSTEM_STACK_SIZE(2), NULL, 7, &g_s_self.task_connect, 1))
                {
                    ESP_LOGE(LOG_TAG, "Failed to create tcp_connect task");
                }
            }
            xSemaphoreGive(g_s_self.lock);
        }
        else
        {
            g_s_self.recover = false;
            xSemaphoreGive(g_s_self.sem_connect);
        }

        /* Check SD card availability and usage */
        float usage = -0.0f;

        if (board_is_sd_card_inserted())
        {
            uint64_t total = 0, used = 0;

            if (ESP_OK == sdcard_get_info(&total, &used, NULL))
            {
                usage = ((float)used / (float)total) * 100.0f;
            }
        }

        /* Create timestamp */
        char timestamp[20] = {0};
        _timestamp(s_payload.timestamp, timestamp, sizeof(timestamp));

        printf("Payload (%s) <%ld, sdcard: %.2f%%>\n", timestamp, s_payload.sequence_number, usage);
        data_file_t *sp_file = &g_s_self.s_file[FILE_WRITE];
        data_file_header_t *sp_header = &sp_file->s_header;

        /* Save the payload in SD card */
        esp_err_t e_err = _payload_save(&s_payload);

        /* If payload save failed */
        if (e_err != ESP_OK)
        {
            /* If only failed to update header */
            if (e_err == ESP_ERR_NOT_FINISHED)
            {
                sp_header->write++;
                e_err = ESP_OK;
            }
        }

        /* Check if recovery is required */
        if (g_s_self.recover)
        {
            /* Start recovery only if connected */
            if (pdTRUE == xSemaphoreTake(g_s_self.sem_connect, 0))
            {
                xSemaphoreGive(g_s_self.sem_connect);

                xSemaphoreTake(g_s_self.lock, portMAX_DELAY);
                if (!g_s_self.task_recovery)
                {
                    if (pdPASS != xTaskCreatePinnedToCore(_task_recovery, "recovery", SYSTEM_STACK_SIZE(4), NULL, 5, &g_s_self.task_recovery, 1))
                    {
                        ESP_LOGE(LOG_TAG, "Failed to create recovery task");
                    }
                }
                xSemaphoreGive(g_s_self.lock);
            }
        }
        else /* Try to send the payload to SeedLink server */
        {
            if (ESP_OK != _payload_send(&s_payload))
            {
                ESP_LOGE(LOG_TAG, "Failed to send payload: %s", timestamp);
            }
            else
            {
                sdcard_lock(portMAX_DELAY);
                /* Update header to mark payload as sent */
                sp_header->read++;

                char info[64] = {0};
                snprintf(info, sizeof(info), "READ  (%s)", timestamp);

                if (ESP_OK != _header_update(info, sp_file->path, sp_header))
                {
                    ESP_LOGE(LOG_TAG, "Failed to update header: %s", sp_file->path);
                }

                ESP_LOGW(LOG_TAG, "Payload (%s) <sent>", timestamp);
                sdcard_unlock();
                sent = true;
            }
        }

        /* If payload neither saved nor sent */
        if (e_err != ESP_OK && !sent)
        {
            /* Requeue the payload */
            if (errQUEUE_FULL == xQueueSendToFront(g_s_self.queue, &s_payload, 0))
            {
                ESP_LOGE(LOG_TAG, "Failed to requeue payload for recovery");
            }
        }
    }

    g_s_self.task = NULL;
    vTaskDelete(NULL);
}

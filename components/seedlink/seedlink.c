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

#include "seedlink.h"
#include "mseed.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include <netdb.h>   // for gethostbyname()


/*****************************************************************************************************************
 * MACROS AND DEFINES
 */

#define SEEDLINK_SERVER_IP     "ring.wscada.net"
#define SEEDLINK_DATALINK_PORT 16000

/*****************************************************************************************************************
 * EXTERN PUBLIC VARIABLES
 */

/*****************************************************************************************************************
 * EXTERN PUBLIC FUNCTION DECLARATIONS
 */

/*****************************************************************************************************************
 * PRIVATE STRUCTURES
 */

typedef struct SEEDLINK
{
    TaskHandle_t task;
    TaskHandle_t task_connect;
    QueueHandle_t queue;
    SemaphoreHandle_t lock;
    SemaphoreHandle_t sem_connect;
    int sock;
} seedlink_t;

/*****************************************************************************************************************
 * PRIVATE FUNCTION DECLARATIONS
 */

static void _task(void *vp_arg);
static void _task_connect(void *vp_arg);

/*****************************************************************************************************************
 * PRIVATE VARIABLES
 */

static const char *LOG_TAG = "[seedlink]";
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

/*****************************************************************************************************************
 * PUBLIC FUNCTION DEFINITIONS
 */

esp_err_t seedlink_init(void)
{
    g_s_self.queue = xQueueCreate(10, sizeof(imu_payload_t));
    ESP_RETURN_ON_FALSE(g_s_self.queue, ESP_ERR_NO_MEM, LOG_TAG, "Failed to create imu payload queue");

    g_s_self.lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(g_s_self.lock, ESP_ERR_NO_MEM, LOG_TAG, "Failed to create seedlink lock");

    g_s_self.sem_connect = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(g_s_self.sem_connect, ESP_ERR_NO_MEM, LOG_TAG, "Failed to create seedlink connect semaphore");

    BaseType_t xRet = xTaskCreatePinnedToCore(_task, "seedlink", 8192, NULL, 5, &g_s_self.task, 1);
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

static uint8_t *_seed_create(float *p_data, mseed_config_t *sp_cfg, const char *channel)
{
    snprintf(sp_cfg->channel, sizeof(sp_cfg->channel), "%s", channel);
    uint8_t *p_record = mseed_record(p_data, sp_cfg);
    uint8_t *p_datalink = mseed_datalink(p_record, sp_cfg);
    return p_datalink;
}

static uint32_t _pkt_create(imu_payload_t *sp_payload, uint8_t *p_buffer, uint32_t buffer_size)
{

    mseed_config_t s_cfg = {0};
    s_cfg.sample_count = IMU_MAX_SAMPLES;
    s_cfg.sequence_number = sp_payload->sequence_number;
    s_cfg.rate_hz = IMU_SAMPLE_RATE_HZ;   
    strlcpy(s_cfg.station,  "DMG37", sizeof(s_cfg.station));   //  station code
    strlcpy(s_cfg.location, "00",    sizeof(s_cfg.location));
    strlcpy(s_cfg.network,  "NP",    sizeof(s_cfg.network));  
    s_cfg.start_time = sp_payload->timestamp;
    s_cfg.time_fract = 0;


    const uint32_t seed_size = 583;
    uint8_t *p_seed = NULL;

    struct channel
    {
        const char *name;
        float *data;
    } channels[] = {
        {"HNN", sp_payload->ax},
        {"HNZ", sp_payload->az},
        {"HNE", sp_payload->ay},
    };

    uint8_t channel_count = sizeof(channels) / sizeof(channels[0]);

    for (int i = 0; i < channel_count; i++)
    {
        p_seed = _seed_create(channels[i].data, &s_cfg, channels[i].name);
        memcpy(p_buffer + (i * seed_size), p_seed, seed_size);
    }

    return seed_size * channel_count;
}

static esp_err_t _payload_send(imu_payload_t *sp_payload)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t buffer[2048];
    uint32_t size = _pkt_create(sp_payload, buffer, sizeof(buffer));
    if (size)
    {
        uint32_t total_sent = 0;
        bool failed = false;
        while (total_sent < size)
        {
            int sent = send(g_s_self.sock, buffer + total_sent, size - total_sent, 0);
            if (sent <= 0)
            {
                failed = true;
                break;
            }
            total_sent += (uint32_t)sent;
        }
        if (failed)
        {
            close(g_s_self.sock);
            g_s_self.sock = -1;
            xSemaphoreTake(g_s_self.sem_connect, portMAX_DELAY);
        }
        else
        {
            /* ---- read back ringserver's response, now that the WRITE command asks for one (the 'A' flag) ---- */
            char resp[128] = {0};
            int r = recv(g_s_self.sock, resp, sizeof(resp) - 1, 0);
            if (r > 0)
            {
                resp[r] = '\0';
                ESP_LOGI(LOG_TAG, "Server response: %s", resp);
            }
            else if (r == 0)
            {
                ESP_LOGW(LOG_TAG, "Server closed connection with no response");
            }
            else
            {
                ESP_LOGW(LOG_TAG, "recv() failed/timed out waiting for response");
            }
            /**/

            ret = ESP_OK;
        }
    }
    return ret;
}


static void _task_connect(void *vp_arg)
{
    uint32_t retry = 2;

    do
    {
        struct sockaddr_in addr = {0};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(SEEDLINK_DATALINK_PORT);

        /* DNS lookup -- SEEDLINK_SERVER_IP is now a hostname, not a literal IP,
           so inet_pton() alone won't work anymore */
        struct hostent *he = gethostbyname(SEEDLINK_SERVER_IP);
        if (he == NULL)
        {
            ESP_LOGE(LOG_TAG, "DNS lookup failed for %s", SEEDLINK_SERVER_IP);
            vTaskDelay(1000);
            continue;
        }
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length); //pass the raw binary form the scoket actually needs

        g_s_self.sock = socket(AF_INET, SOCK_STREAM, 0);
        if (connect(g_s_self.sock, (struct sockaddr *)&addr, sizeof(addr)) == 0)
        {
            struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
            setsockopt(g_s_self.sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            ESP_LOGW(LOG_TAG, "Connected to Seedlink server");
            xSemaphoreGive(g_s_self.sem_connect);
            break;
        }
        close(g_s_self.sock);
        g_s_self.sock = -1;
        vTaskDelay(1000);

    } while (--retry);

    g_s_self.task_connect = NULL;
    vTaskDelete(NULL);
}

static void _task(void *vp_arg)
{
    imu_payload_t s_payload = {0};

    while (pdTRUE == xQueueReceive(g_s_self.queue, &s_payload, portMAX_DELAY))
    {

        /* If not connected to SeedLink server */
        if (pdFALSE == xSemaphoreTake(g_s_self.sem_connect, 0))
        {

            xSemaphoreTake(g_s_self.lock, portMAX_DELAY);
            if (!g_s_self.task_connect)
            {
                if (pdPASS != xTaskCreatePinnedToCore(_task_connect, "connect", 4096, NULL, 7, &g_s_self.task_connect, 1))
                {
                    ESP_LOGE(LOG_TAG, "Failed to create tcp_connect task");
                }
            }
            xSemaphoreGive(g_s_self.lock);
            continue;   // sample is dropped if not connected
        }
        xSemaphoreGive(g_s_self.sem_connect);

        char timestamp[20] = {0};
        _timestamp(s_payload.timestamp, timestamp, sizeof(timestamp));
        printf("Payload (%s) <%lu>\n", timestamp, s_payload.sequence_number);

        if (ESP_OK != _payload_send(&s_payload))
        {
            ESP_LOGE(LOG_TAG, "Failed to send payload: %s", timestamp);
        }
    }

    g_s_self.task = NULL;
    vTaskDelete(NULL);
}

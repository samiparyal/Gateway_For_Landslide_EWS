/*---------------------------------------------------------------------------------------------------------------
 * ad7768.c
 *
 * Created on: July 15, 2025
 * By: Ravi Tamrakar
 *
 *--------------------------------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "rom/ets_sys.h"
#include "driver/gpio.h"

#include "esp_check.h"
#include "esp_heap_caps.h"

#include "ad7768.h"

/*---------------------------------------------------------------------------------------------------------------
 * MACROS AND DEFINES
 *--------------------------------------------------------------------------------------------------------------*/

#define AD7768_RING_BUFFER_SIZE (AD7768_FRAME_SIZE * AD7768_FRAME_NUM * 131) // ~1 second of data at 256 SPS with some extra space for dropped data accounting

#define AD7768_TASK_STACK_SIZE (4096)
#define AD7768_TASK_PRIORITY (configMAX_PRIORITIES - 2)
#define AD7768_TASK_READ_TIMEOUT_MS (50)

/*---------------------------------------------------------------------------------------------------------------
 * EXTERN PUBLIC VARIABLES
 *--------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------------
 * EXTERN PUBLIC FUNCTION DECLARATIONS
 *--------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------------
 * PRIVATE STRUCTURES
 *--------------------------------------------------------------------------------------------------------------*/

typedef struct AD7768
{
    TaskHandle_t task_capture;
    i2s_chan_handle_t i2s_chan;
    gpio_num_t reset_pin;
    uint8_t *p_ring_buffer;
    size_t ring_head;
    size_t ring_tail;
    uint32_t dropped_bytes;
    bool is_initialized;
    bool is_started;
} ad7768_t;

/*---------------------------------------------------------------------------------------------------------------
 * PRIVATE FUNCTION DECLARATIONS
 *--------------------------------------------------------------------------------------------------------------*/

static void _task_capture(void *vp_arg);
#if 1
static size_t _ring_available_locked(void);
static size_t _ring_read_locked(uint8_t *p_data, size_t size);
#endif
static uint8_t *_ring_peek(size_t *out_len);
static void _ring_advance(size_t size);

/*---------------------------------------------------------------------------------------------------------------
 * PRIVATE VARIABLES
 *--------------------------------------------------------------------------------------------------------------*/

static const char *LOG_TAG = "[ad7768]";
static ad7768_t g_s_self = {0};
static uint8_t g_buffer[AD7768_BUFF_SIZE];
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

/*---------------------------------------------------------------------------------------------------------------
 * PRIVATE INLINE FUNCTION DEFINITIONS
 *--------------------------------------------------------------------------------------------------------------*/

static inline uint32_t _extract_channel(uint8_t ch_no, size_t frame_no, uint8_t *p_buff)
{
    size_t pos = frame_no * AD7768_FRAME_SIZE;
    size_t ch_offset = ch_no * 4; // Each channel has 4 bytes (1 byte padding + 3 bytes data)
    
    return ((uint32_t)p_buff[pos + ch_offset + 1] << 16u) |
           ((uint32_t)p_buff[pos + ch_offset + 2] << 8u) |
           (uint32_t)p_buff[pos + ch_offset + 3];
}

static inline int32_t _sign_extend_24bit(uint32_t value)
{
    if (value & 0x800000)
    {
        value |= 0xFF000000;
    }

    return (int32_t)value;
}

/*---------------------------------------------------------------------------------------------------------------
 * PUBLIC FUNCTION DEFINITIONS
 *--------------------------------------------------------------------------------------------------------------*/

esp_err_t ad7768_init(ad7768_cfg_t *p_s_cfg)
{
    g_s_self.reset_pin = p_s_cfg->reset_pin;

    /* Allocate ring buffer from PSRAM */
    g_s_self.p_ring_buffer = (uint8_t *)heap_caps_malloc(AD7768_RING_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    ESP_RETURN_ON_FALSE(g_s_self.p_ring_buffer, ESP_ERR_NO_MEM, LOG_TAG,
                        "Failed to allocate ring buffer from PSRAM (%d bytes)", AD7768_RING_BUFFER_SIZE);

    ESP_RETURN_ON_ERROR(i2s_new_channel(&p_s_cfg->s_i2s_chan_cfg, NULL, &g_s_self.i2s_chan),
                        LOG_TAG, "Failed to create I2S channel: %s", esp_err_to_name(err_rc_));
    ESP_RETURN_ON_ERROR(i2s_channel_init_tdm_mode(g_s_self.i2s_chan, &p_s_cfg->s_i2s_tdm_cfg),
                        LOG_TAG, "Failed to initialize I2S TDM mode: %s", esp_err_to_name(err_rc_));

    g_s_self.is_initialized = true;
    return ESP_OK;
}

void ad7768_deinit(void)
{
    if (g_s_self.is_started)
    {
        ad7768_stop();
    }

    if (g_s_self.p_ring_buffer)
    {
        heap_caps_free(g_s_self.p_ring_buffer);
        g_s_self.p_ring_buffer = NULL;
    }

    if (g_s_self.i2s_chan)
    {
        i2s_del_channel(g_s_self.i2s_chan);
        g_s_self.i2s_chan = NULL;
    }

    g_s_self.is_initialized = false;
}

void ad7768_reset(void)
{
    if (g_s_self.reset_pin != GPIO_NUM_NC)
    {
        gpio_set_level(g_s_self.reset_pin, 0);
        vTaskDelay(2);
        gpio_set_level(g_s_self.reset_pin, 1);
        vTaskDelay(10);
    }
}

esp_err_t ad7768_start(void)
{
    ESP_RETURN_ON_FALSE(g_s_self.is_initialized, ESP_ERR_INVALID_STATE, LOG_TAG, "Not initialized");
    ESP_RETURN_ON_FALSE(!g_s_self.is_started, ESP_ERR_INVALID_STATE, LOG_TAG, "Already started");

    ESP_RETURN_ON_ERROR(i2s_channel_enable(g_s_self.i2s_chan),
                        LOG_TAG, "Failed to enable I2S channel: %s", esp_err_to_name(err_rc_));

    g_s_self.is_started = true;

    taskENTER_CRITICAL(&g_mux);
    g_s_self.ring_head = 0;
    g_s_self.ring_tail = 0;
    g_s_self.dropped_bytes = 0;
    taskEXIT_CRITICAL(&g_mux);

    BaseType_t xRet = (!g_s_self.task_capture)? xTaskCreatePinnedToCore(_task_capture, "ad7768", AD7768_TASK_STACK_SIZE, NULL, AD7768_TASK_PRIORITY, &g_s_self.task_capture, 0) : pdPASS;

    if (xRet != pdPASS)
    {
        g_s_self.is_started = false;
        i2s_channel_disable(g_s_self.i2s_chan);
        ESP_LOGE(LOG_TAG, "Failed to create capture task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t ad7768_stop(void)
{
    ESP_RETURN_ON_FALSE(g_s_self.is_initialized, ESP_ERR_INVALID_STATE, LOG_TAG, "Not initialized");
    ESP_RETURN_ON_FALSE(g_s_self.is_started, ESP_ERR_INVALID_STATE, LOG_TAG, "Not started");

    g_s_self.is_started = false;

    TickType_t wait_ticks = pdMS_TO_TICKS(AD7768_TASK_READ_TIMEOUT_MS + 20);
    TickType_t start_ticks = xTaskGetTickCount();

    while (g_s_self.task_capture && (xTaskGetTickCount() - start_ticks) < wait_ticks)
    {
        vTaskDelay(1);
    }

    ESP_RETURN_ON_ERROR(i2s_channel_disable(g_s_self.i2s_chan),
                        LOG_TAG, "Failed to disable I2S channel: %s", esp_err_to_name(err_rc_));

    return ESP_OK;
}
#if 0
void ad7768_read_channels(double *p_ch1, double *p_ch2, double *p_ch3, size_t frame_cnt)
{
    if (!p_ch1 || !p_ch2 || !p_ch3 || frame_cnt == 0)
    {
        ESP_LOGE(LOG_TAG, "Invalid arguments to read_channels");
        return;
    }

    size_t size = frame_cnt * AD7768_FRAME_SIZE;
    int64_t ch1_sum = 0, ch2_sum = 0, ch3_sum = 0;
    size_t processed = 0;

    // Wait until enough data is available in the ring buffer
    while (1)
    {
        taskENTER_CRITICAL(&g_mux);
        size_t peek_len = 0;
        uint8_t *peek_ptr = _ring_peek(&peek_len);
        size_t available = (g_s_self.ring_head >= g_s_self.ring_tail)
                              ? (g_s_self.ring_head - g_s_self.ring_tail)
                              : (AD7768_RING_BUFFER_SIZE - (g_s_self.ring_tail - g_s_self.ring_head));
        if (peek_ptr && available >= size)
        {
            taskEXIT_CRITICAL(&g_mux);
            break;
        }
        taskEXIT_CRITICAL(&g_mux);
        vTaskDelay(1);
    }

    // Process the data, handling wrap-around if needed
    size_t to_process = size;
    while (to_process > 0)
    {
        taskENTER_CRITICAL(&g_mux);
        size_t peek_len = 0;
        uint8_t *peek_ptr = _ring_peek(&peek_len);
        size_t chunk = (peek_len < to_process) ? peek_len : to_process;
        size_t frames_in_chunk = chunk / AD7768_FRAME_SIZE;
        for (size_t i = 0; i < frames_in_chunk; i++)
        {
            size_t frame_idx = (processed / AD7768_FRAME_SIZE) + i;
            uint32_t ch1_u = _extract_channel(0, frame_idx, peek_ptr);
            uint32_t ch2_u = _extract_channel(1, frame_idx, peek_ptr);
            uint32_t ch3_u = _extract_channel(2, frame_idx, peek_ptr);
            int32_t ch1_raw = _sign_extend_24bit(ch1_u);
            int32_t ch2_raw = _sign_extend_24bit(ch2_u);
            int32_t ch3_raw = _sign_extend_24bit(ch3_u);
            ch1_sum += ch1_raw;
            ch2_sum += ch2_raw;
            ch3_sum += ch3_raw;
        }
        _ring_advance(chunk);
        processed += chunk;
        to_process -= chunk;
        taskEXIT_CRITICAL(&g_mux);
    }

    *p_ch1 = (double)ch1_sum / frame_cnt;
    *p_ch2 = (double)ch2_sum / frame_cnt;
    *p_ch3 = (double)ch3_sum / frame_cnt;
}
#else // Deprecated: memcpy-based read. Use zero-copy API instead.
size_t ad7768_read(uint8_t *p_buffer, size_t size)
{
    size_t bytes_read = 0;

    if (!g_s_self.is_started)
    {
        return 0;
    }

    if (!p_buffer || (size == 0))
    {
        taskENTER_CRITICAL(&g_mux);
        bytes_read = _ring_available_locked();
        taskEXIT_CRITICAL(&g_mux);
        return bytes_read;
    }

    while (g_s_self.is_started)
    {
        taskENTER_CRITICAL(&g_mux);
        bytes_read = _ring_read_locked(p_buffer, size);
        taskEXIT_CRITICAL(&g_mux);

        if (bytes_read > 0)
        {
            return bytes_read;
        }

        vTaskDelay(1);
    }

    return 0;
}
#endif

/*---------------------------------------------------------------------------------------------------------------
 * PRIVATE FUNCTION DEFINITIONS
 *--------------------------------------------------------------------------------------------------------------*/

static size_t _ring_available_locked(void)
{
    if (!g_s_self.p_ring_buffer)
    {
        return 0;
    }

    if (g_s_self.ring_head >= g_s_self.ring_tail)
    {
        return g_s_self.ring_head - g_s_self.ring_tail;
    }

    return AD7768_RING_BUFFER_SIZE - (g_s_self.ring_tail - g_s_self.ring_head);
}

static size_t _ring_free_locked(void)
{
    if (!g_s_self.p_ring_buffer)
    {
        return 0;
    }

    return (AD7768_RING_BUFFER_SIZE - 1) - _ring_available_locked();
}

// Zero-copy ring buffer read API
// Returns pointer to next readable region and its length (up to end of buffer)
// Returns NULL if no data available
static uint8_t *_ring_peek(size_t *out_len)
{
    if (!g_s_self.p_ring_buffer || !out_len)
    {
        return NULL;
    }

    size_t available = _ring_available_locked();

    if (available == 0)
    {
        *out_len = 0;
        return NULL;
    }

    size_t first_chunk = AD7768_RING_BUFFER_SIZE - g_s_self.ring_tail;
    *out_len = (available < first_chunk) ? available : first_chunk;

    return &g_s_self.p_ring_buffer[g_s_self.ring_tail];
}

// Advances the read pointer by len bytes (must be <= available)
static void _ring_advance(size_t len)
{
    if (!g_s_self.p_ring_buffer || len == 0)
    {
        return;
    }

    g_s_self.ring_tail = (g_s_self.ring_tail + len) % AD7768_RING_BUFFER_SIZE;
}

// Deprecated: memcpy-based read. Use zero-copy API instead.
static size_t _ring_read_locked(uint8_t *p_data, size_t size)
{
    if (!g_s_self.p_ring_buffer)
    {
        return 0;
    }

    size_t available = _ring_available_locked();
    size_t read_size = (size < available) ? size : available;
    size_t first_chunk = AD7768_RING_BUFFER_SIZE - g_s_self.ring_tail;

    if (read_size == 0)
    {
        return 0;
    }

    if (first_chunk > read_size)
    {
        first_chunk = read_size;
    }

    memcpy(p_data, &g_s_self.p_ring_buffer[g_s_self.ring_tail], first_chunk);

    if (read_size > first_chunk)
    {
        memcpy(p_data + first_chunk, g_s_self.p_ring_buffer, read_size - first_chunk);
    }

    g_s_self.ring_tail = (g_s_self.ring_tail + read_size) % AD7768_RING_BUFFER_SIZE;

    return read_size;
}

static void _ring_write_locked(const uint8_t *cp_data, size_t size)
{
    if (!g_s_self.p_ring_buffer)
    {
        return;
    }

    size_t free_space = _ring_free_locked();

    if (size > free_space)
    {
        size_t dropped = size - free_space;

        g_s_self.ring_tail = (g_s_self.ring_tail + dropped) % AD7768_RING_BUFFER_SIZE;
        g_s_self.dropped_bytes += (uint32_t)dropped;
    }

    size_t first_chunk = AD7768_RING_BUFFER_SIZE - g_s_self.ring_head;

    if (first_chunk > size)
    {
        first_chunk = size;
    }

    memcpy(&g_s_self.p_ring_buffer[g_s_self.ring_head], cp_data, first_chunk);

    if (size > first_chunk)
    {
        memcpy(g_s_self.p_ring_buffer, cp_data + first_chunk, size - first_chunk);
    }

    g_s_self.ring_head = (g_s_self.ring_head + size) % AD7768_RING_BUFFER_SIZE;
}

static void _task_capture(void *vp_arg)
{
    (void)vp_arg;

    while (g_s_self.is_started)
    {
        size_t bytes_read = 0;
        esp_err_t e_err = i2s_channel_read(g_s_self.i2s_chan, g_buffer, sizeof(g_buffer), &bytes_read, AD7768_TASK_READ_TIMEOUT_MS);

        if ((e_err == ESP_OK) && (bytes_read > 0))
        {
            taskENTER_CRITICAL(&g_mux);
            _ring_write_locked(g_buffer, bytes_read);
            taskEXIT_CRITICAL(&g_mux);
        }
        else if ((e_err != ESP_OK) && (e_err != ESP_ERR_TIMEOUT) && g_s_self.is_started)
        {
            ESP_LOGE(LOG_TAG, "Failed to read data from I2S channel: %s", esp_err_to_name(e_err));
            ad7768_stop();
            ad7768_reset();
            ad7768_start();
        }

        if (g_s_self.dropped_bytes)
        {
            printf("Dropped %u bytes due to ring buffer overflow\n", g_s_self.dropped_bytes);
            g_s_self.dropped_bytes = 0;
        }
    }

    g_s_self.task_capture = NULL;
    vTaskDelete(NULL);
}

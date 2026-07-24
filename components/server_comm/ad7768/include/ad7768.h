#pragma once

#include "driver/i2s_tdm.h"

#define AD7768_FRAME_SIZE (16)
#define AD7768_FRAME_NUM (240)
#define AD7768_SAMPLE_RATE (31250)                              // 31.25kHz sample rate to achieve 4MHz BCLK with 16 bytes frame size and 240 frames per read
#define AD7768_BUFF_SIZE (AD7768_FRAME_SIZE * AD7768_FRAME_NUM) // 240 frames per read, each frame is 16 bytes (4 slots x 32-bit)

// Default configuration macro for AD7768
#define AD7768_DEFAULT_CFG {                                                                                                                     \
    .s_i2s_chan_cfg = {                                                                                                                          \
        .id = I2S_NUM_1,                                                                                                                         \
        .role = I2S_ROLE_SLAVE,                                                                                                                  \
        .dma_desc_num = 6,                                                                                                                       \
        .dma_frame_num = AD7768_FRAME_NUM,                                                                                                       \
        .auto_clear_after_cb = false,                                                                                                            \
        .auto_clear_before_cb = false,                                                                                                           \
        .intr_priority = 0,                                                                                                                      \
    },                                                                                                                                           \
    .s_i2s_tdm_cfg = {                                                                                                                          \
        .clk_cfg = {                                                                                                                              \
            .clk_src = I2S_CLK_SRC_DEFAULT,                                                                                                      \
            .mclk_multiple = I2S_MCLK_MULTIPLE_1024,                                                                                             \
            .sample_rate_hz = AD7768_SAMPLE_RATE, /* Fixed so that the BCLK is set to 4MHz. BCLK = no. of slot x data bit width x sample rate */ \
            .bclk_div = 8,                        /* Making sure MCLK is 32MHz */                                                                \
        },                                                                                                                                       \
        .slot_cfg = I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_8BIT, I2S_SLOT_MODE_STEREO, 0xffff),                                      \                              
        .gpio_cfg = {                                                                                                                            \
            .ws = GPIO_NUM_NC,                                                                                                                   \
            .dout = GPIO_NUM_NC,                                                                                                                 \
            .din = GPIO_NUM_NC,                                                                                                                  \
            .bclk = GPIO_NUM_NC,                                                                                                                 \
            .mclk = GPIO_NUM_NC,                                                                                                                 \
        },                                                                                                                                       \
    },                                                                                                                                           \
    .reset_pin = GPIO_NUM_NC,                                                                                                                    \
}

typedef struct AD7768_CFG
{
    i2s_chan_config_t s_i2s_chan_cfg; // I2S channel configuration
    i2s_tdm_config_t s_i2s_tdm_cfg;   // I2S TDM configuration
    gpio_num_t reset_pin;             // GPIO pin for ADC reset
} ad7768_cfg_t;

esp_err_t ad7768_init(ad7768_cfg_t *p_s_cfg);
void ad7768_deinit(void);
void ad7768_reset(void);

esp_err_t ad7768_start(void);
esp_err_t ad7768_stop(void);

void ad7768_read_channels(double *p_ch1, double *p_ch2, double *p_ch3, size_t frame_cnt);
size_t ad7768_read(uint8_t *p_buffer, size_t size);
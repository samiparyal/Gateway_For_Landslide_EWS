#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

esp_err_t seedlink_init(void);

QueueHandle_t seedlink_get_queue(void);
uint32_t seedlink_get_stored(void);

#define IMU_MAX_SAMPLES 114
// 512-byte record (fixed by SeedLink) - 56 bytes (48 header + 8 blockette1000) = 456 bytes free
// 456 / 4 bytes-per-float32-sample = 114 samples max per record

#define IMU_SAMPLE_RATE_HZ 120   // TRAINING_MODE_RATE_MS on the sensor node

typedef struct {
    uint32_t sequence_number;
    uint64_t timestamp;
    float ax[IMU_MAX_SAMPLES];
    float ay[IMU_MAX_SAMPLES];
    float az[IMU_MAX_SAMPLES];
} imu_payload_t;
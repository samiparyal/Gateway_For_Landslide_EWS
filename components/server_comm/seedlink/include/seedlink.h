#pragma once

esp_err_t seedlink_init(void);

QueueHandle_t seedlink_get_queue(void);
uint32_t seedlink_get_stored(void);
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "host/ble_gap.h"


/* one gateway sees many sensors; track each peer's last mfg_data separately */
#define MAX_TRACKED_SENSORS 1

typedef struct {
    ble_addr_t addr;
    uint8_t mfg_data[BLE_HS_ADV_MAX_FIELD_SZ];
    uint8_t mfg_data_len;
    bool occupied;
} sensor_state_t;

/*
 * passive scanning for
 * LANDSLIDE_EWS advs.
 */
void gap_scan_start(uint8_t own_addr_type);

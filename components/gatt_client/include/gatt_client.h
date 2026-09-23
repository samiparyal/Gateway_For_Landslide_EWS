#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "host/ble_gap.h"
#include "esp_timer.h"

#define STARTUP_GRACE_PERIOD_US 120000000 /* 60s to find all the active sensors */
extern esp_timer_handle_t s_grace_timer;

typedef struct {
    /* AlertStatus characteristic (0xFF02): [state][trigger_reason] */
    uint8_t alarm;
    uint8_t trigger;
    bool has_alert_status;

    /* TiltData characteristic (0xFF03): [dev_x100 u16 LE][vel_x100 u16 LE] */
    uint16_t dev_x100;
    uint16_t vel_x100;
    bool has_tilt_data;

    /* RawImuSample characteristic (0xFF04) */
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    uint64_t imu_timestamp_ms;   /* node's wall-clock ms, synced via CTS-style time-sync write */
    uint8_t imu_is_hist_burst;
    bool has_raw_imu;
} gatt_landslide_data_t;

/*
 * Fires once a GATT session ends, however it ended (connect failure,
 * peer disconnect, service not found, ...), so the caller can resume BLE
 * scanning - initiating a connection cancels any in-progress scan.
 */
typedef void (*gatt_session_end_cb_t)(void);

/*
 * own_addr_type: the value ble_app_on_sync_cb() got back from
 * ble_hs_id_infer_auto(); reused here for outgoing connections.
 */
void gatt_client_init(uint8_t own_addr_type);

void gatt_client_set_session_end_cb(gatt_session_end_cb_t cb);

int gatt_client_connect(const ble_addr_t *peer_addr);


void gatt_client_restart_node(const char *station);
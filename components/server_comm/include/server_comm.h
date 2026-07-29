#pragma once

#include <stdbool.h>

extern bool g_gatt_connect_requested;
extern bool hide_gatt_logs;

/* Training-data collection mode: raw IMU handler skips dedup + JSON report
 * Match with TRAINING_MODE_ENABLED on the sensor node. */
extern bool training_mode_sim_logs;

void server_comm_init(void);

/*
 * feeding the raw command string/body received from the server
 * here. Recognizes "connect" and sets g_gatt_connect_requested accordingly.
 */
void server_comm_handle_command(const char *cmd_str);

/*
 * Sends an already-built JSON payload up to the server.
 */
void server_comm_send_json(const char *json_str);

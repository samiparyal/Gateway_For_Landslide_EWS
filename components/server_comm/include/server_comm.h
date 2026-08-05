#pragma once

#include <stdbool.h>

extern bool g_gatt_connect_requested;
extern bool hide_gatt_logs;
extern bool training_mode_sim_logs;

void server_comm_init(void);

void server_comm_handle_command(const char *cmd_str);

/*
 * Sends an already-built JSON payload up to the server.
 */
void server_comm_send_json(const char *json_str);

#pragma once

#include <stdbool.h>

extern bool g_gatt_connect_requested;

void server_comm_init(void);

/*
 * feeding the raw command string/body received from the server
 * here (e.g. straight from an HTTP request body or a parsed JSON field).
 * Recognizes "connect" and sets g_gatt_connect_requested accordingly.
 */
void server_comm_handle_command(const char *cmd_str);

/*
 * Sends an already-built JSON payload up to the server.
 */
void server_comm_send_json(const char *json_str);

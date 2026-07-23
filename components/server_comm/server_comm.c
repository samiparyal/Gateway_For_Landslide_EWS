#include "server_comm.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "SERVER_COMM";

bool g_gatt_connect_requested = true; /* for testing use true */

void server_comm_init(void)
{
    /*
     * TODO(server): call server_comm_handle_command() with the command string whenever
     * one arrives.
     */
}

void server_comm_handle_command(const char *cmd_str)
{
    if (cmd_str == NULL) {
        return;
    }
    if (strcmp(cmd_str, "GATT_Connect") == 0) {
        g_gatt_connect_requested = true;
    }
}

void server_comm_send_json(const char *json_str)
{
    ESP_LOGI(TAG, "\n [gateway -> server] %s \n", json_str);

}

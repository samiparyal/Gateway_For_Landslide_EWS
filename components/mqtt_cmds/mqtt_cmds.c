#include "mqtt_cmds.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "esp_log.h"
#include "gatt_client.h"
#include <string.h>
#include "esp_system.h"

static const char *TAG = "MQTT_CMDS";

#define MQTT_BROKER_URI "mqtt://admin.easy-q.online:1883"
#define MQTT_CMD_TOPIC "RTS666/sub/rts/landslideEWScmds"

static void handle_command(const char *cmd)
{
    ESP_LOGI(TAG, "Received command: %s", cmd);

    if (strcmp(cmd, "restart-node-all") == 0)
    {
        gatt_client_restart_node("all");
    }
    else if(strcmp(cmd, "restart-gateway") == 0)
    {
        ESP_LOGI(TAG, "Restarting gateway...");
        esp_restart();
    }
    else
    {
        ESP_LOGW(TAG, "Unknown command: %s", cmd);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch((esp_mqtt_event_id_t)event_id)
    {
        case MQTT_EVENT_CONNECTED:
        {
            ESP_LOGI(TAG, "Connected to MQTT broker");
            esp_mqtt_client_subscribe(client, MQTT_CMD_TOPIC, 0);
            break;
        }
        case MQTT_EVENT_DATA:
        {
            cJSON *root = cJSON_ParseWithLength(event->data, event->data_len);
            if(root)
            {
                cJSON *cmd_item = cJSON_GetObjectItem(root, "cmd");
                if(cJSON_IsString(cmd_item))
                {
                    handle_command(cmd_item->valuestring);
                }
                cJSON_Delete(root);
            }
            break;
        }
        default:
            break;
    }
}

void mqtt_cmds_init(void)
{
    esp_mqtt_client_config_t cfg =
    {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.username = "rtsmqtt",
        .credentials.authentication.password = "rtsmqttpassword123*#",
        .network.disable_auto_reconnect = true,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}
#include "json_builder.h"
#include <stdio.h>
#include <string.h>
#include "cJSON.h"

static void addr_to_str(const ble_addr_t *addr, char *out, size_t out_len)
{
    snprintf(out, out_len, "%02x:%02x:%02x:%02x:%02x:%02x",
             addr->val[5], addr->val[4], addr->val[3],
             addr->val[2], addr->val[1], addr->val[0]);
}

/* status byte is TiltState_t from tilt_detector.h on the sensor */
static const char *status_to_str(uint8_t status)
{
    switch (status) {
        case 0:  return "CALIBRATING";
        case 1:  return "NORMAL";
        case 2:  return "WARNING";
        case 3:  return "CRITICAL";
        default: return "UNKNOWN";
    }
}

/* trigger bitmask, per tilt_detector.h on the sensor */
#define TRIGGER_ANGLE                          0x01U
#define TRIGGER_RATE                            0x02U
#define TRIGGER_LARGE_DEV                       0x04U
#define TRIGGER_ACCELERATION_GRAVITY_DEVIATION  0x08U

static void trigger_to_str(uint8_t trigger, char *out, size_t out_len)
{
    if (trigger == 0) {
        snprintf(out, out_len, "NONE");
        return;
    }

    out[0] = '\0';
    if (trigger & TRIGGER_ANGLE)                          strncat(out, "ANGLE ", out_len - strlen(out) - 1);
    if (trigger & TRIGGER_RATE)                           strncat(out, "RATE ", out_len - strlen(out) - 1);
    if (trigger & TRIGGER_LARGE_DEV)                      strncat(out, "LARGE_DEV ", out_len - strlen(out) - 1);
    if (trigger & TRIGGER_ACCELERATION_GRAVITY_DEVIATION) strncat(out, "GRAVITY_DEV ", out_len - strlen(out) - 1);
}

// char *json_builder_build_advert(const ble_addr_t *addr, int8_t rssi,
//                                  uint8_t alarm, uint8_t src_deg,
//                                  uint8_t vel_x100, uint8_t trigger)
// {
//     char addr_str[18];
//     addr_to_str(addr, addr_str, sizeof(addr_str));

//     char trigger_str[64];
//     trigger_to_str(trigger, trigger_str, sizeof(trigger_str));

//     cJSON *root = cJSON_CreateObject();
//     if (root == NULL) {
//         return NULL;
//     }

//     cJSON_AddStringToObject(root, "source", "advert");
//     cJSON_AddStringToObject(root, "address", addr_str);
//     cJSON_AddNumberToObject(root, "rssi", rssi);
//     cJSON_AddStringToObject(root, "status", status_to_str(alarm));
//     cJSON_AddNumberToObject(root, "deviation_deg", src_deg);
//     cJSON_AddNumberToObject(root, "rate_deg_per_h", vel_x100 / 100.0);
//     cJSON_AddStringToObject(root, "trigger", trigger_str);

//     char *out = cJSON_PrintUnformatted(root);
//     cJSON_Delete(root);
//     return out;
// }

// char *json_builder_build_gatt(const ble_addr_t *addr, const gatt_landslide_data_t *data)
// {
//     char addr_str[18];
//     addr_to_str(addr, addr_str, sizeof(addr_str));

//     cJSON *root = cJSON_CreateObject();
//     if (root == NULL) 
//     {
//         return NULL;
//     }

//     cJSON_AddStringToObject(root, "source", "gatt");
//     cJSON_AddStringToObject(root, "address", addr_str);

//     if (data->has_alert_status) 
//     {
//         char trigger_str[64];
//         trigger_to_str(data->trigger, trigger_str, sizeof(trigger_str));

//         cJSON_AddStringToObject(root, "status", status_to_str(data->alarm));
//         cJSON_AddStringToObject(root, "trigger", trigger_str);
//     }
//     if (data->has_tilt_data) 
//     {
//         cJSON_AddNumberToObject(root, "deviation_deg", data->dev_x100 / 100.0);
//         cJSON_AddNumberToObject(root, "rate_deg_per_h", data->vel_x100 / 100.0);
//     }
//     if (data->has_raw_imu) 
//     {
//         cJSON_AddNumberToObject(root, "accel_x", data->accel_x);
//         cJSON_AddNumberToObject(root, "accel_y", data->accel_y);
//         cJSON_AddNumberToObject(root, "accel_z", data->accel_z);
//         cJSON_AddNumberToObject(root, "gyro_x", data->gyro_x);
//         cJSON_AddNumberToObject(root, "gyro_y", data->gyro_y);
//         cJSON_AddNumberToObject(root, "gyro_z", data->gyro_z);
//         cJSON_AddNumberToObject(root, "imu_timestamp_ms", data->imu_timestamp_ms);
//         cJSON_AddNumberToObject(root, "imu_is_hist_burst", data->imu_is_hist_burst);
//     }

//     char *out = cJSON_PrintUnformatted(root);
//     cJSON_Delete(root);
//     return out;
// }

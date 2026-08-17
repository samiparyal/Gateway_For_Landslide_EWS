#include "gap_scan.h"
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_hs_adv.h"
#include "host/ble_uuid.h"
#include "nimble/ble.h"
#include "cJSON.h"
#include "server_comm.h"
#include "json_builder.h"
#include "gatt_client.h"

#define TAG "GAP_SCAN"

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);

void gap_scan_start(uint8_t own_addr_type)
{
    /* Legacy ble_gap_disc() only scans the LE 1M primary channel and will
       never see a Coded-PHY (long range) advertisement. ble_gap_ext_disc()
       scans both PHYs at once */
    struct ble_gap_ext_disc_params uncoded_params = {0};
    uncoded_params.passive = 1;
    uncoded_params.itvl = uncoded_params.window = 0x0010; // 10ms

    struct ble_gap_ext_disc_params coded_params = {0};
    coded_params.passive = 1;
    coded_params.itvl = coded_params.window = 0x0010; // 10ms

    /* duration=0, period=0 -> scan continuously.
       filter_duplicates=0 (debugging), filter_policy=0 (no accept-list), limited=0 (general discovery) */
    int rc = ble_gap_ext_disc(own_addr_type, 0, 0, 0, 0, 0,
                               &uncoded_params, &coded_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_ext_disc failed: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "Scanning started");
}

static void log_addr(const ble_addr_t *addr)
{
    ESP_LOGI(TAG, "  addr: %02x:%02x:%02x:%02x:%02x:%02x (type %d)",
             addr->val[5], addr->val[4], addr->val[3],
             addr->val[2], addr->val[1], addr->val[0],
             addr->type);
}

static bool adv_matches_ews(const struct ble_hs_adv_fields *fields)
{
    bool has_uuid = false;
    for (int i = 0; i < fields->num_uuids16; i++) {
        if (ble_uuid_u16(&fields->uuids16[i].u) == 0xFF01) {
            has_uuid = true;
            break;
        }
    }

    bool has_name = fields->name != NULL &&
                     fields->name_len == strlen("LANDSLIDE_EWS") &&
                     memcmp(fields->name, "LANDSLIDE_EWS", fields->name_len) == 0;

    /* both the 16-bit service UUID and the exact local name must be present */
    return has_uuid && has_name;
}


static sensor_state_t s_sensors[MAX_TRACKED_SENSORS];

static bool mfg_data_changed(const ble_addr_t *addr, const uint8_t *mfg_data, uint8_t mfg_data_len)
{
    sensor_state_t *slot = NULL;
    sensor_state_t *free_slot = NULL;

    for (int i = 0; i < MAX_TRACKED_SENSORS; i++) {
        if (s_sensors[i].occupied && ble_addr_cmp(&s_sensors[i].addr, addr) == 0) {
            slot = &s_sensors[i];
            break;
        }
        if (!s_sensors[i].occupied && free_slot == NULL) { //slot not assigned to the sensor
            free_slot = &s_sensors[i]; //that sensor slot/space is free
        }
    }

    if (slot == NULL) {
        if (free_slot == NULL) {
            ESP_LOGW(TAG, "Sensor table full (%d)", MAX_TRACKED_SENSORS);
            return true; /* no slot to track it in */
        }
        slot = free_slot;
        slot->addr = *addr;
        slot->occupied = true;
        slot->mfg_data_len = 0;
    }

    bool changed = mfg_data_len != slot->mfg_data_len ||
                   memcmp(mfg_data, slot->mfg_data, mfg_data_len) != 0;

    if (changed) {
        memcpy(slot->mfg_data, mfg_data, mfg_data_len);
        slot->mfg_data_len = mfg_data_len;
    }

    return changed;
}

/* trigger bitmask, per tilt_detector.h on the sensor */
#define TRIGGER_ANGLE                          0x01U
#define TRIGGER_RATE                            0x02U
#define TRIGGER_LARGE_DEV                       0x04U
#define TRIGGER_ACCELERATION_GRAVITY_DEVIATION  0x08U

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    /* imppp- ble_gap_ext_disc() reports arrive as BLE_GAP_EVENT_EXT_DISC
       (event->ext_disc), not BLE_GAP_EVENT_DISC (event->disc) */
    const uint8_t *adv_data;
    uint8_t adv_data_len;
    const ble_addr_t *adv_addr;
    int8_t adv_rssi;

    if (event->type == BLE_GAP_EVENT_DISC) {
        adv_data     = event->disc.data;
        adv_data_len = event->disc.length_data;
        adv_addr     = &event->disc.addr;
        adv_rssi     = event->disc.rssi;
    } else if (event->type == BLE_GAP_EVENT_EXT_DISC) {
        adv_data     = event->ext_disc.data;
        adv_data_len = event->ext_disc.length_data;
        adv_addr     = &event->ext_disc.addr;
        adv_rssi     = event->ext_disc.rssi;
    } else {
        return 0;
    }

    struct ble_hs_adv_fields fields;
    int rc = ble_hs_adv_parse_fields(&fields, adv_data, adv_data_len);

    if (rc != 0) {
        ESP_LOG_BUFFER_HEX(TAG, adv_data, adv_data_len); //malformed adv data
        return 0;
    }

    if (!adv_matches_ews(&fields)) {
        return 0; /* not our device */
    }

    if (g_gatt_connect_requested) {
        gatt_client_connect(adv_addr);
    }

    if (fields.mfg_data == NULL || fields.mfg_data_len < 6 ||
        !mfg_data_changed(adv_addr, fields.mfg_data, fields.mfg_data_len)) {
        return 0; /* nothing new to report from this peer */
    }

    log_addr(adv_addr);

    /* mfg_data layout: [company_id_lo][company_id_hi][alarm][src][vel_x100][trigger] */
    uint8_t status     = fields.mfg_data[2];
    uint8_t src_deg    = fields.mfg_data[3];
    uint8_t vel_x100   = fields.mfg_data[4];
    uint8_t trigger    = fields.mfg_data[5];

    const char *status_str;
    switch (status) {
        case 0:  status_str = "Calibrating...";       break;
        case 1:  status_str = "Ready - Normal State";  break;
        case 2:  status_str = "Warning";               break;
        case 3:  status_str = "CRITICAL";       break;
        default: status_str = "Unknown";                break;
    }

    char trigger_str[64] = {0};
    if (trigger == 0) {
        strcpy(trigger_str, "none ");
    } else {
        if (trigger & TRIGGER_ANGLE)                          strcat(trigger_str, "ANGLE ");
        if (trigger & TRIGGER_RATE)                           strcat(trigger_str, "RATE ");
        if (trigger & TRIGGER_LARGE_DEV)                      strcat(trigger_str, "LARGE_DEV ");
        if (trigger & TRIGGER_ACCELERATION_GRAVITY_DEVIATION) strcat(trigger_str, "GRAVITY_DEV ");
    }

    ESP_LOGI(TAG, "------------------------------------------------------------------------------------------------");
    ESP_LOGI(TAG, "  status=%u [%s]  |  dev=%u deg  |  rate=%.2f deg/h   |   trigger=0x%02x [%s]   ",
             status, status_str, src_deg, vel_x100 / 100.0f, trigger, trigger_str);
    ESP_LOGI(TAG, "------------------------------------------------------------------------------------------------");

    char *json = json_builder_build_advert(adv_addr, adv_rssi,
                                            status, src_deg, vel_x100, trigger);
    if (json != NULL) {
        server_comm_send_json(json);
        cJSON_free(json);
    }

    return 0;
}
#include "gatt_client.h"
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "cJSON.h"
#include "server_comm.h"
#include "json_builder.h"

static const char *TAG = "GATT_CLIENT";

/* UUIDs from the sensor firmware's landslide_service.h */
#define GATT_LANDSLIDE_SVC_UUID 0xFF01U
#define GATT_ALERT_STATUS_UUID  0xFF02U
#define GATT_TILT_DATA_UUID     0xFF03U
#define GATT_RAW_IMU_UUID       0xFF04U

typedef struct {
    bool active;
    uint16_t conn_handle;
    uint16_t svc_end_handle;
    uint16_t alert_status_val_handle;
    uint16_t tilt_data_val_handle;
    uint16_t raw_imu_val_handle;
    ble_addr_t peer_addr;
    gatt_landslide_data_t data;
} gatt_session_t;

static uint8_t s_own_addr_type;
static gatt_session_t s_session;
static gatt_session_end_cb_t s_session_end_cb;

static int gatt_gap_event_cb(struct ble_gap_event *event, void *arg);

void gatt_client_init(uint8_t own_addr_type)
{
    s_own_addr_type = own_addr_type;
    memset(&s_session, 0, sizeof(s_session));
}

void gatt_client_set_session_end_cb(gatt_session_end_cb_t cb)
{
    s_session_end_cb = cb;
}

static void end_session(void)
{
    memset(&s_session, 0, sizeof(s_session));
    if (s_session_end_cb != NULL) {
        s_session_end_cb();
    }
}

static uint16_t le16_to_uint16(const uint8_t *p) //little endian two consecutive bytes to plain uint16_t
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); 
}

static uint32_t le32_to_uint32(const uint8_t *p)
{
    return (uint32_t)(p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static void report(void)
{
    char *json = json_builder_build_gatt(&s_session.peer_addr, &s_session.data);
    if (json != NULL) {
        server_comm_send_json(json);
        cJSON_free(json);
    }
}

static void handle_alert_status_notify(struct os_mbuf *om)
{
    uint8_t buf[2];
    if (OS_MBUF_PKTLEN(om) < sizeof(buf)) {
        ESP_LOGW(TAG, "AlertStatus notify too short");
        return;
    }
    os_mbuf_copydata(om, 0, sizeof(buf), buf);

    s_session.data.alarm = buf[0];
    s_session.data.trigger = buf[1];
    s_session.data.has_alert_status = true;
    report();
}

static void handle_tilt_data_notify(struct os_mbuf *om)
{
    uint8_t buf[4];
    if (OS_MBUF_PKTLEN(om) < sizeof(buf)) {
        ESP_LOGW(TAG, "TiltData notify too short");
        return;
    }
    os_mbuf_copydata(om, 0, sizeof(buf), buf);

    s_session.data.dev_x100 = le16_to_uint16(&buf[0]);
    s_session.data.vel_x100 = le16_to_uint16(&buf[2]);
    s_session.data.has_tilt_data = true;
    report();
}

static void handle_raw_imu_notify(struct os_mbuf *om)
{
    uint8_t buf[21];
    if (OS_MBUF_PKTLEN(om) < sizeof(buf)) {
        ESP_LOGW(TAG, "RawImuSample notify too short");
        return;
    }
    os_mbuf_copydata(om, 0, sizeof(buf), buf);

    s_session.data.accel_x = (int16_t)le16_to_uint16(&buf[0]);
    s_session.data.accel_y = (int16_t)le16_to_uint16(&buf[2]);
    s_session.data.accel_z = (int16_t)le16_to_uint16(&buf[4]);
    s_session.data.gyro_x  = (int16_t)le16_to_uint16(&buf[6]);
    s_session.data.gyro_y  = (int16_t)le16_to_uint16(&buf[8]);
    s_session.data.gyro_z  = (int16_t)le16_to_uint16(&buf[10]);
    /* buf[12..15] duplicate TiltData's dev_x100/vel_x100 - not re-read here,
       TiltData is the authoritative source for those two fields */
    s_session.data.imu_timestamp_ms = le32_to_uint32(&buf[16]);
    s_session.data.imu_is_hist_burst = buf[20];
    s_session.data.has_raw_imu = true;
    report();
}

/*
 * This firmware declares exactly one descriptor (the CCCD) per
 * characteristic - see landslide_service.c, .descrs.descr_count = 1 for all
 * three. GATT handles are always allocated sequentially, so the CCCD handle
 * is always val_handle + 1 here; (Change) only if the sensor firmware ever adds another
 * descriptor ahead of the CCCD on one of these characteristics.
 */
static void subscribe_if_found(uint16_t conn_handle, uint16_t val_handle)
{
    if (val_handle == 0) {
        return;
    }
    uint16_t enable_notify = 1;
    int rc = ble_gattc_write_flat(conn_handle, val_handle + 1, &enable_notify,
                                   sizeof(enable_notify), NULL, NULL);  //writing 1 to its CCCD to enable notifications
    if (rc != 0) 
    {
        ESP_LOGW(TAG, "Failed to subscribe (val_handle=%d): rc=%d", val_handle, rc);
    }
}

static int on_chr_disc(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;

    if (error->status == 0 && chr != NULL) {
        uint16_t uuid16 = ble_uuid_u16(&chr->uuid.u);
        if (uuid16 == GATT_ALERT_STATUS_UUID) {
            s_session.alert_status_val_handle = chr->val_handle;
        } else if (uuid16 == GATT_TILT_DATA_UUID) {
            s_session.tilt_data_val_handle = chr->val_handle;
        } else if (uuid16 == GATT_RAW_IMU_UUID) {
            s_session.raw_imu_val_handle = chr->val_handle;
        }
    } else if (error->status != 0 && error->status != BLE_HS_EDONE) {
        ESP_LOGW(TAG, "Characteristic discovery error: %d", error->status);
    } else if (error->status == BLE_HS_EDONE) {
        /* characteristic discovery finished - subscribe to whichever we found */
        subscribe_if_found(conn_handle, s_session.alert_status_val_handle);
        subscribe_if_found(conn_handle, s_session.tilt_data_val_handle);
        subscribe_if_found(conn_handle, s_session.raw_imu_val_handle);

        if (s_session.alert_status_val_handle == 0 &&
            s_session.tilt_data_val_handle == 0 &&
            s_session.raw_imu_val_handle == 0) {
            ESP_LOGW(TAG, "No landslide characteristics found on peer");
        }
    }
    return 0;
}

static int on_svc_disc(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_svc *service, void *arg)
{
    (void)arg;

    if (error->status == 0 && service != NULL) {
        s_session.svc_end_handle = service->end_handle;
        ble_gattc_disc_all_chrs(conn_handle, service->start_handle,
                                 service->end_handle, on_chr_disc, NULL);
    } else if (error->status != 0 && error->status != BLE_HS_EDONE) {
        ESP_LOGW(TAG, "Service discovery error: %d", error->status);
    } else if (error->status == BLE_HS_EDONE && s_session.svc_end_handle == 0) {
        ESP_LOGW(TAG, "Landslide service (uuid 0x%04X) not found on peer, terminating",
                 GATT_LANDSLIDE_SVC_UUID);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    
    return 0;
}

static int gatt_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_session.active = true;
            s_session.conn_handle = event->connect.conn_handle;
            ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
            ble_gattc_disc_svc_by_uuid(event->connect.conn_handle,
                                        BLE_UUID16_DECLARE(GATT_LANDSLIDE_SVC_UUID),
                                        on_svc_disc, NULL);
        } else {
            ESP_LOGE(TAG, "Connect failed: status=%d", event->connect.status);
            end_session();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        end_session();
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        if (event->notify_rx.attr_handle == s_session.alert_status_val_handle) {
            handle_alert_status_notify(event->notify_rx.om);
        } else if (event->notify_rx.attr_handle == s_session.tilt_data_val_handle) {
            handle_tilt_data_notify(event->notify_rx.om);
        } else if (event->notify_rx.attr_handle == s_session.raw_imu_val_handle) {
            handle_raw_imu_notify(event->notify_rx.om);
        }
        return 0;

    default:
        return 0;
    }
}

int gatt_client_connect(const ble_addr_t *peer_addr)
{
    if (s_session.active) {
        ESP_LOGW(TAG, "Already connecting/connected, ignoring new connect request");
        return BLE_HS_EBUSY;
    }

    ble_gap_disc_cancel();

    memset(&s_session, 0, sizeof(s_session));
    s_session.peer_addr = *peer_addr;

    int rc = ble_gap_connect(s_own_addr_type, peer_addr, 30000, NULL,  gatt_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: %d", rc);
        end_session();
    }
    return rc;
}
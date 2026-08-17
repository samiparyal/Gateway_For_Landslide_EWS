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
#include "seedlink.h"
#include "freertos/task.h"  
#include <time.h>
#include <stdio.h>

static const char *TAG = "GATT_CLIENT";

/* UUIDs from the sensor firmware's landslide_service.h */
#define GATT_LANDSLIDE_SVC_UUID 0xFF01U
#define GATT_ALERT_STATUS_UUID  0xFF02U
#define GATT_TILT_DATA_UUID     0xFF03U
#define GATT_RAW_IMU_UUID       0xFF04U
#define GATT_CONTROL_UUID       0xFF05U

typedef struct {
    bool active;
    uint16_t conn_handle;
    uint16_t svc_end_handle;
    uint16_t alert_status_val_handle;
    uint16_t tilt_data_val_handle;
    uint16_t raw_imu_val_handle;
    uint16_t control_val_handle;
    ble_addr_t peer_addr;
    gatt_landslide_data_t data;
} gatt_session_t;

static uint8_t s_own_addr_type;
static gatt_session_t s_session;
static gatt_session_end_cb_t s_session_end_cb;

/* dedup */
static uint8_t s_last_alert_status[2];
static bool s_has_last_alert_status;

static uint8_t s_last_tilt_data[4];
static bool s_has_last_tilt_data;

/* excludes imu_timestamp_ms */
static uint8_t s_last_raw_imu[17]; /* buf[0..15] (accel/gyro) + buf[24] (hist_burst) */
static bool s_has_last_raw_imu;

static int gatt_gap_event_cb(struct ble_gap_event *event, void *arg);

//new: seedlink
static imu_payload_t s_seedlink_payload;
static uint16_t s_seedlink_idx = 0;
static uint32_t s_seedlink_sequence = 1;


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
    s_has_last_alert_status = false;
    s_has_last_tilt_data = false;
    s_has_last_raw_imu = false;
    if (s_session_end_cb != NULL) {
        s_session_end_cb();
    }
}

static uint16_t le16_to_uint16(const uint8_t *p) //little endian two consecutive bytes to plain uint16_t
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); 
}

static uint64_t le64_to_uint64(const uint8_t *p)
{
    uint64_t v = 0;
    for (int i = 7; i >= 0; i--) {
        v = (v << 8) | p[i];
    }
    return v;
}

static void report(void)
{
    /* Training mode: raw simplified stream only*/
    if (training_mode) {
        hide_gatt_logs = true;
        return;
    }
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

    /* state (buf[0]) specifically, not trigger (buf[1]) - TSS is capped at
       1 post/sec, so only a real state transition should post, not every
       trigger-reason bit flip (those can happen without the state changing). */
    bool state_changed = !s_has_last_alert_status || buf[0] != s_last_alert_status[0];

    s_session.data.alarm = buf[0];
    s_session.data.trigger = buf[1];
    s_session.data.has_alert_status = true;

    bool changed = !s_has_last_alert_status || memcmp(buf, s_last_alert_status, sizeof(buf)) != 0;
    if (changed) {
        memcpy(s_last_alert_status, buf, sizeof(buf));
        s_has_last_alert_status = true;
        if (!hide_gatt_logs) {
            report();
        }
    }

    if (state_changed && s_session.data.has_raw_imu && s_session.data.has_tilt_data) {
        server_comm_post_snapshot(s_session.data.alarm, s_session.data.trigger,
                                   s_session.data.dev_x100, s_session.data.vel_x100,
                                   s_session.data.accel_x, s_session.data.accel_y, s_session.data.accel_z,
                                   s_session.data.gyro_x, s_session.data.gyro_y, s_session.data.gyro_z);
    }
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

    bool changed = !s_has_last_tilt_data || memcmp(buf, s_last_tilt_data, sizeof(buf)) != 0;
    if (changed) {
        memcpy(s_last_tilt_data, buf, sizeof(buf));
        s_has_last_tilt_data = true;
        if (!hide_gatt_logs) {
            report();
        }
    }
}

static void handle_raw_imu_notify(struct os_mbuf *om)
{
    uint8_t buf[25];
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
       TiltData is the main source for those two fields */
    s_session.data.imu_timestamp_ms = le64_to_uint64(&buf[16]);   /* now 8 bytes (16..23), widened to avoid ~50-day wraparound */
    s_session.data.imu_is_hist_burst = buf[24];
    s_session.data.has_raw_imu = true;


    /* ---- feeding the seedlink accumulator - training mode only ---- */
    if (training_mode)
    {
        if (s_seedlink_idx == 0)
        {
            s_seedlink_payload.timestamp = time(NULL);
            s_seedlink_payload.sequence_number = s_seedlink_sequence++;
        }

        s_seedlink_payload.ax[s_seedlink_idx] = s_session.data.accel_x / 16384.0f;   /*4g / 65536 counts = 1/16384 g per count = 0.000061 g/LSB*/
        s_seedlink_payload.ay[s_seedlink_idx] = s_session.data.accel_y / 16384.0f;
        s_seedlink_payload.az[s_seedlink_idx] = s_session.data.accel_z / 16384.0f;
        s_seedlink_idx++;

        if (s_seedlink_idx >= IMU_MAX_SAMPLES)
        {
            xQueueSend(seedlink_get_queue(), &s_seedlink_payload, 0);
            s_seedlink_idx = 0;
        }
    }
    /* ---- end: seedlink accumulator ---- */

    if (training_mode) {
        printf("%llu,%d,%d,%d,%d,%d,%d\n",
               (unsigned long long)s_session.data.imu_timestamp_ms,
               s_session.data.accel_x, s_session.data.accel_y, s_session.data.accel_z,
               s_session.data.gyro_x,  s_session.data.gyro_y,  s_session.data.gyro_z);
        return;
    }

    /* compare accel/gyro (buf[0..15]) + hist_burst (buf[24]) only - timestamp
       (buf[16..23]) is excluded, see s_last_raw_imu declaration */
    uint8_t key[17];
    memcpy(key, buf, 16);
    key[16] = buf[24];

    bool changed = !s_has_last_raw_imu || memcmp(key, s_last_raw_imu, sizeof(key)) != 0;
    if (changed) {
        memcpy(s_last_raw_imu, key, sizeof(key));
        s_has_last_raw_imu = true;
        if (!hide_gatt_logs) {
            report();
        }
        /* TSS post moved to handle_alert_status_notify() - it needs to fire
           on state change, not on every changed accel sample (TSS is capped
           at 1 post/sec, RawIMU changes far more often than that). */
    }
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

//
static void send_control(uint16_t conn_handle)
{
    if (s_session.control_val_handle == 0) {
        return;
    }
    uint8_t payload[2] = {
        restart_sensor_node ? 1U : 0U,
        training_mode ? 1U : 0U
    };
    int rc = ble_gattc_write_flat(conn_handle, s_session.control_val_handle, payload, sizeof(payload), NULL, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to write control characteristic: rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Sent control: restart=%d training_mode=%d", payload[0], payload[1]);
        if (payload[0] == 1U) {
            restart_sensor_node = false;   /* one-shot: consumed, don't resend on the next reconnect */
        }
    }
}

static int on_chr_disc(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;

    if (error->status == 0 && chr != NULL) {
        uint16_t uuid16 = ble_uuid_u16(&chr->uuid.u);
        if (uuid16 == GATT_ALERT_STATUS_UUID) 
        {
            s_session.alert_status_val_handle = chr->val_handle;
        } 
        else if (uuid16 == GATT_TILT_DATA_UUID) 
        {
            s_session.tilt_data_val_handle = chr->val_handle;
        } 
        else if (uuid16 == GATT_RAW_IMU_UUID) 
        {
            s_session.raw_imu_val_handle = chr->val_handle;
        }
        else if (uuid16 == GATT_CONTROL_UUID) 
        {
            s_session.control_val_handle = chr->val_handle;
        }
    } 
    else if (error->status != 0 && error->status != BLE_HS_EDONE) 
    {
        ESP_LOGW(TAG, "Characteristic discovery error: %d", error->status);
    } 
    else if (error->status == BLE_HS_EDONE) 
    {
        /* characteristic discovery finished - subscribe to whichever we found */
        subscribe_if_found(conn_handle, s_session.alert_status_val_handle);
        subscribe_if_found(conn_handle, s_session.tilt_data_val_handle);
        subscribe_if_found(conn_handle, s_session.raw_imu_val_handle);

        send_control(conn_handle);

        if (s_session.alert_status_val_handle == 0 &&
            s_session.tilt_data_val_handle == 0 &&
            s_session.raw_imu_val_handle == 0) 
        {
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
    s_has_last_alert_status = false;
    s_has_last_tilt_data = false;
    s_has_last_raw_imu = false;

    static const struct ble_gap_conn_params cp = {
    .scan_itvl = 0x0010, .scan_window = 0x0010,
    .itvl_min = 6,   /* 7.5 ms */
    .itvl_max = 12,  /* 15 ms  */
    .latency = 0,
    .supervision_timeout = 400,  /* 4 s */
    .min_ce_len = 0, .max_ce_len = 0,
    };
 
    // int rc = ble_gap_ext_connect(s_own_addr_type, peer_addr, 5000,
    //                               BLE_GAP_LE_PHY_CODED_MASK,
    //                               NULL, NULL, &cp,
    //                               gatt_gap_event_cb, NULL);

    int rc = ble_gap_ext_connect(s_own_addr_type, peer_addr, 60000,
                              BLE_GAP_LE_PHY_1M_MASK,
                              &cp, NULL, NULL,
                              gatt_gap_event_cb, NULL);

    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_ext_connect failed: %d", rc);
        end_session();
    }
    return rc;
}
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

#define GATT_LANDSLIDE_SVC_UUID 0xFF01U
#define GATT_ALERT_STATUS_UUID  0xFF02U
#define GATT_TILT_DATA_UUID     0xFF03U
#define GATT_RAW_IMU_UUID       0xFF04U
#define GATT_CONTROL_UUID       0xFF05U

#define MAX_GATT_SESSIONS 2   /*increase if more sensors used*/

typedef struct {
    bool active;
    uint16_t conn_handle;
    uint16_t svc_end_handle;
    uint16_t alert_status_val_handle;
    uint16_t tilt_data_val_handle;
    uint16_t raw_imu_val_handle;
    uint16_t control_val_handle;
    ble_addr_t peer_addr;
    const sensor_id_t *sensor_id;
    gatt_landslide_data_t data;

    uint8_t last_alert_status[2];
    bool has_last_alert_status;
    uint8_t last_tilt_data[4];
    bool has_last_tilt_data;

    imu_payload_t seedlink_payload;
    uint16_t seedlink_idx;
    uint32_t seedlink_sequence;
} gatt_session_t;

static uint8_t s_own_addr_type;
static gatt_session_t s_sessions[MAX_GATT_SESSIONS];
static gatt_session_end_cb_t s_session_end_cb; 

static int gatt_gap_event_cb(struct ble_gap_event *event, void *arg);

void gatt_client_init(uint8_t own_addr_type)
{
    s_own_addr_type = own_addr_type;
    memset(s_sessions, 0, sizeof(s_sessions));
}

void gatt_client_set_session_end_cb(gatt_session_end_cb_t cb)
{
    s_session_end_cb = cb;
}

static gatt_session_t *session_alloc(const ble_addr_t *peer_addr)
{
    for (int i = 0; i < MAX_GATT_SESSIONS; i++) 
    {
        if (!s_sessions[i].active) 
        {
            memset(&s_sessions[i], 0, sizeof(s_sessions[i]));
            s_sessions[i].active = true;
            s_sessions[i].peer_addr = *peer_addr;
            s_sessions[i].sensor_id = server_comm_sensor_id_lookup(peer_addr->val);
            s_sessions[i].seedlink_sequence = 1;
            if (s_sessions[i].sensor_id == NULL) 
            {
                ESP_LOGW(TAG, "Unknown sensor address");
            }
            return &s_sessions[i];
        }
    }
    return NULL;
}

static void session_free(gatt_session_t *sess)
{
    memset(sess, 0, sizeof(*sess));
}

static uint16_t le16_to_uint16(const uint8_t *p)
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


static void report(gatt_session_t *sess)
{
    /* JSON build + log disabled */
    // char *json = json_builder_build_gatt(&sess->peer_addr, &sess->data);
    // if (json != NULL)
    // {
    //     server_comm_log_json(json);
    //     cJSON_free(json);and 
    // }

    ESP_LOGI(TAG, "[%s] alarm=%u trigger=%u dev=%u.%02udeg vel=%u.%02udeg/h accel=(%d,%d,%d) gyro=(%d,%d,%d)",
             sess->sensor_id ? sess->sensor_id->station : "UNKNOWN",
             sess->data.alarm, sess->data.trigger,
             sess->data.dev_x100 / 100, sess->data.dev_x100 % 100,
             sess->data.vel_x100 / 100, sess->data.vel_x100 % 100,
             sess->data.accel_x, sess->data.accel_y, sess->data.accel_z,
             sess->data.gyro_x, sess->data.gyro_y, sess->data.gyro_z);
}

static void handle_alert_status_notify(gatt_session_t *sess, struct os_mbuf *om)
{
    uint8_t buf[2];
    if (OS_MBUF_PKTLEN(om) < sizeof(buf)) 
    {
        ESP_LOGW(TAG, "AlertStatus notify too short");
        return;
    }
    os_mbuf_copydata(om, 0, sizeof(buf), buf); /* 0 = no offset, copy from the very first byte of the mbuf */

    bool state_changed = !sess->has_last_alert_status || buf[0] != sess->last_alert_status[0];

    sess->data.alarm = buf[0];
    sess->data.trigger = buf[1];
    sess->data.has_alert_status = true;

    bool changed = !sess->has_last_alert_status || memcmp(buf, sess->last_alert_status, sizeof(buf)) != 0;
    if (changed) 
    {
        memcpy(sess->last_alert_status, buf, sizeof(buf));
        sess->has_last_alert_status = true;
        if (!hide_gatt_logs) 
        {
            report(sess);
        }
    }

    if (state_changed) //&& sess->data.has_raw_imu && sess->data.has_tilt_data) 
    {
        server_comm_post_snapshot(sess->sensor_id ? sess->sensor_id->origin_code : "600",
                                   sess->data.alarm, sess->data.trigger,
                                   sess->data.dev_x100, sess->data.vel_x100,
                                   sess->data.accel_x, sess->data.accel_y, sess->data.accel_z,
                                   sess->data.gyro_x, sess->data.gyro_y, sess->data.gyro_z);
    }
}

static void handle_tilt_data_notify(gatt_session_t *sess, struct os_mbuf *om)
{
    uint8_t buf[4];
    if (OS_MBUF_PKTLEN(om) < sizeof(buf)) 
    {
        ESP_LOGW(TAG, "TiltData notify too short");
        return;
    }
    os_mbuf_copydata(om, 0, sizeof(buf), buf);

    sess->data.dev_x100 = le16_to_uint16(&buf[0]);
    sess->data.vel_x100 = le16_to_uint16(&buf[2]);
    sess->data.has_tilt_data = true;

    bool changed = !sess->has_last_tilt_data || memcmp(buf, sess->last_tilt_data, sizeof(buf)) != 0;
    if (changed)
    {
        memcpy(sess->last_tilt_data, buf, sizeof(buf));
        sess->has_last_tilt_data = true;
        /* multi-sensor: tilt deviation/rate drift alone shouldn't spam the
           log  */
        if (server_comm_known_sensor_count() <= 1 && !hide_gatt_logs)
        {
            report(sess);
        }
    }
}

static void handle_raw_imu_notify(gatt_session_t *sess, struct os_mbuf *om)
{
    uint8_t buf[25];
    if (OS_MBUF_PKTLEN(om) < sizeof(buf)) 
    {
        ESP_LOGW(TAG, "RawImuSample notify too short");
        return;
    }
    os_mbuf_copydata(om, 0, sizeof(buf), buf);

    sess->data.accel_x = (int16_t)le16_to_uint16(&buf[0]);
    sess->data.accel_y = (int16_t)le16_to_uint16(&buf[2]);
    sess->data.accel_z = (int16_t)le16_to_uint16(&buf[4]);
    sess->data.gyro_x  = (int16_t)le16_to_uint16(&buf[6]);
    sess->data.gyro_y  = (int16_t)le16_to_uint16(&buf[8]);
    sess->data.gyro_z  = (int16_t)le16_to_uint16(&buf[10]);
    sess->data.imu_timestamp_ms = le64_to_uint64(&buf[16]);
    sess->data.imu_is_hist_burst = buf[24];
    sess->data.has_raw_imu = true;

    if (training_mode)
    {
        seedlink_send(&sess->seedlink_payload, &sess->seedlink_idx, &sess->seedlink_sequence,
                                   sess->sensor_id, sess->data.accel_x, sess->data.accel_y, sess->data.accel_z);

        if (server_comm_known_sensor_count() <= 1)
        {
            /* single sensor: bench-debug CSV visibility, no UART flood risk */
            printf("%llu,%d,%d,%d,%d,%d,%d\n",
                   (unsigned long long)sess->data.imu_timestamp_ms,
                   sess->data.accel_x, sess->data.accel_y, sess->data.accel_z,
                   sess->data.gyro_x,  sess->data.gyro_y,  sess->data.gyro_z);
        }
    }

    /* raw IMU never triggers report() - accel/gyro noise changes almost
       every sample (esp. during calibration movement), so a dedup'd report()
       here just floods the console. Only handle_alert_status_notify() logs,
       on a real alarm/trigger change. */
}

static void subscribe_if_found(uint16_t conn_handle, uint16_t val_handle,
                                ble_gatt_attr_fn *cb, void *cb_arg)
{
    if (val_handle == 0) 
    {
        if (cb != NULL) cb(conn_handle, NULL, NULL, cb_arg);
        return;
    }
    uint16_t enable_notify = 1;
    int rc = ble_gattc_write_flat(conn_handle, val_handle + 1, &enable_notify,
                                   sizeof(enable_notify), cb, cb_arg);
    if (rc != 0)
    {
        ESP_LOGW(TAG, "Failed to subscribe (val_handle=%d): rc=%d", val_handle, rc);
        if (cb != NULL) cb(conn_handle, NULL, NULL, cb_arg);
    }
}

static void send_control(uint16_t conn_handle, gatt_session_t *sess)
{
    if (sess->control_val_handle == 0) 
    {
        return;
    }
    uint8_t payload[2] = 
    {
        restart_sensor_node ? 1U : 0U,
        training_mode ? 1U : 0U
    };
    int rc = ble_gattc_write_flat(conn_handle, sess->control_val_handle, payload, sizeof(payload), NULL, NULL);
    if (rc != 0)
    {
        ESP_LOGW(TAG, "Failed to write control characteristic: rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "Sent control: restart=%d training_mode=%d", payload[0], payload[1]);
        if (payload[0] == 1U) {
            restart_sensor_node = false;
        }
    }
}

static int on_last_subscribe_done(uint16_t conn_handle, const struct ble_gatt_error *error,
                                   struct ble_gatt_attr *attr, void *arg)
{
    (void)error;
    (void)attr;
    send_control(conn_handle, (gatt_session_t *)arg);
    return 0;
}

static int on_chr_disc(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_chr *chr, void *arg)
{
    gatt_session_t *sess = (gatt_session_t *)arg;

    if (error->status == 0 && chr != NULL) 
    {
        uint16_t uuid16 = ble_uuid_u16(&chr->uuid.u);
        if (uuid16 == GATT_ALERT_STATUS_UUID) 
        {
            sess->alert_status_val_handle = chr->val_handle;
        } 
        else if (uuid16 == GATT_TILT_DATA_UUID) 
        {
            sess->tilt_data_val_handle = chr->val_handle;
        } 
        else if (uuid16 == GATT_RAW_IMU_UUID) 
        {
            sess->raw_imu_val_handle = chr->val_handle;
        } 
        else if (uuid16 == GATT_CONTROL_UUID) 
        {
            sess->control_val_handle = chr->val_handle;
        }
    } 
    else if (error->status != 0 && error->status != BLE_HS_EDONE) 
    {
        ESP_LOGW(TAG, "Characteristic discovery error: %d", error->status);
    } 
    else if (error->status == BLE_HS_EDONE) 
    {
        subscribe_if_found(conn_handle, sess->alert_status_val_handle, NULL, NULL);
        subscribe_if_found(conn_handle, sess->tilt_data_val_handle, NULL, NULL);
        subscribe_if_found(conn_handle, sess->raw_imu_val_handle, on_last_subscribe_done, sess);

        if (sess->alert_status_val_handle == 0 &&
            sess->tilt_data_val_handle == 0 &&
            sess->raw_imu_val_handle == 0) 
        {
            ESP_LOGW(TAG, "No landslide characteristics found on peer");
        }
    }
    return 0;
}

static int on_svc_disc(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_svc *service, void *arg)
{
    gatt_session_t *sess = (gatt_session_t *)arg;

    if (error->status == 0 && service != NULL) 
    {
        sess->svc_end_handle = service->end_handle;
        ble_gattc_disc_all_chrs(conn_handle, service->start_handle,
                                 service->end_handle, on_chr_disc, sess);
    } 
    else if (error->status != 0 && error->status != BLE_HS_EDONE) 
    {
        ESP_LOGW(TAG, "Service discovery error: %d", error->status);
    } 
    else if (error->status == BLE_HS_EDONE && sess->svc_end_handle == 0) 
    {
        ESP_LOGW(TAG, "Landslide service (uuid 0x%04X) not found on peer, terminating",
                 GATT_LANDSLIDE_SVC_UUID);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }

    return 0;
}

static int gatt_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    gatt_session_t *sess = (gatt_session_t *)arg;

    switch (event->type) 
    {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0)
            {
                sess->conn_handle = event->connect.conn_handle;
                ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
                ble_gattc_disc_svc_by_uuid(event->connect.conn_handle,
                                            BLE_UUID16_DECLARE(GATT_LANDSLIDE_SVC_UUID),
                                            on_svc_disc, sess);
            }
            else
            {
                ESP_LOGE(TAG, "Connect failed: status=%d", event->connect.status);
                session_free(sess);
            }
            /* connect procedure has resolved (success or fail) - radio is
               free again, safe to resume scanning for other sensors here.*/
            if (s_session_end_cb != NULL)
            {
                s_session_end_cb();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            session_free(sess);
            if (s_session_end_cb != NULL) 
            {
                s_session_end_cb();
            }
            return 0;

        case BLE_GAP_EVENT_NOTIFY_RX:
            if (event->notify_rx.attr_handle == sess->alert_status_val_handle) 
            {
                handle_alert_status_notify(sess, event->notify_rx.om);
            } 
            else if (event->notify_rx.attr_handle == sess->tilt_data_val_handle) 
            {
                handle_tilt_data_notify(sess, event->notify_rx.om);
            } 
            else if (event->notify_rx.attr_handle == sess->raw_imu_val_handle) 
            {
                handle_raw_imu_notify(sess, event->notify_rx.om);
            }
            return 0;

        default:
            return 0;
    }
}

int gatt_client_connect(const ble_addr_t *peer_addr)
{
    for (int i = 0; i < MAX_GATT_SESSIONS; i++) 
    {
        if (s_sessions[i].active && ble_addr_cmp(&s_sessions[i].peer_addr, peer_addr) == 0) 
        {
            return BLE_HS_EALREADY;
        }
    }

    gatt_session_t *sess = session_alloc(peer_addr);
    if (sess == NULL) 
    {
        ESP_LOGW(TAG, "All %d GATT session slots in use, dropping connect", MAX_GATT_SESSIONS);
        return BLE_HS_ENOMEM;
    }

    ble_gap_disc_cancel();

    static const struct ble_gap_conn_params cp =
    {
        /* 25% duty scan  */
        .scan_itvl = 0x0060, .scan_window = 0x0015, //scan 15 ms out of every 60 ms
        .itvl_min = 6,   //Connection interval range, in units of 1.25ms: 6 × 1.25ms = 7.5ms
        .itvl_max = 12, //12 × 1.25ms = 15ms
        .latency = 0,
        .supervision_timeout = 400,
        .min_ce_len = 0, .max_ce_len = 0,
    };

    int rc = ble_gap_ext_connect(s_own_addr_type, peer_addr, 60000,
                              BLE_GAP_LE_PHY_1M_MASK,
                              &cp, NULL, NULL,
                              gatt_gap_event_cb, sess);

    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_ext_connect failed: %d", rc);
        session_free(sess);
        /* connect never actually started - nothing in flight, safe to
           resume scanning immediately. If rc==0, scanning resumes later, in
           BLE_GAP_EVENT_CONNECT once the connect procedure resolves. */
        if (s_session_end_cb != NULL)
        {
            s_session_end_cb();
        }
    }
    return rc;
}
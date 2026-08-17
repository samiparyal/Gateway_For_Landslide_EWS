#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Must match the sensor node's LSM6DSV16X full-scale config exactly - see
   lsm6dsv16x_xl_full_scale_set() / lsm6dsv16x_gy_full_scale_set() calls in
   the node's Core/Src/imu.c*/
#define NODE_ACCEL_FULL_SCALE_G     2      /* node: LSM6DSV16X_2g */
#define NODE_GYRO_FULL_SCALE_DPS    1000   /* node: LSM6DSV16X_1000dps */

/* Derived sensitivities (g or dps per LSB) - LSM6DSV16X sensitivity scales
   linearly with full-scale range: 0.061 mg/LSB per +-2g of range, 35
   mdps/LSB per +-1000dps of range */
#define ACCEL_G_PER_LSB       (0.000061f * ((float)NODE_ACCEL_FULL_SCALE_G / 2.0f))
#define GYRO_DPS_PER_LSB      (0.035f    * ((float)NODE_GYRO_FULL_SCALE_DPS / 1000.0f))

extern bool g_gatt_connect_requested;
extern bool hide_gatt_logs;
extern bool training_mode;
extern bool restart_sensor_node; 

void server_comm_init(void);

void server_comm_handle_command(const char *cmd_str);

void server_comm_send_json(const char *json_str);

/*
 * Posts one full state snapshot to the TSS Data Import API (POST /import) as
 * ten observations
 */
void server_comm_post_snapshot(uint8_t status, uint8_t trigger,
                                uint16_t dev_x100, uint16_t vel_x100,
                                int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz);

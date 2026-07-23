#pragma once

#include <stdint.h>
#include "host/ble_gap.h"
#include "gatt_client.h"

/*
 * Builds a JSON string describing a matched sensor advertisement.
 */
char *json_builder_build_advert(const ble_addr_t *addr, int8_t rssi,
                                 uint8_t alarm, uint8_t src_deg,
                                 uint8_t vel_x100, uint8_t trigger);

/*
 * Builds a JSON string bundling everything currently known from an active
 * GATT session
 */
char *json_builder_build_gatt(const ble_addr_t *addr,
                               const gatt_landslide_data_t *data);

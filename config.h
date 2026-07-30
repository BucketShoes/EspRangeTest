#pragma once
#include <stdint.h>

// ---- Identity / naming ----
#define DEVICE_NAME_PREFIX "ESPRT-"

// ---- BLE ----
// Custom 128-bit UUIDs for this project's GATT service.
#define BLE_SVC_UUID       "9c7a0001-1b2c-4a7e-9a1e-5f6b2c3d4e5f"
#define BLE_CHAR_OWN_UUID  "9c7a0002-1b2c-4a7e-9a1e-5f6b2c3d4e5f"  // notify: this board's BoardSnapshot
#define BLE_CHAR_PEER_UUID "9c7a0003-1b2c-4a7e-9a1e-5f6b2c3d4e5f"  // notify: peer's BoardSnapshot (relayed via ESP-NOW)
#define BLE_CHAR_INFO_UUID "9c7a0004-1b2c-4a7e-9a1e-5f6b2c3d4e5f"  // read: board id/name string

#define BLE_SNAPSHOT_PERIOD_MS 1000
#define BLE_RSSI_POLL_MS       300
#define BLE_SCAN_KICK_MS       5000
#define BLE_SCAN_DURATION_S    3

// ---- ESP-NOW ----
#define ESPNOW_MAGIC       0x54524553UL  // "SERT" tag, used to ignore stray packets
#define ESPNOW_TX_PERIOD_MS 250
#define ESPNOW_CHANNEL      1            // fixed channel shared by softAP + STA on both boards

// ---- HTTP / softAP ----
#define HTTP_AP_PASSWORD ""  // open AP - keep field setup simple

// ---- TX power ----
#define BLE_TX_POWER_DBM 9
#define WIFI_TX_POWER_DBM_QUARTER 78  // esp_wifi_set_max_tx_power units are 0.25dBm steps; 78 = ~19.5dBm (near max)

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
#define BLE_CHAR_HELLO_UUID "9c7a0005-1b2c-4a7e-9a1e-5f6b2c3d4e5f"  // write: sender's boardId, used by the peripheral side to positively identify the peer board vs. the phone

#define BLE_SNAPSHOT_PERIOD_MS 500  // 2Hz
#define BLE_RSSI_POLL_MS       150
// Advertising interval, in 0.625ms units (NimBLE/HCI convention) - 20-30ms is the
// standard "fast" connectable interval. Left unset, NimBLE-Arduino's default is 0
// ("use stack default"), which is ambiguous - set explicitly instead. A few units of
// per-boot random jitter get added on top (see ble_link.cpp) so two otherwise-identical
// boards don't run perfectly-synchronized advertising timing.
#define BLE_ADV_INTERVAL_MIN 32  // 32 * 0.625ms = 20ms
#define BLE_ADV_INTERVAL_MAX 48  // 48 * 0.625ms = 30ms
#define BLE_ADV_INTERVAL_JITTER 8

// Connection interval, in 1.25ms units (different HCI convention from advertising).
#define BLE_CONN_INTERVAL_MIN 24  // 24 * 1.25ms = 30ms
#define BLE_CONN_INTERVAL_MAX 40  // 40 * 1.25ms = 50ms
// Supervision timeout, in 10ms units - how long a connection tolerates zero successful
// packet exchange before the controller gives up on it. Set to the BLE spec max (32s):
// the connection is established over legacy PHY then upgraded to Coded S=8, so if it's
// going to survive a dip at the edge of range, it needs to hold on through the fade
// rather than time out and force re-establishing from scratch.
#define BLE_CONN_SUPERVISION_TIMEOUT 3200  // 3200 * 10ms = 32s

// Both boards always try to dial the peer (no elected "central" board) - a cooldown
// with jitter between attempts keeps a peer that's out of range from being hammered
// with reconnect attempts, and keeps the two boards' attempts from staying in lockstep.
#define BLE_CONNECT_RETRY_MS 4000
#define BLE_CONNECT_RETRY_JITTER_MS 2000

// ---- ESP-NOW ----
#define ESPNOW_MAGIC       0x54524553UL  // "SERT" tag, used to ignore stray packets
#define ESPNOW_TX_PERIOD_MS 250
#define ESPNOW_CHANNEL      1            // fixed channel shared by softAP + STA on both boards

// ---- HTTP / softAP ----
#define HTTP_AP_PASSWORD ""  // open AP - keep field setup simple

// ---- TX power ----
#define BLE_TX_POWER_DBM 9
#define WIFI_TX_POWER_DBM_QUARTER 78  // esp_wifi_set_max_tx_power units are 0.25dBm steps; 78 = ~19.5dBm (near max)

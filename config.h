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
// write (1 byte): 0 = LR mode (ESP-NOW LR + BLE Coded test, default at boot), 1 = WiFi/FTM
// mode (STA protocol forced to 11B only, ESP-NOW TX paused, AP answers FTM requests) -
// readable from the phone's page even when WiFi itself is misbehaving, since it's BLE.
#define BLE_CHAR_MODE_UUID "9c7a0005-1b2c-4a7e-9a1e-5f6b2c3d4e5f"

// Hardcoded identity for both boards - only two of these exist for this project, so
// there's no reason to discover the peer's BLE address at runtime (via scanning, or via
// an ESP-NOW-relayed handshake, both of which were real sources of flakiness). Each
// board compares its own MAC (identityMac()) against these two constants at boot to
// figure out which one it is and, by elimination, which one the peer is - then any
// incoming connection can be positively identified as "the peer" the instant it
// connects, just by comparing its address, no handshake or timing dependency at all.
// Get each board's exact MAC from its own boot log line (main.cpp prints it).
#define BOARD_MAC_3EFE { 0x58, 0xE6, 0xC5, 0xDF, 0x3E, 0xFE }
#define BOARD_MAC_63CA { 0x20, 0x6E, 0xF1, 0x12, 0x63, 0xCA }



#define BLE_SNAPSHOT_PERIOD_MS 500  // 2Hz
#define BLE_RSSI_POLL_MS       150
// Advertising interval, in 0.625ms units (NimBLE/HCI convention) - 20-30ms is the
// standard "fast" connectable interval. Left unset, NimBLE-Arduino's default is 0
// ("use stack default"), which is ambiguous - set explicitly instead. A few units of
// per-boot random jitter get added on top (see ble_link.cpp) so two otherwise-identical
// boards don't run perfectly-synchronized advertising timing.
// Slowed from the usual 20-30ms "fast" connectable interval - this radio also has to
// carry ESP-NOW (LR or 11B, depending on mode) and, in WiFi/FTM mode, FTM's own
// time-critical burst exchange; frequent BLE advertising is airtime this project can
// currently spare less of than a typical BLE-only device.
#define BLE_ADV_INTERVAL_MIN 800  // 800 * 0.625ms = 500ms
#define BLE_ADV_INTERVAL_MAX 960  // 960 * 0.625ms = 600ms
#define BLE_ADV_INTERVAL_JITTER 32  // 32 * 0.625ms = 20ms

// Connection interval, in 1.25ms units (different HCI convention from advertising).
// Slowed for the same shared-radio reason as BLE_ADV_INTERVAL above, but not as far -
// this governs ongoing traffic for the whole life of a connection (a bigger, more
// persistent airtime cost than advertising, which stops once connected), but BLE_RSSI_POLL_MS
// (150ms) and BLE_SNAPSHOT_PERIOD_MS (500ms, the live relay's actual cadence) both get
// capped at whatever this is - pushing it past those would bottleneck the data this rig
// exists to collect. ~3x slower than the previous 30-50ms, not the ~20x used for
// advertising.
#define BLE_CONN_INTERVAL_MIN 80   // 80 * 1.25ms = 100ms
#define BLE_CONN_INTERVAL_MAX 120  // 120 * 1.25ms = 150ms
// Supervision timeout, in 10ms units - how long a connection tolerates zero successful
// packet exchange before the controller gives up on it. Needs to be long enough to ride
// out a real fade at the edge of range during a field walk without forcing a full
// reconnect (which could mean walking back into range just to re-pair) - 16s.
#define BLE_CONN_SUPERVISION_TIMEOUT 3000  // 3000 * 10ms = 30s

// Both boards always try to dial the peer (no elected "central" board) - a cooldown
// with jitter between attempts keeps a peer that's out of range from being hammered
// with reconnect attempts, and keeps the two boards' attempts from staying in lockstep.
#define BLE_CONNECT_RETRY_MS 4000
#define BLE_CONNECT_RETRY_JITTER_MS 2000

// ---- ESP-NOW ----
#define ESPNOW_MAGIC       0x54524553UL  // "SERT" tag, used to ignore stray packets
#define ESPNOW_TX_PERIOD_MS 250
#define ESPNOW_CHANNEL      1            // fixed channel shared by softAP + STA on both boards
#define PDR_WINDOW_MS       6000         // PDR reset period (see stats.h - RollingLink - pending a rolling-window rewrite)

// ---- HTTP / softAP ----
#define HTTP_AP_PASSWORD ""  // open AP - keep field setup simple

// ---- TX power ----
#define BLE_TX_POWER_DBM 9
#define WIFI_TX_POWER_DBM_QUARTER 78  // esp_wifi_set_max_tx_power units are 0.25dBm steps; 78 = ~19.5dBm (near max)

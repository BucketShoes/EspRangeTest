// Shared bits. Deliberately small and dumb - this is a throwaway range tester, not a
// framework. Everything is one global table printed over serial.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"

// Log a failing init call instead of aborting on it. ESP_ERROR_CHECK turns any one bad
// return into a panic-and-reboot, which on a board with three radios means a boot loop that
// tells you nothing about which radio was unhappy. A range tester with two working radios is
// still useful; a rebooting one is not.
#define RT_TRY(tag, call)                                                        \
    do {                                                                         \
        const esp_err_t _e = (call);                                             \
        if (_e != ESP_OK) {                                                      \
            ESP_LOGE(tag, "%s -> %s", #call, esp_err_to_name(_e));               \
        }                                                                        \
    } while (0)

enum {
    CH_ESPNOW = 0,
    CH_BLE_ADV,   // extended advertising, coded PHY, S=8 requested
    CH_154,       // raw 802.15.4 - same PHY as Thread/Zigbee, so same range
    CH_COUNT
};

extern const char *rt_chan_name[CH_COUNT];

#define RT_MAGIC     0x5254
#define RT_MAX_PEERS 6

// 8 bytes. All nodes are little-endian ESP32s, so a packed struct straight onto the wire is
// fine - no hand serialisation needed.
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t  node;   // low byte of the sender's MAC
    int8_t   txdbm;
    uint32_t seq;    // per (sender, channel)
} rt_pkt_t;

uint8_t  rt_node_id(void);
uint32_t rt_ms(void);

// Which radios are currently allowed to transmit. 0 = all of them; otherwise only
// channel (solo-1), so one radio can be tested without the others competing for the antenna.
extern volatile int g_solo;
bool rt_tx_enabled(int chan);
void rt_set_solo(int solo);

// Fill in a packet ready to send on this channel, advancing that channel's sequence number.
void rt_fill(rt_pkt_t *p, int chan, int8_t txdbm);

// Record a reception. Ignores anything that is not ours.
void rt_rx(const void *data, int len, int chan, int8_t rssi, uint8_t lqi);

// Print the whole table.
void rt_report(void);

// Current state as short CSV text lines, for the web UI. Text rather than a binary format
// on purpose: it is the same information the serial report shows, it is readable in a BLE
// debugging app, and it needs no decoder on the browser side.
//   S,<node>,<uptime_s>,<solo>
//   R,<peer>,<chan>,<rssi>,<avg>,<min>,<max>,<pdr_now>,<pdr_all>,<rx>,<miss>,<age_ms>
#define RT_LINE_MAX 72
int rt_snapshot_lines(char out[][RT_LINE_MAX], int max);

void rt_espnow_start(void);
void rt_ble_start(void);
void rt_154_start(void);

// Connectable legacy advert + GATT service, so a phone browser can see the numbers.
// Legacy because Chrome's scanner cannot see extended or coded adverts at all.
void rt_ui_on_sync(uint8_t own_addr_type);
void rt_ui_init(void);
void rt_ui_notify(void);

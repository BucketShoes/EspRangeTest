// Shared bits. Deliberately small and dumb - this is a throwaway range tester, not a
// framework. Everything is one global table printed over serial.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"

// Bring-up ladder. Set with -DRT_STAGE=n in platformio.ini. Raise it one step at a time,
// flashing after each, so a board that will not boot names the layer that broke it instead
// of leaving the whole thing suspect.
//
//   0  heartbeat only - proves toolchain, partition table, flash config, console
//   1  + wi-fi and esp-now
//   2  + ble coded beacon and scanner
//   3  + 802.15.4
//   4  + phone UI over BLE GATT        <- the finished thing
//
// Defaults to 0: a board that boots is worth more than one that does everything.
#ifndef RT_STAGE
#define RT_STAGE 0
#endif

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

// Low-contention mode values are 0..CH_COUNT (see g_lc below) plus one more: LC_WIFI_UI, a
// state that isn't "just one channel" like the others. It keeps ESP-NOW and the Wi-Fi AP on,
// forces BLE fully off, and forces LR off - the one mode where a phone must reach the board
// over Wi-Fi instead of BLE, which only works with LR off.
#define LC_WIFI_UI (CH_COUNT + 1)
#define LC_COUNT   (CH_COUNT + 2)  // total states the button cycles through

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

// ms with +/-5% of randomness. Every transmit loop delays by this rather than by a constant:
// two boards running the same firmware would otherwise sit at exactly the same period and
// either collide on the air every time or never, so the loss figure would be measuring the
// timers rather than the range. It also keeps a channel's transmits from landing in permanent
// lockstep with the phone link's connection events.
uint32_t rt_jitter_ms(uint32_t ms);

// Low-contention mode: which channel gets the antenna mostly to itself. 0 = all of them
// (normal operation); otherwise only channel (g_lc-1) transmits, and everything else that
// might touch the radio on its own schedule backs off too - the Wi-Fi driver stops outright,
// the BLE coded-PHY scanner stops, and the phone-UI advert/connection/notify cadence all slow
// down. The point is a clean per-radio baseline, not just "this channel stops sending packets
// while everyone else keeps using the antenna."
//
// rt_tx_enabled() below answers only the narrow question the three tx loops ask. It is not the
// whole of what a mode means - see rt_apply_lc_radios(), which is - and reading it as though
// it were is how "low contention" spent several revisions isolating nothing at all.
extern volatile int g_lc;
bool rt_tx_enabled(int chan);
void rt_set_lc(int lc);

// LR (long range PHY) affects Wi-Fi and ESP-NOW together and needs a full radio reinit to
// change - see wifi_apply() in main.c. Merely having WIFI_PROTOCOL_LR in the protocol
// list blinds a phone to the SoftAP even though ESP-to-ESP links keep working, which is why
// this is a deliberate, infrequent, operator-commanded toggle (long-hold GPIO9) rather than
// part of the low-contention cycle - see LC_WIFI_UI above for the one mode that forces it off.
extern volatile bool g_lr;
void rt_set_lr(bool lr);

// Command bytes accepted on the GATT command characteristic (rt_ui.c). Low values are a
// low-contention mode; the high-bit values are LR, which used to be a super-long button hold
// and is not any more - the button is down to two gestures and restore has to be one of them
// (see HOLD_MS in main.c). Putting LR here is safe precisely because holding the button undoes
// it: no command can strand the board that the physical control cannot take back.
//
// Values outside both ranges are ignored, so an older web UI that only ever sends 0..LC_COUNT-1
// keeps working unchanged.
#define RT_CMD_LR_OFF 0x80
#define RT_CMD_LR_ON  0x81

// ---- Transmit power ----------------------------------------------------------------------
//
// Raw dBm per channel, over each radio's real range, set at runtime.
//
// Not preset "levels": named levels that mapped to different dBm per radio made the channels
// non-comparable by construction, which is the one thing this instrument exists to avoid. A
// number is a number - ask all three for -6dBm and they are all at -6dBm.
//
// Per channel and settable together (chan == CH_COUNT), because both are real tests: BLE held
// low enough to keep a phone connected while 802.15.4 runs flat out, or every radio matched so
// the comparison is fair.
//
// Ranges are the hardware's, not a choice:
//   espnow/wifi   2 .. 20 dBm - esp_wifi_set_max_tx_power takes 0.25dBm units over [8,84], so
//                               2dBm really is the floor the API offers, and the hardware
//                               quantises further onto a fixed ladder (see esp_wifi.h).
//   ble_adv     -15 .. 20 dBm - controller levels, 3dB steps (ESP_PWR_LVL_N15 .. P20).
//   154         -15 .. 20 dBm - esp_ieee802154_set_txpower, 3dB steps.
//
// Every radio is asked, then **read back**, because all three quantise. rt_power_actual() is
// what came out, and is what goes into the packet and the report; rt_power_dbm() is what was
// asked for. When the two differ the radio rounded - that is information, not an error.
//
// Boards boot at each radio's minimum. That is the state to flash into while an antenna path
// is unproven: if RF is going somewhere it should not, minimum is where it does least harm,
// and a link that works at minimum proves the path far better than one that works at maximum.
typedef struct {
    int8_t min, max, step;  // dBm; step is the radio's own granularity
} rt_pwr_range_t;

const rt_pwr_range_t *rt_power_range(int chan);

// Two-byte command: { RT_CMD_PWR_SET + chan, (int8_t)dbm }. chan == CH_COUNT sets all three,
// each clamped to its own range - so "everything to -15" leaves Wi-Fi at its floor of 2.
#define RT_CMD_PWR_SET 0xB0

extern volatile int8_t g_pwr_dbm[CH_COUNT];
int8_t rt_power_dbm(int chan);      // requested
int8_t rt_power_actual(int chan);   // what the radio reported back
void   rt_set_power(int chan, int dbm);

// Called by each radio once it has set and read back its own power.
void rt_power_set_actual(int chan, int8_t dbm);

// Applied per radio rather than centrally: each needs a different call, and the BLE ones need
// their advertising instance reconfigured rather than a value poked, which cannot be done from
// the button task. Those two just mark themselves dirty and pick it up on their own cycle.
void rt_wifi_apply_power(void);
void rt_154_apply_power(void);
void rt_ble_apply_power(void);

// True once BLE's transmit power has actually been programmed, or if BLE is not running at
// all. on_sync is asynchronous, so app_main cannot assume it has happened yet - and the RF
// switch must not be powered until it has. See the antenna comment in main.c.
bool rt_ble_power_ready(void);

// Called by rt_set_lc() after every mode change, from whichever context asked for it (button
// task, or a BLE command write). Defined in main.c because that is the file owning the Wi-Fi
// driver - rt_stats.c has no business touching a radio directly. It applies everything a mode
// change means at the radio level, which is more than muting tx loops:
//   - stops the Wi-Fi driver outright in modes that isolate a non-Wi-Fi channel. Gating only
//     ESP-NOW's tx loop leaves the SoftAP beaconing every ~100ms and its receiver on
//     continuously, both of which outrank 802.15.4 in the coex arbiter - so the antenna stays
//     exactly as busy while the report claims the channel has been isolated.
//   - forces LR off for LC_WIFI_UI, the one mode where a phone must reach the SoftAP.
//
// Note what it deliberately does not do: touch anyone's coexistence priority. Isolation comes
// from radios being off or backed off, never from re-ranking the arbiter - see the long
// comment at the top of rt_154.c for why that is not the same thing.
void rt_apply_lc_radios(int lc);

// Whether the Wi-Fi driver is actually started right now, as opposed to merely not being
// asked to transmit. Printed in the report: the recurring bug in this project is a radio
// quietly still on while the numbers imply it is not.
bool rt_wifi_active(void);

// Shared by rt_set_lc() and rt_set_lr(): a mode or PHY change makes old RSSI/loss numbers
// incomparable to new ones, so both wipe the table the same way.
void rt_stats_reset(void);

// Wipe one channel only. A power change invalidates that channel's history and nothing else,
// so throwing away the other two channels' data alongside it would be gratuitous.
void rt_stats_reset_chan(int chan);

// Fill in a packet ready to send on this channel, advancing that channel's sequence number.
void rt_fill(rt_pkt_t *p, int chan, int8_t txdbm);

// The two halves of "did it actually transmit". Both are driven by the radio's own completion
// callback, not by what we asked for, so together they answer the only question that matters
// when a channel reads 100% loss: was it not heard, or did it never leave the antenna?
//
// Reporting attempts-minus-failures is not good enough. That makes "it worked" the default
// assumption for anything not explicitly reported broken, which is the same swallowed-failure
// trap as everywhere else in this project - so a confirmed success has to be counted as
// deliberately as a confirmed rejection.
void rt_tx_ok(int chan);
void rt_tx_failed(int chan);

// Record a reception. Ignores anything that is not ours.
void rt_rx(const void *data, int len, int chan, int8_t rssi, uint8_t lqi);

// Print the whole table.
void rt_report(void);

// Current state as short CSV text lines, for the web UI. Text rather than a binary format
// on purpose: it is the same information the serial report shows, it is readable in a BLE
// debugging app, and it needs no decoder on the browser side.
//   S,<node>,<uptime_s>,<lc>,<lr>,<pwr_espnow>,<pwr_ble_adv>,<pwr_154>
//   R,<peer>,<chan>,<rssi>,<avg>,<min>,<max>,<pdr_now>,<pdr_all>,<rx>,<miss>,<age_ms>
#define RT_LINE_MAX 72
int rt_snapshot_lines(char out[][RT_LINE_MAX], int max);

void rt_espnow_start(void);
void rt_ble_start(void);
void rt_154_start(void);

// Re-add the broadcast peer after the Wi-Fi driver has been stopped and restarted around a
// mode change. A no-op before rt_espnow_start() has run.
void rt_espnow_resume(void);


// Connectable legacy advert + GATT service, so a phone browser can see the numbers.
// Legacy because Chrome's scanner cannot see extended or coded adverts at all.
void rt_ui_on_sync(uint8_t own_addr_type);
void rt_ui_init(void);
void rt_ui_notify(void);

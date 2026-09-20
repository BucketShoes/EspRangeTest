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

// 32 bits, not 16.
//
// Every channel here is promiscuous by necessity - a BLE scanner sees every advert in the
// room, ESP-NOW sees every ESP-NOW frame, and 802.15.4 sees whatever is on channel 26 - so the
// magic is the only thing separating a measurement from somebody else's traffic. At 16 bits
// that is one chance in 65536 per foreign packet, which sounds small until you are parked in a
// room full of beacons for an hour: the false positives arrive at a steady trickle, they get
// counted as receptions, and a channel that is actually out of range reports that it is not.
//
// That is not a hypothetical. BLE was the channel that consistently "won" earlier range tests,
// and BLE is also the channel most exposed to it, because phones and fitness trackers emit
// manufacturer-specific adverts constantly. Any of those, of the right length, with the right
// two bytes, was recorded as a packet from a peer that does not exist.
//
// At 32 bits it is one in four billion, and combined with the exact-length check in rt_ble.c
// the false-accept rate stops being something that needs thinking about.
#define RT_MAGIC     0x9C7A5254u
#define RT_MAX_PEERS 6

// "pdr now": delivery ratio over a sliding window, kept per link as RT_PDR_BUCKETS equal slices
// of time. The window moves forward one slice at a time, so a reading covers between
// (BUCKETS-1)/BUCKETS and all of RT_PDR_WINDOW_MS depending on how far into the newest slice it
// is taken. More buckets = smoother slide, 8 bytes each per link.
//
// Time-based rather than counted in reports, so changing REPORT_MS does not change what the
// figure means. At 4 Hz a 1 s window moved in 25% steps; 10 s is 40 packets and 2.5% steps.
#define RT_PDR_WINDOW_MS 10000
#define RT_PDR_BUCKETS   10

// 12 bytes. All nodes are little-endian ESP32s, so a packed struct straight onto the wire is
// fine - no hand serialisation needed.
//
// node is the last three bytes of the sender's MAC, little-endian. It was one byte, and boards
// sharing a low MAC byte were merged into one peer. Every receiver checks the exact length, so
// firmware on either side of that change simply ignores the other's packets.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  node[3];
    int8_t   txdbm;
    uint32_t seq;    // per (sender, channel)
} rt_pkt_t;

static inline uint32_t rt_pkt_node(const rt_pkt_t *p)
{
    return p->node[0] | ((uint32_t)p->node[1] << 8) | ((uint32_t)p->node[2] << 16);
}

// 24 bits: mac[3] << 16 | mac[4] << 8 | mac[5]. Printed as six hex digits, matching the name.
uint32_t rt_node_id(void);

// "ESPRT-" + rt_node_id() in hex, for the SoftAP SSID and the BLE name.
const char *rt_node_name(void);

uint32_t rt_ms(void);

// ms with +/-5% of randomness, tick-quantised. Only the BLE advert refresh uses this now, and
// there the controller picks the actual advertising instants itself.
uint32_t rt_jitter_ms(uint32_t ms);

// Sleep a uniformly random time in [min_ms, max_ms], at microsecond resolution. The ESP-NOW
// and 802.15.4 transmit loops use this so that whether any one packet lands inside some
// receiver's listening window - the far board's BLE scan, a coex slot, a connection event -
// is a fresh coin toss every packet, never a fixed phase.
//
// Two things vTaskDelay(rt_jitter_ms()) got wrong for that:
//   - the tick is 10ms, so every transmit started on the same 10ms grid as every other
//     tick-woken task on the board, and "250 +/-12" was really three or four fixed delays;
//   - +/-5% moves the phase ~7ms per packet, so consecutive packets were strongly correlated:
//     one that missed a 200ms scan window left the next few likely to miss it too.
// Callers ask for +/-50% of their period: same mean rate, and each packet's phase against
// anything periodic up to one full period is independent of the last.
//
// Backed by an esp_timer, so the wake-up is off the tick grid. One per task.
typedef struct rt_sleeper rt_sleeper_t;
rt_sleeper_t *rt_sleeper_new(const char *name);
void rt_sleep_rand(rt_sleeper_t *s, uint32_t min_ms, uint32_t max_ms);

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
#define RT_CMD_ANT_INT 0x82
#define RT_CMD_ANT_EXT 0x83

// ---- Control-link PHY --------------------------------------------------------------------
//
// The phone connection is used two completely different ways, and they want opposite PHYs:
//
//   coded S=8 - the link itself is under test. How far can a phone stay connected? This is the
//               long-range option and the default, because it is also the safe one: a link
//               that is too slow still reaches.
//   2M        - the link is not under test at all; the phone is in a pocket next to the board
//               and is only a screen for some *other* channel's results. Here the report is
//               pure overhead stolen from the channel being measured, and 2M is the cheapest
//               way to carry it - roughly 16x less airtime per byte than coded S=8.
//
// Nothing in between. 1M and S=2 are compromises for a decision that does not need one.
//
// Defaults to coded, and the button restore puts it back - same rule as everywhere else here:
// no command may leave the board somewhere the physical control cannot reach. Selecting 2M and
// then walking out of range is a real way to lose the link, which is precisely why the restore
// gesture has to undo it.
#define RT_CMD_PHY_CODED 0x84
#define RT_CMD_PHY_2M    0x85

extern volatile bool g_conn_2m;
void rt_set_conn_phy(bool two_m);

// What the controller says the connection actually settled on, from
// BLE_GAP_EVENT_PHY_UPDATE_COMPLETE: 0 unknown, 1 = 1M, 2 = 2M, 3 = coded. Requested and
// achieved are both reported, for the same reason transmit power is - the phone can decline,
// and an 8x airtime difference is not something to assume went through.
uint8_t rt_conn_phy_actual(void);

// ---- Antenna selection -------------------------------------------------------------------
//
// The XIAO's RF switch has two ports: RF1 is the onboard ceramic chip antenna, RF2 is the U.FL
// connector. GPIO14 drives the switch's VCTL - low selects RF1, high RF2 - and GPIO3 powers
// the switch at all (see main.c).
//
// Always internal at boot, never remembered. Selecting an antenna that is not fitted takes the
// radio off the air completely, and that is not a state to wake up in: it would kill every
// control path at once, leaving only the button. Boot-to-internal means the worst a bad
// selection can cost is a power cycle.
//
// This is a UI command rather than a button gesture on purpose. The button has exactly two
// gestures - tap for the next mode, hold to restore - and a third would turn the hold into a
// timed window, which is precisely what the recovery gesture must never be.
extern volatile bool g_ant_ext;
void rt_set_antenna(bool external);

// ---- Transmit mute -----------------------------------------------------------------------
//
// ESP-NOW and 802.15.4 skip sending their test packets. That is all it does - receiving and
// everything reported about received packets is untouched. For carrying several listening
// boards together without them flooding each other.
//
// Only those two: BLE and the SoftAP carry the control paths.
//
// Not folded into rt_tx_enabled(), because rt_rx() also uses that to discard packets arriving
// on a channel outside the current mode. Off at boot; the button restore clears it.
#define RT_CMD_TX_UNMUTE 0x86
#define RT_CMD_TX_MUTE   0x87

extern volatile bool g_tx_mute;
void rt_set_tx_mute(bool mute);

// ---- Clearing the results ----------------------------------------------------------------
//
// Same wipe as ever, moved to a button.
//
// Every setting used to do it on the way past: power, mode, LR and antenna each cleared the
// table, on the grounds that numbers from before a change cannot be compared with numbers
// from after it. Sound in principle, backwards in practice - the board whose slider you moved
// is the *transmitter*, and the table it wiped was everything it had *received*, which that
// slider did not touch. With two boards under test, nudging one board's power threw away the
// other board's results mid-walk, and a walk cannot be repeated by standing still.
//
// So nothing clears itself now. The operator says when a measurement starts.
#define RT_CMD_STATS_RESET 0x88

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

// 802.15.4's own counters, for the periodic report. frames = everything the radio handed up,
// ours = what survived validation, coex = transmits the arbiter refused. Zeroed on targets
// without the radio.
void rt_154_counters(uint32_t *frames, uint32_t *ours, uint32_t *coex);
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

// Wipe the results table and the transmit counters. Driven by RT_CMD_STATS_RESET and nothing
// else - see that define for why no setting does it on the way past any more.
void rt_stats_reset(void);

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

// Noise floor is only reported by one of the three radios, so the others say so rather than
// pretending. ESP-NOW's rx_ctrl carries a measured noise_floor in dBm, which makes SNR real:
// rssi - noise. 802.15.4's frame info has rssi and lqi but no noise floor - lqi is the
// standard's own link-quality metric and is already reported. NimBLE's advert reports give
// rssi alone. Inventing a number for the two that do not measure it would make the columns
// look comparable when they are not.
#define RT_NOISE_NONE 127

// Record a reception. Ignores anything that is not ours.
void rt_rx(const void *data, int len, int chan, int8_t rssi, uint8_t lqi, int8_t noise);

// Why the board last reset, as a short word ("power-on", "BROWNOUT", "PANIC"...). Printed in
// every report, not once at boot: a board that resets mid-walk scrolls its startup banner past
// long before anyone looks, and "it came back on defaults" is not a diagnosis.
const char *rt_reset_reason(void);

// The same thing as a code, for the packed report. Kept in step with the name table in main.c;
// the page holds the matching list. 0 = unknown.
uint8_t rt_reset_code(void);

// Print the whole table.
void rt_report(void);

// ---- The report, packed binary ------------------------------------------------------------
//
// This used to be CSV text, one ATT notification per line, and the reasoning for that was
// sound as far as it went: same information as the serial report, readable in a BLE debugging
// app, no decoder needed in the browser. What it missed is that on coded PHY the cost of a
// report is dominated by the *number of notifications*, not by their size - each one pays a
// fresh preamble, access address and FEC block 1 before a single byte of payload, and at S=8
// that fixed cost is ~376us per packet before the payload's own 8us/bit.
//
// The text report had grown to 8 notifications (S + 3xT + X + 3xR) at 1Hz - the largest single
// consumer of airtime on the board, larger than any channel it was reporting on. Packed, the
// two-board case is 142 bytes in ONE notification. Same information, an eighth of the packets.
//
// Everything is little-endian and hand-serialised field by field - no packed structs on this
// wire, because the other end is JavaScript and a DataView has to agree with it byte for byte.
//
//   chunk header (4 bytes, every chunk)
//     u8  type    0x01 status chunk (status block, then as many rows as fit)
//                 0x02 rows-only continuation chunk
//     u8  gen     report generation; same across every chunk of one report, wraps at 256
//     u8  idx     chunk index within this report, from 0
//     u8  flags   bit0 = last chunk of this report
//
//   status block (83 bytes, only in a 0x01 chunk, immediately after the header)
//     u8  ver           RT_RPT_VER
//     u24 node          rt_node_id()
//     u8  lc
//     u8  state         bit0 lr, bit1 ant_ext, bit2 wifi_active, bit3 conn_2m requested,
//                       bits 4-5 conn PHY actually in use (0 unknown, 1 1M, 2 2M, 3 coded),
//                       bit6 tx_mute
//     u32 up_ms         board clock when the whole snapshot was taken; every age below is
//                       measured against it, so the page can put ages on the board's timebase
//                       instead of on arrival times
//     i8  pwr[3]        achieved dBm per channel, CH_ order
//     u8  rst           reset reason, as a code - see rt_reset_code()
//     u32 heap_free
//     u32 heap_min
//     u32 rx154_frames
//     u32 rx154_ours
//     u32 coex_refused
//     u8  n_rows        rows in the whole report, across all chunks
//     tx[3], 16 bytes each, CH_ order:
//       u32 queued, u32 ok, u32 rejected, u16 offmode_rx, u16 offmode_age_s
//
//   row record (23 bytes, packed end to end after whichever block precedes them)
//     u24 peer, u8 chan
//     i8  rssi_last, i8 rssi_avg, i8 rssi_min, i8 rssi_max
//     i8  pdr_now, i8 pdr_all      -1 where there is no data yet
//     u32 rx, u32 missed
//     u16 age_ds                   0.1s units, clamped at 6553.5s. v2 sent ms here, which
//                                  clamped at 65.5s - a silent link sat at "65s ago" forever
//     i8  snr                      -128 where the radio measures no noise floor
//     u8  lqi                      802.15.4 only
//     i8  peer_txdbm               what the far end said it transmitted at
//
// Counters stay u32 rather than being squeezed: at 4 packets/s a u24 wraps in seven weeks and
// the six bytes saved are not worth a counter that silently rolls over mid-test.
// 35 bytes of status + 3 x 16 bytes of per-channel tx accounting. Checked against the
// serialiser at runtime rather than trusted - see rt_snapshot_chunk().
#define RT_RPT_VER      4
#define RT_RPT_HDR      4
#define RT_RPT_STATUS   83
#define RT_RPT_ROW      23
#define RT_RPT_TYPE_STATUS 0x01
#define RT_RPT_TYPE_ROWS   0x02

// Biggest chunk we will ever build. ATT MTU is requested at 247, so 244 bytes of payload; the
// real limit is read back per connection with ble_att_mtu() and this is only the ceiling.
#define RT_RPT_CHUNK_MAX 244

// Serialise the next chunk of a report into out. Call with *state zeroed to start a report,
// then keep calling until it returns 0; each call emits one notification's worth. Returns the
// number of bytes written.
//
// cap is the largest chunk this connection can carry (ble_att_mtu() - 3, capped at
// RT_RPT_CHUNK_MAX). gen is the report generation, chosen by the caller once per report.
typedef struct {
    int row_next;   // index of the next row to emit
    int chunk;      // chunks emitted so far
    int started;    // status block has gone out
} rt_rpt_state_t;

int rt_snapshot_chunk(uint8_t *out, int cap, uint8_t gen, rt_rpt_state_t *st);

// How many rows this report will contain. Needed up front, because n_rows goes in the status
// block and the status block goes out first.
int rt_snapshot_rows(void);

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

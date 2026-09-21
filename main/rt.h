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
#define RT_PDR_WINDOW_MS 20000
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

// ---- Where the sender was ----------------------------------------------------------------
//
// A board with a GNSS fix appends its position to every packet it sends: 6 bytes, 18 in all.
// A board without one sends the plain 12, exactly as before. Receivers accept those two lengths
// and nothing else, so the exact-length filter against foreign traffic still holds.
//
// The key is the sequence number, not time. (sender, channel, seq) is the one thing the sender
// and every receiver already agree on without any arithmetic, so it is what a position is filed
// under everywhere: the packet carries it, every receiver logs it against that seq, and the
// sender's own log can reproduce it for every seq it sent - heard or not - from one record per
// fix. See rt_geo_at() for how.
//
// Only the fraction of a degree travels. The whole degrees are the same for every board in a
// range test and for the phone standing among them, so the page supplies them from whatever
// full position it has - the phone's GPS, the GNSS board's own log, or the last one it saw -
// picking the degree that lands nearest. That is safe while everything is within half a degree
// (~50km) of the reference, which a radio range test always is.
//
// Resolution is 1e-5 degree: 1.1m of latitude, less of longitude. Finer than that would be finer
// than the GNSS; coarser starts to show as steps. Marks that do land on the same spot are spread
// by the page when it draws them, which is a display problem and solved there.
//
//   48 bits, little-endian:  bits 0-16  latitude,  1e-5 degree units mod 100000 (0..99999)
//                            bits 17-33 longitude, the same
//                            bits 34-47 altitude,  metres above mean sea level, signed, clamped
//
// Every byte here is a longer target for a bit error at the edge of range, which is why it is
// packed this tight: a board with coordinates in its packets delivers very slightly worse than
// one without, and that cost was accepted when the owner asked for coordinates in the packet.
typedef struct __attribute__((packed)) {
    uint8_t b[6];
} rt_geo_wire_t;

typedef struct __attribute__((packed)) {
    rt_pkt_t      p;
    rt_geo_wire_t g;
} rt_pkt_geo_t;

#define RT_PKT_LEN     ((int)sizeof(rt_pkt_t))       // 12
#define RT_PKT_GEO_LEN ((int)sizeof(rt_pkt_geo_t))   // 18
#define RT_PKT_MAX     RT_PKT_GEO_LEN

#define RT_GEO_FRAC  100000   // 1e-5 degree units per degree

// What a receiver gets out of a packet: fractions of a degree, and altitude.
typedef struct {
    int32_t latf, lonf;   // 0..99999
    int16_t alt_m;
} rt_pos_t;

// Floor division, so negative coordinates quantise and wrap the same way as positive ones.
static inline int32_t rt_floordiv(int64_t a, int64_t b)
{
    int64_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0))) {
        q--;
    }
    return (int32_t)q;
}

// Full position in 1e-7 degrees to the wire: rounded to 1e-5, then only the fraction kept.
static inline void rt_geo_pack(uint8_t out[6], int32_t lat_e7, int32_t lon_e7, int16_t alt_m)
{
    const int32_t lat = rt_floordiv((int64_t)lat_e7 + 50, 100);
    const int32_t lon = rt_floordiv((int64_t)lon_e7 + 50, 100);
    const uint64_t la = (uint64_t)(lat - rt_floordiv(lat, RT_GEO_FRAC) * RT_GEO_FRAC);
    const uint64_t lo = (uint64_t)(lon - rt_floordiv(lon, RT_GEO_FRAC) * RT_GEO_FRAC);
    const int32_t  al = alt_m < -8192 ? -8192 : alt_m > 8191 ? 8191 : alt_m;
    const uint64_t v  = la | (lo << 17) | ((uint64_t)(al & 0x3FFF) << 34);
    for (int i = 0; i < 6; i++) {
        out[i] = (uint8_t)(v >> (8 * i));
    }
}

static inline void rt_geo_unpack(const uint8_t in[6], rt_pos_t *p)
{
    uint64_t v = 0;
    for (int i = 0; i < 6; i++) {
        v |= (uint64_t)in[i] << (8 * i);
    }
    p->latf = (int32_t)(v & 0x1FFFF);
    p->lonf = (int32_t)((v >> 17) & 0x1FFFF);
    const int32_t a = (int32_t)((v >> 34) & 0x3FFF);
    p->alt_m = (int16_t)(a >= 0x2000 ? a - 0x4000 : a);
}

// ---- Momentum ------------------------------------------------------------------------------
//
// Fixes come once a second and packets four times a second, and a fix can go missing, so what a
// packet carries is not the last fix but a smooth track through them: velocity tracked over
// time, and a position that eases from wherever the track was toward each new fix.
//
// The GNSS reader keeps velocity as an EMA of fix-to-fix motion with a 2s time constant, in
// floating point relative to the first good fix (rt_gnss.c). What it publishes with each fix is
// the fix F, the velocity V, and E - where the track was, minus F, at the moment F arrived - so
// the track never jumps. For a packet sent time t after the fix:
//
//     position = F + E x w(t) + V x g(t)      w(t) = e^(-t/1s)      g(t) = t e^(-t^2 / 2(2s)^2)
//
// w eases the track onto the fix, over a second: a 2s ease on top of a velocity that itself
// takes a couple of seconds to die away left a stopped drone drawn 5m past where it stopped for
// several seconds. g is the velocity term: close to t for the first second, so
// steady flight is followed closely; peaking at 1.2s of travel two seconds on; back to zero by
// about eight. So with fixes arriving, it runs a little behind and occasionally overshoots and
// comes back, and if fixes stop it coasts on briefly and returns to the last real fix instead
// of running off along a straight line, or stopping dead and piling everything up on one spot.
//
// t is counted in sequence numbers times each channel's nominal period, never read off a clock,
// and w and g are tables rather than exp(): so the position is a pure function of (fix, seq), in
// integers, and the sender's log, every receiver and the page all arrive at the same fake
// location for the same seq, exactly. The page carries identical tables and arithmetic.
#define RT_GEO_K_MAX 60   // table length - 1, in 250ms steps: 15s, past any use

static const uint16_t RT_GEO_W[RT_GEO_K_MAX + 1] = {   // 32768 e^(-k/4)
    32768, 25520, 19875, 15479, 12055, 9388, 7312, 5694, 4435, 3454, 2690, 2095, 1631, 1271,
    990, 771, 600, 467, 364, 283, 221, 172, 134, 104, 81, 63, 49, 38, 30, 23, 18, 14, 11, 9, 7,
    5, 4, 3, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static const uint16_t RT_GEO_G[RT_GEO_K_MAX + 1] = {   // 1000 t e^(-t^2/8), t = k/4 seconds
    0, 248, 485, 699, 882, 1028, 1132, 1193, 1213, 1195, 1145, 1069, 974, 868, 757, 647, 541,
    444, 358, 283, 220, 167, 125, 92, 67, 47, 33, 23, 15, 10, 7, 4, 3, 2, 1, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// The fix as published: F in 1e-7 degrees, E and V in 1e-7 degrees and 1e-7 degrees per second.
// utc_ds is used only on the board, to time the velocity; it goes nowhere.
typedef struct {
    int32_t  lat_e7, lon_e7;
    int16_t  alt_m;
    int16_t  elat, elon;
    int16_t  vlat, vlon;
    uint32_t utc_ds;
    uint8_t  hdop_ds;
    uint8_t  sats;
} rt_geo_t;

// Time since the fix in table steps, from how far this channel's seq has moved on since it.
static inline uint32_t rt_geo_k(uint32_t steps, int chan)
{
    const uint32_t per = chan == CH_BLE_ADV ? 2 : 1;   // 500ms or 250ms a seq, in 250ms steps
    return steps > RT_GEO_K_MAX ? RT_GEO_K_MAX + 1 : steps * per;
}

static inline void rt_geo_at(const rt_geo_t *g, uint32_t k, int32_t *lat, int32_t *lon)
{
    *lat = g->lat_e7;
    *lon = g->lon_e7;
    if (k <= RT_GEO_K_MAX) {
        *lat += (int32_t)((int64_t)g->elat * RT_GEO_W[k] / 32768)
              + (int32_t)((int64_t)g->vlat * RT_GEO_G[k] / 1000);
        *lon += (int32_t)((int64_t)g->elon * RT_GEO_W[k] / 32768)
              + (int32_t)((int64_t)g->vlon * RT_GEO_G[k] / 1000);
    }
}

// ---- GNSS (rt_gnss.c) ----------------------------------------------------------------------
//
// Optional. NMEA on UART1: the module's TX goes to GPIO20, its RX to GPIO19. Nothing is sent to
// the module yet, so the GPIO19 wire is optional. The baud rate is found by trying each common
// one until a sentence with a valid checksum arrives; with nothing fitted the pin idles on its
// pull-up and the board simply never has a fix, which is the same as every board before this.
//
// The reader publishes each new fix through rt_geo_publish() below, and withdraws it with
// rt_geo_lost() when the fix drops or goes stale. Between those two calls every packet this
// board sends carries the fix.
#define RT_GNSS_NONE 0   // no valid NMEA seen at any baud rate - no module, or not wired
#define RT_GNSS_NMEA 1   // talking, but no fix
#define RT_GNSS_FIX  2

typedef struct {
    uint8_t  state;     // RT_GNSS_*
    bool     had_fix;   // at some point since boot
    uint8_t  sats;
    uint8_t  hdop_ds;
    uint32_t baud;      // 0 while still searching
} rt_gnss_st_t;

void rt_gnss_start(void);
void rt_gnss_status(rt_gnss_st_t *out);
bool rt_gnss_had_fix(void);

// Hand a new fix to the transmit path, atomically with a snapshot of the sequence counters, and
// log it. Every sequence number handed out after this call carries this fix (moved on by
// rt_geo_at), every one before it the previous one - which is exactly what the log's fix record
// lets the page reproduce.
void rt_geo_publish(const rt_geo_t *g);
void rt_geo_lost(void);
// The fix packets are currently carrying, if any.
bool rt_geo_get(rt_geo_t *out);

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

// ---- The packet log ----------------------------------------------------------------------
//
// Eleven bytes: { 0x8C, u32 session, u32 block, u16 offset }, all little-endian. "I have
// everything of yours before this point - send me the rest." The board streams its log from
// there (see RT_LOG_* below) until the connection drops, and does nothing until asked, so a
// page that does not know about the log costs nothing.
#define RT_CMD_LOG_FROM 0x8C

// ---- User LED ----------------------------------------------------------------------------
//
// The XIAO's LED on GPIO15: off, lit, or blinking at 2Hz - for finding a board in long grass,
// confirming which of two identical boards you are holding, or marking a moment in a walk.
// The blink exists because steady and dark are both things a board can be by accident; a 2Hz
// blink is unmistakably something someone asked for, which is what you want when the question
// is "is that one mine?" across a field.
//
// Off at boot and never remembered, and off is *floating*, not driven: the pin is left as
// reset found it until the LED is first asked for. GPIO15 is a strapping pin, so not driving
// it is the quieter default, and it means an LED left on cannot survive a power cycle.
//
// Nothing else depends on this. It touches no radio, claims no airtime and is not a control
// path, which is also why the button restore leaves it alone - see restore_control() in
// main.c. Polarity lives in one define, LED_ON_LEVEL in main.c.
#define RT_LED_OFF   0
#define RT_LED_ON    1
#define RT_LED_BLINK 2
#define RT_LED_COUNT 3

// Three explicit states, not a "next state" byte, even though the UI presents them as one
// button that cycles. Cycling is a view; the wire carries the state itself, so a write that
// is retried, duplicated or lost cannot leave the board one step out of phase with the button
// that sent it - and two phones looking at the same board always agree.
#define RT_CMD_LED_OFF   0x89
#define RT_CMD_LED_ON    0x8A
#define RT_CMD_LED_BLINK 0x8B

extern volatile uint8_t g_led;
void rt_set_led(int mode);
const char *rt_led_name(int mode);

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
// out must hold RT_PKT_MAX bytes. Returns the length to send: RT_PKT_GEO_LEN while this board
// has a GNSS fix, RT_PKT_LEN otherwise.
int rt_fill(void *out, int chan, int8_t txdbm);

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
//   status block (84 bytes, only in a 0x01 chunk, immediately after the header)
//     u8  ver           RT_RPT_VER
//     u24 node          rt_node_id()
//     u8  lc
//     u8  state         bit0 lr, bit1 ant_ext, bit2 wifi_active, bit3 conn_2m requested,
//                       bits 4-5 conn PHY actually in use (0 unknown, 1 1M, 2 2M, 3 coded),
//                       bit6 tx_mute
//     u8  led           0 off, 1 on, 2 blinking at 2Hz. Its own byte rather than more bits
//                       in state: state was full at bit7, and three states do not fit in one
//                       bit. Squeezing it in beside something else would have made the
//                       next field along someone else's problem to find.
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
//   v7 appends 18 bytes to the status block, after tx[3] - nothing before it moves (v6 had
//   20, the same plus a u16 block size at the end):
//     u8  gnss          bits 0-1 RT_GNSS_* state, bit2 has had a fix since boot
//     u8  gnss_sats
//     u8  gnss_hdop     x10, 255 unknown
//     u8  gnss_baud     baud / 1200, 0 while still searching
//     u32 log_session   random per boot, never 0 - a board that rebooted has a new log
//     u32 log_oldest    oldest block still held
//     u32 log_newest    the block being written now
//     u16 log_cap       blocks the ring holds (RT_LOG_BLOCK bytes each), 0 if there was no
//                       memory for one
//
//   row record (23 bytes, packed end to end after whichever block precedes them)
//     u24 peer, u8 chan            v6: bit7 of chan = this peer's last packet on this channel
//                                  carried its position
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
// 36 bytes of status + 3 x 16 bytes of per-channel tx accounting. Checked against the
// serialiser at runtime rather than trusted - see rt_snapshot_chunk().
//
// The version byte is what lets the page decode more than one layout, and v5 is the first
// bump where that is worth doing. Every earlier one changed the packets boards send *each
// other* - a mixed pair could not measure anything, so there was nothing to be compatible
// with and the page simply refused the older board. v5 changed only what a board tells a
// phone: the 12-byte measurement packet is untouched, so a v4 board and a v5 board still
// range-test each other exactly as before, and the only difference is one byte of offset in
// the report. The page decodes both, which is what keeps "change an LED" from meaning
// "reflash every board in the drawer".
//
// v6 is not that kind of bump, and says so: the measurement packet grew a second length (see
// rt_pkt_geo_t), and an older board discards it as foreign. Mixed fleets still range-test each
// other while no board has a GNSS fix, and stop hearing a board the moment it gets one. Reflash
// the lot.
//
// v7 is v6 reworked: positions keyed by seq instead of GNSS time and carried as fractions of a
// degree, so the packet extension is 6 bytes instead of 14 (18 in all, and v6 and v7 cannot hear
// each other's), the status tail lost log_bsize (18 bytes, not 20), and the log records changed.
// The page reads a v6 report but does not ask a v6 board for its log.
#define RT_RPT_VER      7
#define RT_RPT_HDR      4
#define RT_RPT_STATUS   102
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

// ---- The packet log (rt_log.c) ---------------------------------------------------------
//
// The report above says how each link is doing *now*. It cannot say where anything happened,
// and it cannot say anything at all about the minutes a drone spent out of range of the phone.
// The log does both: one record per packet this board heard that has a position attached -
// either because the sender carried one, or because this board has a GNSS fix of its own -
// plus this board's own fixes. It lives in RAM, a ring of fixed-size blocks, oldest dropped
// first, and it is streamed to the phone over the same notify characteristic as the report.
//
// Not flash: flash writes stall the chip, and a stalled receiver is its own source of lost
// packets. So the log is a fixed RT_LOG_BYTES of RAM, taken first thing at boot, before any radio
// has allocated anything - one clean block, nothing for later mode changes to fragment around.
// The size is set by hand from the heap figures the serial report prints (free, lowest ever,
// largest free block) once a board has been through real use, phone connections included. On a
// long flight it keeps the most recent part; hence the compact records - every byte saved is
// flight time kept.
//
// Every block stands alone: it starts with its own clock and names every link it refers to, so
// a block can be decoded without the one before it. That is what lets the board send the newest
// block first and backfill the older ones behind it, and lets a block dropped off the end of the
// ring cost only its own records.
//
// Block (RT_LOG_BLOCK bytes, used from the front):
//   u32 t0             board ms when the block was opened; "clock" starts here
//   records, packed, each starting with a type byte. Event records carry u8 dt: clock advances
//   by dt x 4ms first. The board rounds against the clock it has already emitted rather than the
//   raw time, so the error never accumulates past 2ms.
//
//   0x01 TIME   (3)  u16 ms             clock += ms. For gaps too long for a u8 dt.
//   0x02 FIX    (34) dt, i32 lat_e7, i32 lon_e7, i16 alt_m, i16 elat, i16 elon, i16 vlat,
//                    i16 vlon, u8 hdop_ds, u8 sats, u32 seq_next[3]
//                    This board's own new fix, as published (see Momentum). seq_next is each
//                    channel's next sequence number at that instant: from there until the next
//                    FIX or NOFIX, packet seq on channel c carried
//                    rt_geo_pack(rt_geo_at(fix, rt_geo_k(seq - seq_next[c], c))).
//   0x03 NOFIX  (14) dt, u32 seq_next[3]
//                    Fix lost; packets from seq_next onward carry no position.
//   0x04 RX     (15) dt, u24 node, u8 chan (bit7 = carried a position: the node's current POS),
//                    i8 ptx, u32 seq, u16 gap, i8 rssi, u8 q
//                    A packet heard, and a new link reference: the Nth RX record in a block
//                    is ref N, for the short records that follow. gap is seq minus the previous
//                    sequence number heard on this link, so gap-1 were missed - the same
//                    arithmetic as the results table, done where the table is. 0 = the same seq
//                    heard again (a BLE advert goes out 2-3 times per seq, and each one heard
//                    counts, as in the table), or unknown.
//   0x05 POS    (10) u24 node, 6 bytes exactly as on the wire (rt_geo_wire_t)
//                    Where that node's next packet said it was. Sets the node's current POS
//                    for the records after it in this block.
//   0x40+ RXD   (8)  type = 0x40 | ref, dt, u8 gap, i8 rssi, u8 q, i8 dlat, i8 dlon, i8 dalt
//                    A packet heard on an existing ref that carried a position this far from
//                    the node's current POS - which it then becomes. Deltas in wire units
//                    (1e-5 degree, metres), latitude and longitude wrapping at 100000. With
//                    momentum this is the usual record for a moving sender; a jump too big for
//                    the deltas is a POS and then an RXS instead.
//   0x80+ RXS   (5)  type = 0x80 | geo << 6 | ref, dt, u8 gap, i8 rssi, u8 q
//                    A packet heard on an existing ref: seq = that ref's last seq + gap. geo: it
//                    carried exactly the node's current POS.
//
//   q is the channel's quality figure: SNR in dB (as i8) on espnow, LQI on 154, 0 on ble_adv.
//
// Streamed as notifications of type 0x03, alongside the report's 0x01/0x02 chunks:
//   u8 0x03, u8 flags (bit0 = this reaches the end of a closed block), u32 session, u32 block,
//   u16 offset, then block bytes [offset, offset + n). n may be 0 when all that is left to say
//   is that the block closed.
#define RT_LOG_BLOCK       1024
#define RT_LOG_HDR         12
#define RT_LOG_TYPE        0x03

// The ring's size. A placeholder until real heap figures say otherwise: a moving drone heard by
// a board costs that board ~8 bytes a packet, 4 packets a second - about 2KB a minute - and the
// drone logs ~5 bytes per packet it hears plus 34 a second for its fixes.
#define RT_LOG_BYTES       (32 * 1024)

typedef struct {
    uint32_t block;
    uint16_t off;
} rt_log_cur_t;

// Allocate the ring. Called first thing in app_main, before any radio. Before this - or if the
// allocation failed - every appender below is a no-op.
void rt_log_init(void);

// Appenders. Safe from any context including ISRs: 802.15.4 delivers packets from one.
void rt_log_fix(const rt_geo_t *g, const uint32_t seq_next[CH_COUNT]);
void rt_log_nofix(const uint32_t seq_next[CH_COUNT]);
void rt_log_rx(uint32_t node, int chan, int8_t ptx, uint32_t seq, uint32_t gap, int8_t rssi,
               uint8_t q, const rt_geo_wire_t *sender);

typedef struct {
    uint32_t session, oldest, newest;
    uint16_t cap, bsize;
} rt_log_st_t;

void rt_log_status(rt_log_st_t *out);

// Build one notification's worth of log starting at *c, at most cap bytes. Returns the length,
// or 0 if there is nothing to send from there yet. Does not move *c: on a successful send the
// caller copies *adv into it, so a notification that fails to queue is simply built again.
int rt_log_chunk(const rt_log_cur_t *c, uint8_t *out, int cap, rt_log_cur_t *adv);

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

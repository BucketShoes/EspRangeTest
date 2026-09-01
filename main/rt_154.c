// Raw 802.15.4 frames.
//
// No Thread, no Zigbee. Both of those sit on this exact PHY (2.4GHz O-QPSK, 250kbps), so
// the range result is identical - the stacks only add addressing and routing, which this
// project does not care about. Raw frames also mean no joining, no coordinator, and no
// association to lose at the far end.

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "esp_ieee802154.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// Must be included explicitly: an undefined macro evaluates to 0 in #if, so without this
// the whole radio would silently compile itself out on a chip that has one.
#include "soc/soc_caps.h"

#include "rt.h"

#if SOC_IEEE802154_SUPPORTED

static const char *TAG = "154";

#define TX_PERIOD_MS 250
#define CHANNEL      26     // top of the band, furthest from most Wi-Fi traffic
#define PANID        0x5254

// Retry a refused transmit instead of giving up on it.
//
// This is the one way to get a bigger share of the antenna that takes nothing from anybody.
// A coexistence refusal is not a collision and not a timeout - the arbiter says no immediately
// and the frame never goes out, so asking again a few milliseconds later costs no airtime and
// steals no slot. We are not outranking Wi-Fi or BLE, we are just declining to give up after
// the first no, and the frame lands the moment a gap appears.
//
// Retries re-send the *same* frame. rt_fill() runs once per scheduled packet, so the sequence
// number does not advance on a retry and the receiver's loss arithmetic is untouched: one
// scheduled packet is still one sequence number, however many attempts it took.
//
// Backoff is randomised because two boards running this firmware would otherwise retry in
// lockstep and keep colliding with each other's retries instead of with the gaps.
#define TX_TRIES      6
#define TX_BACKOFF_MS 20    // actual delay is 10..30ms; FreeRTOS tick here is 10ms
#define TX_WAIT_MS    50    // how long to wait for the radio's verdict on one attempt

// Header: FCF(2) seq(1) dstpan(2) dstaddr(2) srcaddr(2) = 9 bytes, then payload, then a
// 2-byte FCS the radio appends itself.
#define HDR_LEN 9
#define FCS_LEN 2

static uint8_t s_seq;

// What the radio says its transmit power actually is, read back rather than assumed, and put
// into the packet so the far end records the real number. The driver defaults this to its
// maximum, so it was never the reason 802.15.4 was quiet - but "the payload says 20 dBm
// because someone typed 20" is exactly the kind of unchecked claim this project keeps getting
// caught by.
static int8_t s_txpower;

// Frames the radio handed us, and frames that survived validation. Both are reported, because
// they answer different questions and the difference between them is the whole diagnosis:
// "heard nothing at all" is an antenna or a distance problem, "heard plenty, kept none" is a
// parsing or addressing problem, and they look identical if you only count what you kept.
static volatile uint32_t s_rx_frames;
static volatile uint32_t s_rx_ours;

// Coexistence priority is deliberately left at the ESP-IDF default.
//
// The temptation is obvious: esp_ieee802154_util.c pins ordinary 802.15.4 tx/rx to
// IEEE802154_LOW, the bottom of four tiers, and esp_ieee802154_set_coex_config() would move it
// up. Don't. Wi-Fi and BLE are not "greedy" here, they are *scheduled* - a BLE connection event
// or a beacon has to happen at its moment or the link degrades, and the phone control channel
// degrading is the one failure this rig cannot tolerate. 802.15.4's normal duty cycle, on the
// other hand, is 100% receive with occasional transmits, so promoting it above them does not
// share the antenna more fairly, it hands 802.15.4 the whole thing.
//
// The isolation a low-contention mode promises is supposed to come from other radios actually
// being off or backed off - see main.c's wifi_apply() and rt_ble.c's scanner gate - not from
// re-ranking who wins a fight that should not be happening. Changing priorities is a different
// way to get the same result, and a worse-behaved one: it is invisible, it only shows up as
// somebody else's link getting flaky, and there is no button that restores it.
static const char *tx_err_name(esp_ieee802154_tx_error_t e)
{
    switch (e) {
    case ESP_IEEE802154_TX_ERR_NONE:        return "none";
    case ESP_IEEE802154_TX_ERR_CCA_BUSY:    return "cca_busy";   // channel found busy
    case ESP_IEEE802154_TX_ERR_ABORT:       return "abort";
    case ESP_IEEE802154_TX_ERR_NO_ACK:      return "no_ack";     // frames here never request one
    case ESP_IEEE802154_TX_ERR_INVALID_ACK: return "invalid_ack";
    case ESP_IEEE802154_TX_ERR_COEXIST:     return "coexist";    // Wi-Fi/BLE arbiter refused the slot
    case ESP_IEEE802154_TX_ERR_SECURITY:    return "security";
    default:                                return "?";
    }
}

// Every rejection, by reason, for as long as it keeps happening.
//
// esp_ieee802154_transmit_failed() runs in ISR context (straight out of ieee802154_isr), where
// ESP_LOGW's underlying vfprintf takes a newlib lock and aborts - so the ISR cannot print. The
// previous version worked around that by latching only a *change* of error type, which meant a
// radio being refused four times a second logged one line at boot and then went quiet. That is
// indistinguishable from a one-off hiccup, and it is exactly how 100%-rejected 802.15.4 stayed
// hidden. A refusal is the single most useful debug signal this channel produces, so:
//
//   ISR   - increments a counter, nothing else.
//   task  - prints totals and the per-window delta, every ERR_REPORT_MS, while anything is
//           still being rejected. Silent only when the radio is genuinely not failing.
#define TX_ERR_N      8      // >= the esp_ieee802154_tx_error_t range; bounds-checked anyway
#define ERR_REPORT_MS 2000   // matches the results report, so the two read as one timeline

static volatile uint32_t s_tx_err_n[TX_ERR_N];    // ISR writes, tx_task reads
static uint32_t          s_tx_err_shown[TX_ERR_N];
static uint32_t          s_err_next_ms;

void rt_154_apply_power(void)
{
    RT_TRY(TAG, esp_ieee802154_set_txpower(rt_power_dbm(CH_154)));
    // Read back: the driver quantises to 3dB steps, and this value goes into every packet, so
    // the far end records what was actually transmitted rather than what was requested.
    s_txpower = esp_ieee802154_get_txpower();
    rt_power_set_actual(CH_154, s_txpower);
    ESP_LOGI(TAG, "tx power %d dBm (asked %d)", s_txpower, rt_power_dbm(CH_154));
}

static void report_tx_errors(void)
{
    // Signed difference so this survives the rt_ms() wrap at ~49 days.
    if ((int32_t)(rt_ms() - s_err_next_ms) < 0) {
        return;
    }
    s_err_next_ms = rt_ms() + ERR_REPORT_MS;

    char   buf[160];
    size_t n     = 0;
    bool   fresh = false;

    for (int e = 0; e < TX_ERR_N; e++) {
        const uint32_t tot = s_tx_err_n[e];
        if (tot == 0) {
            continue;
        }
        const uint32_t delta = tot - s_tx_err_shown[e];
        s_tx_err_shown[e] = tot;
        if (delta) {
            fresh = true;
        }
        const int w = snprintf(buf + n, sizeof(buf) - n, "%s%s %lu(+%lu)", n ? ", " : "",
                               tx_err_name((esp_ieee802154_tx_error_t)e),
                               (unsigned long)tot, (unsigned long)delta);
        if (w < 0 || (size_t)w >= sizeof(buf) - n) {
            break;
        }
        n += (size_t)w;
    }

    // Only when something was rejected in this window. A channel that recovers stops nagging,
    // but a channel that is still failing says so every two seconds, forever.
    if (fresh) {
        ESP_LOGW(TAG, "tx rejected: %s", buf);
    }

    // The receiver's side of the same question. "Nothing heard" and "plenty heard, none of it
    // ours" are completely different faults and the results table cannot tell them apart, so
    // say both. Silent while the radio is genuinely hearing nothing at all - which is itself
    // the answer when the other board is transmitting.
    static uint32_t shown_frames, shown_ours;
    if (s_rx_frames != shown_frames || s_rx_ours != shown_ours) {
        ESP_LOGI(TAG, "rx %lu frames, %lu ours (+%lu, +%lu)",
                 (unsigned long)s_rx_frames, (unsigned long)s_rx_ours,
                 (unsigned long)(s_rx_frames - shown_frames),
                 (unsigned long)(s_rx_ours - shown_ours));
        shown_frames = s_rx_frames;
        shown_ours   = s_rx_ours;
    }
}

// Set by the ISR callbacks, read by tx_task between attempts. s_tx_task is what lets the ISR
// wake the task the instant the verdict is in, rather than the task polling for it.
static TaskHandle_t  s_tx_task;
static volatile bool s_tx_landed;

// One scheduled packet, up to TX_TRIES attempts at getting it on the air. Returns whether the
// radio ever confirmed it went out.
static bool transmit_with_retries(uint8_t *frame)
{
    for (int attempt = 0; attempt < TX_TRIES; attempt++) {
        if (attempt > 0) {
            vTaskDelay(pdMS_TO_TICKS(TX_BACKOFF_MS / 2 + esp_random() % TX_BACKOFF_MS));
        }

        s_tx_landed = false;
        ulTaskNotifyTake(pdTRUE, 0);  // drop any verdict left over from a previous packet

        if (esp_ieee802154_transmit(frame, false) != ESP_OK) {
            continue;  // driver would not even take it; no callback is coming
        }
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TX_WAIT_MS)) == 0) {
            break;     // no verdict at all - something is wrong that retrying will not fix
        }
        if (s_tx_landed) {
            return true;
        }
    }
    return false;
}

static void tx_task(void *pv)
{
    (void)pv;
    s_tx_task = xTaskGetCurrentTaskHandle();

    for (;;) {
        report_tx_errors();
        if (rt_tx_enabled(CH_154)) {
            uint8_t frame[1 + HDR_LEN + sizeof(rt_pkt_t) + FCS_LEN];
            uint8_t *f = &frame[1];

            // Data frame, short dest + short src, PAN ID compressed, no ack requested
            // (broadcast, and an ack would change what we are measuring).
            f[0] = 0x41;
            f[1] = 0x88;
            f[2] = s_seq++;
            f[3] = (uint8_t)(PANID & 0xFF);
            f[4] = (uint8_t)(PANID >> 8);
            f[5] = 0xFF;  // broadcast short address
            f[6] = 0xFF;
            f[7] = rt_node_id();
            f[8] = 0x00;

            rt_pkt_t p;
            rt_fill(&p, CH_154, s_txpower);
            memcpy(&f[HDR_LEN], &p, sizeof(p));

            frame[0] = HDR_LEN + sizeof(rt_pkt_t) + FCS_LEN;  // PSDU length incl. FCS

            // ok/failed is the verdict on the packet, not on an attempt - so queued still
            // equals ok + failed however many refusals happened along the way. The refusals
            // themselves are not lost: report_tx_errors() counts every one of them, which is
            // where the cost of getting a frame out shows up.
            if (transmit_with_retries(frame)) {
                rt_tx_ok(CH_154);
            } else {
                rt_tx_failed(CH_154);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(rt_jitter_ms(TX_PERIOD_MS)));
    }
}

// Is this one of ours, laid out the way rt_154.c lays them out?
//
// The radio is in promiscuous mode, so it hands up *every* 802.15.4 frame on channel 26 - any
// PAN, any addressing mode, anything a Zigbee or Thread device nearby happens to emit. HDR_LEN
// is only 9 bytes for the specific addressing this file transmits; a frame using long
// addresses or an uncompressed PAN has a different header length, so reading its payload at a
// fixed offset finds whatever happened to be at byte 10. Previously the only thing standing
// between that and the results table was rt_rx()'s two-byte magic - one chance in 65536 that
// arbitrary bytes look like a measurement.
//
// So check the header actually is the shape the offset assumes, before trusting the offset:
// data frame, PAN ID compressed, short destination, short source - byte for byte what
// tx_task builds - addressed to our PAN, broadcast, and with the header's source address
// agreeing with the sender id inside the payload. A foreign frame now has to match all of
// that as well as the magic.
static bool frame_is_ours(const uint8_t *f, int psdu)
{
    if (psdu < HDR_LEN + (int)sizeof(rt_pkt_t) + FCS_LEN) {
        return false;
    }
    // FCF: type = data, PAN ID compression set.
    if ((f[0] & 0x07) != 0x01 || (f[0] & 0x40) == 0) {
        return false;
    }
    // FCF: destination and source address modes both "short" - this is what fixes HDR_LEN.
    if (((f[1] >> 2) & 0x03) != 0x02 || ((f[1] >> 6) & 0x03) != 0x02) {
        return false;
    }
    if ((uint16_t)(f[3] | (f[4] << 8)) != PANID) {
        return false;
    }
    if (f[5] != 0xFF || f[6] != 0xFF) {  // we only ever broadcast
        return false;
    }
    // The sender id appears twice - in the header's source address and in the payload. A frame
    // that is genuinely ours agrees with itself.
    return f[7] == f[HDR_LEN + offsetof(rt_pkt_t, node)];
}

void esp_ieee802154_receive_done(uint8_t *frame, esp_ieee802154_frame_info_t *info)
{
    // frame[0] is the PSDU length, including the trailing FCS.
    const int      psdu = frame[0];
    const uint8_t *f    = &frame[1];

    s_rx_frames++;
    if (frame_is_ours(f, psdu)) {
        s_rx_ours++;
        rt_rx(&f[HDR_LEN], psdu - HDR_LEN - FCS_LEN, CH_154, info->rssi, info->lqi);
    }
    esp_ieee802154_receive_handle_done(frame);
}

// The radio confirming a frame actually went out. ack is NULL here - these frames never request
// one - so this fires on transmission, not on delivery, which is exactly the question being
// asked: did the antenna ever get used at all?
void esp_ieee802154_transmit_done(const uint8_t *frame, const uint8_t *ack,
                                  esp_ieee802154_frame_info_t *ack_info)
{
    (void)frame; (void)ack; (void)ack_info;
    s_tx_landed = true;
    BaseType_t woken = pdFALSE;
    if (s_tx_task) {
        vTaskNotifyGiveFromISR(s_tx_task, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

// ISR context - counter increments and a wake-up, no logging. See report_tx_errors() above.
// Note this no longer calls rt_tx_failed(): a refused attempt is not a lost packet until the
// retries are exhausted, and that verdict belongs to tx_task.
void esp_ieee802154_transmit_failed(const uint8_t *frame, esp_ieee802154_tx_error_t error)
{
    (void)frame;
    if ((unsigned)error < TX_ERR_N) {
        s_tx_err_n[error]++;
    }
    s_tx_landed = false;
    BaseType_t woken = pdFALSE;
    if (s_tx_task) {
        vTaskNotifyGiveFromISR(s_tx_task, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

void rt_154_start(void)
{
    RT_TRY(TAG, esp_ieee802154_enable());
    RT_TRY(TAG, esp_ieee802154_set_channel(CHANNEL));
    rt_154_apply_power();
    RT_TRY(TAG, esp_ieee802154_set_panid(PANID));
    RT_TRY(TAG, esp_ieee802154_set_short_address(rt_node_id()));
    RT_TRY(TAG, esp_ieee802154_set_promiscuous(true));  // hear everything, filter in rt_rx
    RT_TRY(TAG, esp_ieee802154_set_rx_when_idle(true));
    RT_TRY(TAG, esp_ieee802154_receive());

    xTaskCreate(tx_task, "154_tx", 3072, NULL, 4, NULL);
    ESP_LOGI(TAG, "channel %d, tx every %dms", CHANNEL, TX_PERIOD_MS);
}

#else  // no 802.15.4 radio on this target (C3, S3)

void rt_154_start(void) {}
void rt_154_apply_power(void) {}

#endif

// Raw 802.15.4 frames.
//
// No Thread, no Zigbee. Both of those sit on this exact PHY (2.4GHz O-QPSK, 250kbps), so
// the range result is identical - the stacks only add addressing and routing, which this
// project does not care about. Raw frames also mean no joining, no coordinator, and no
// association to lose at the far end.

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
            rt_fill(&p, CH_154, 20);
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

void esp_ieee802154_receive_done(uint8_t *frame, esp_ieee802154_frame_info_t *info)
{
    // frame[0] is the PSDU length, including the trailing FCS.
    const int psdu = frame[0];
    const int payload = psdu - HDR_LEN - FCS_LEN;
    if (payload >= (int)sizeof(rt_pkt_t)) {
        rt_rx(&frame[1 + HDR_LEN], payload, CH_154, info->rssi, info->lqi);
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

#endif

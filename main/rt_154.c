// Raw 802.15.4 frames.
//
// No Thread, no Zigbee. Both of those sit on this exact PHY (2.4GHz O-QPSK, 250kbps), so
// the range result is identical - the stacks only add addressing and routing, which this
// project does not care about. Raw frames also mean no joining, no coordinator, and no
// association to lose at the far end.

#include <string.h>

#include "esp_ieee802154.h"
#include "esp_log.h"
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

// Header: FCF(2) seq(1) dstpan(2) dstaddr(2) srcaddr(2) = 9 bytes, then payload, then a
// 2-byte FCS the radio appends itself.
#define HDR_LEN 9
#define FCS_LEN 2

static uint8_t s_seq;

// Coexistence priority.
//
// ESP-IDF ships 802.15.4 pinned to the bottom of the arbiter: esp_ieee802154_util.c defaults
// s_coex_config.txrx to IEEE802154_LOW, the lowest of four tiers, so any Wi-Fi or BLE request
// wins the antenna over it deterministically rather than probabilistically. That is why the
// symptom was ESP_IEEE802154_TX_ERR_COEXIST on *every* frame from the very first one, not a
// degraded packet delivery ratio - the radio is not disadvantaged, it is shut out.
//
// Stopping the other radios (main.c's wifi_apply(), rt_ble.c's scanner gate) is the real fix
// and this is the belt to that braces: whatever is still up - the phone-UI link, which has to
// stay reachable - should not be able to shut 802.15.4 out again. HIGH while 802.15.4 is the
// channel under test, where by definition nothing else on this board has a claim worth
// honouring; MIDDLE otherwise, which puts it level with the ack tier the driver already uses
// for itself instead of below everything.
//
// Note what this does *not* mean: the "all radios" numbers are still contended numbers, just
// contended rather than pre-decided. A run where every 802.15.4 frame is rejected measures
// the arbiter, not the range.
static void apply_coex_pti(int lc)
{
#if !CONFIG_IEEE802154_TEST && (CONFIG_ESP_COEX_SW_COEXIST_ENABLE || CONFIG_EXTERNAL_COEX_ENABLE)
    const bool solo = (lc == CH_154 + 1);
    const esp_ieee802154_coex_config_t cfg = {
        .idle    = IEEE802154_IDLE,
        .txrx    = solo ? IEEE802154_HIGH : IEEE802154_MIDDLE,
        .txrx_at = solo ? IEEE802154_HIGH : IEEE802154_MIDDLE,
    };
    esp_ieee802154_set_coex_config(cfg);

    // Only on an actual change: every mode switch lands here, and four of the five modes want
    // the same setting, so logging unconditionally would put a "154:" line under button taps
    // that changed nothing about this radio - including at stages below 3, where it does not
    // even exist yet.
    static int s_logged = -1;
    if (s_logged != solo) {
        s_logged = solo;
        ESP_LOGI(TAG, "coex priority %s",
                 solo ? "high (154 is the channel under test)" : "middle");
    }
#else
    (void)lc;
#endif
}

void rt_154_set_lc(int lc)
{
    apply_coex_pti(lc);
}

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

// esp_ieee802154_transmit_failed() runs in ISR context (called straight out of
// ieee802154_isr), where ESP_LOGW's underlying vfprintf takes a newlib lock and aborts. So
// the ISR only records the reason; tx_task, an ordinary task, notices the change and logs it.
static volatile esp_ieee802154_tx_error_t s_last_tx_err = ESP_IEEE802154_TX_ERR_NONE;
static volatile bool                      s_tx_err_pending;

static void tx_task(void *pv)
{
    (void)pv;
    for (;;) {
        if (s_tx_err_pending) {
            s_tx_err_pending = false;
            ESP_LOGW(TAG, "transmit failed: %s", tx_err_name(s_last_tx_err));
        }
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
            esp_ieee802154_transmit(frame, false);
        }
        vTaskDelay(pdMS_TO_TICKS(TX_PERIOD_MS));
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

void esp_ieee802154_transmit_done(const uint8_t *frame, const uint8_t *ack,
                                  esp_ieee802154_frame_info_t *ack_info)
{
    (void)frame; (void)ack; (void)ack_info;
}

void esp_ieee802154_transmit_failed(const uint8_t *frame, esp_ieee802154_tx_error_t error)
{
    (void)frame;
    if (error != s_last_tx_err) {
        s_last_tx_err    = error;
        s_tx_err_pending = true;
    }
    rt_tx_failed(CH_154);
}

void rt_154_start(void)
{
    RT_TRY(TAG, esp_ieee802154_enable());
    apply_coex_pti(g_lc);  // before the first transmit, not after the first rejection
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
void rt_154_set_lc(int lc) { (void)lc; }

#endif

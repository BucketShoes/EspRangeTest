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

static void tx_task(void *pv)
{
    (void)pv;
    for (;;) {
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
    (void)frame; (void)error;
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

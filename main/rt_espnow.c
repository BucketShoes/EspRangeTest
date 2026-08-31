// ESP-NOW broadcast. Connectionless, no pairing, no ack - exactly what a range test wants.

#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rt.h"

static const char *TAG = "espnow";

#define TX_PERIOD_MS 250

static const uint8_t BCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// Set once esp_now_init() has run. Low-contention modes stop and restart the Wi-Fi driver,
// and main.c's wifi_apply() calls rt_espnow_resume() on every start - including the very
// first one, which happens before this file has been initialised at all.
static bool s_inited;

static void on_rx(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    rt_rx(data, len, CH_ESPNOW, info->rx_ctrl->rssi, 0);
}

// esp_now_send() returning ESP_OK only means the driver queued the frame, not that it went
// out - the actual air result lands here, asynchronously.
static void on_send(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    (void)mac_addr;
    if (status == ESP_NOW_SEND_SUCCESS) {
        rt_tx_ok(CH_ESPNOW);
    } else {
        rt_tx_failed(CH_ESPNOW);
    }
}

static void tx_task(void *pv)
{
    (void)pv;
    for (;;) {
        if (rt_tx_enabled(CH_ESPNOW)) {
            rt_pkt_t p;
            rt_fill(&p, CH_ESPNOW, 20);
            // A non-OK return here means the driver would not even queue it - the common
            // case is ESP_ERR_ESPNOW_NO_MEM under heavy contention. A queued send that fails
            // in the air is reported later, in on_send().
            if (esp_now_send(BCAST, (const uint8_t *)&p, sizeof(p)) != ESP_OK) {
                rt_tx_failed(CH_ESPNOW);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(rt_jitter_ms(TX_PERIOD_MS)));
    }
}

// Called after every esp_wifi_start(), so the peer table is known-good rather than assumed to
// have survived the stop. Confirmed by the logs that it does survive - the peer list belongs to
// the ESP-NOW module, which esp_wifi_stop() does not touch - so this is now a check rather than
// a blind re-add. Asking esp_now_add_peer() and swallowing ESP_ERR_ESPNOW_EXIST worked, but the
// component logs its own "Peer exists. Please call API esp_now_mod_peer()!" warning before
// returning, which reads like a fault every time a mode changes.
// Pin the transmit rate instead of inheriting whatever rate control decides.
//
// ESP-NOW rides the Wi-Fi PHY configuration - which is exactly why turning LR on for an
// ESP-NOW test drags Wi-Fi into LR too and takes the phone's view of the SoftAP with it. That
// coupling is real and is not going away. What this adds is an explicit statement of the rate
// for our own frames, so a range result is attributable to a known modulation rather than to
// whatever the driver felt like at the time.
//
// 1 Mbps long-preamble DSSS is the longest-range non-LR rate; LORA_250K is the LR one. The
// result is logged rather than swallowed: it is worth knowing whether the LR rate is accepted
// while the protocol list does *not* contain WIFI_PROTOCOL_LR, because if it were, ESP-NOW
// could run at LR rate with the AP still visible to a phone - which would be a genuinely new
// option rather than the all-or-nothing choice LR is today. Do not assume it works; read the
// line.
static void pin_rate(void)
{
    esp_now_rate_config_t cfg = { 0 };
    cfg.phymode = g_lr ? WIFI_PHY_MODE_LR : WIFI_PHY_MODE_11B;
    cfg.rate    = g_lr ? WIFI_PHY_RATE_LORA_250K : WIFI_PHY_RATE_1M_L;
    cfg.ersu    = false;
    cfg.dcm     = false;

    const esp_err_t err = esp_now_set_peer_rate_config((uint8_t *)BCAST, &cfg);
    ESP_LOGI(TAG, "tx rate %s -> %s", g_lr ? "LR 250k" : "11b 1M", esp_err_to_name(err));
}

void rt_espnow_resume(void)
{
    if (!s_inited) {
        return;
    }
    if (!esp_now_is_peer_exist(BCAST)) {
        esp_now_peer_info_t peer = { 0 };
        memcpy(peer.peer_addr, BCAST, 6);
        peer.channel = 0;  // whatever channel the interface is already on
        peer.ifidx   = WIFI_IF_STA;
        peer.encrypt = false;
        RT_TRY(TAG, esp_now_add_peer(&peer));
        ESP_LOGI(TAG, "broadcast peer re-added after a wifi restart");
    }
    // Re-pinned on every Wi-Fi start: the rate lives in the driver, which has just restarted,
    // and g_lr may have changed while it was down.
    pin_rate();
}

void rt_espnow_start(void)
{
    RT_TRY(TAG, esp_now_init());
    RT_TRY(TAG, esp_now_register_recv_cb(on_rx));
    RT_TRY(TAG, esp_now_register_send_cb(on_send));
    s_inited = true;
    rt_espnow_resume();

    xTaskCreate(tx_task, "espnow_tx", 3072, NULL, 4, NULL);
    ESP_LOGI(TAG, "broadcasting every %dms", TX_PERIOD_MS);
}

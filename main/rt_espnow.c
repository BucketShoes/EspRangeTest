// ESP-NOW broadcast. Connectionless, no pairing, no ack - exactly what a range test wants.

#include <string.h>

#include "esp_log.h"
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
    if (status != ESP_NOW_SEND_SUCCESS) {
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
// have survived the stop. It does survive - the peer list belongs to the ESP-NOW module, which
// esp_wifi_stop() does not touch - so the usual outcome here is ESP_ERR_ESPNOW_EXIST, which is
// success by another name. Re-adding costs two lines; finding out the hard way, mid-walk, that
// an assumption about somebody else's driver was wrong costs a test run.
void rt_espnow_resume(void)
{
    if (!s_inited) {
        return;
    }
    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, BCAST, 6);
    peer.channel = 0;  // whatever channel the interface is already on
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;

    const esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "esp_now_add_peer -> %s", esp_err_to_name(err));
    }
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

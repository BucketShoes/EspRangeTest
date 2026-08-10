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

static void on_rx(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    rt_rx(data, len, CH_ESPNOW, info->rx_ctrl->rssi, 0);
}

static void tx_task(void *pv)
{
    (void)pv;
    for (;;) {
        if (rt_tx_enabled(CH_ESPNOW)) {
            rt_pkt_t p;
            rt_fill(&p, CH_ESPNOW, 20);
            esp_now_send(BCAST, (const uint8_t *)&p, sizeof(p));
        }
        vTaskDelay(pdMS_TO_TICKS(TX_PERIOD_MS));
    }
}

void rt_espnow_start(void)
{
    RT_TRY(TAG, esp_now_init());
    RT_TRY(TAG, esp_now_register_recv_cb(on_rx));

    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, BCAST, 6);
    peer.channel = 0;  // whatever channel the interface is already on
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    RT_TRY(TAG, esp_now_add_peer(&peer));

    xTaskCreate(tx_task, "espnow_tx", 3072, NULL, 4, NULL);
    ESP_LOGI(TAG, "broadcasting every %dms", TX_PERIOD_MS);
}

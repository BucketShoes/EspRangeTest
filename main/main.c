// ESP32-C6 range tester.
//
// All three radios transmit a small numbered packet on a timer and listen the rest of the
// time. Every couple of seconds the results table goes out over serial: RSSI and packet
// loss per peer, per radio. Flash two boards, walk away with one, watch the numbers.
//
// GPIO9 (the BOOT button) cycles low-contention mode, so a single radio can be tested with
// the others silent - the three share one antenna and arbitrate for it, so running them
// together costs something. How much is one of the things worth measuring.

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"

#include "rt.h"

static const char *TAG = "rt";

#define REPORT_MS   2000
#define BUTTON_GPIO 9
#define LONG_MS     800
#define WIFI_CHAN   1

#if RT_STAGE >= 1
static void wifi_start(void)
{
    RT_TRY(TAG, esp_netif_init());
    RT_TRY(TAG, esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    RT_TRY(TAG, esp_wifi_init(&cfg));
    RT_TRY(TAG, esp_wifi_set_storage(WIFI_STORAGE_RAM));
    RT_TRY(TAG, esp_wifi_set_mode(WIFI_MODE_STA));
    RT_TRY(TAG, esp_wifi_start());

    // Both boards must sit on the same channel for ESP-NOW; nothing here associates to an
    // AP, so pinning it is enough.
    RT_TRY(TAG, esp_wifi_set_channel(WIFI_CHAN, WIFI_SECOND_CHAN_NONE));
    RT_TRY(TAG, esp_wifi_set_max_tx_power(78));  // 0.25dBm units, ~19.5dBm
}
#endif

// Tap cycles which radio is in low contention, long press returns to all-on. Polled,
// because a button needs debouncing anyway and this keeps it off the interrupt path.
static void button_task(void *pv)
{
    (void)pv;
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    bool     down = false;
    uint32_t t_down = 0;
    bool     fired = false;

    for (;;) {
        const bool now_down = gpio_get_level(BUTTON_GPIO) == 0;

        if (now_down && !down) {
            down   = true;
            fired  = false;
            t_down = rt_ms();
        } else if (now_down && down && !fired && (rt_ms() - t_down) > LONG_MS) {
            fired = true;
            rt_set_lc(0);
        } else if (!now_down && down) {
            down = false;
            if (!fired) {
                rt_set_lc((g_lc + 1) % (CH_COUNT + 1));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        // Nothing here stores anything - Wi-Fi config is kept in RAM - so a bad NVS
        // partition is worth a complaint, not a boot loop.
        ESP_LOGW(TAG, "nvs_flash_init -> %s (continuing)", esp_err_to_name(err));
    }

    esp_chip_info_t info;
    esp_chip_info(&info);
    ESP_LOGI(TAG, "%s rev v%d.%d, node %02X", CONFIG_IDF_TARGET,
             info.revision / 100, info.revision % 100, rt_node_id());

    ESP_LOGI(TAG, "stage %d (raise RT_STAGE in platformio.ini to add radios)", RT_STAGE);

    // Brought up one at a time with a line before each, so if anything does take the board
    // down the last line printed names the culprit.
#if RT_STAGE >= 1
    ESP_LOGI(TAG, "init: wifi");
    wifi_start();
    ESP_LOGI(TAG, "init: espnow");
    rt_espnow_start();
#endif
#if RT_STAGE >= 2
    ESP_LOGI(TAG, "init: ble");
    rt_ble_start();
#endif
#if RT_STAGE >= 3
#if SOC_IEEE802154_SUPPORTED
    ESP_LOGI(TAG, "init: 802.15.4");
    rt_154_start();
#else
    ESP_LOGW(TAG, "no 802.15.4 radio on this target - that channel is disabled");
#endif
#endif
    ESP_LOGI(TAG, "init: done");

    xTaskCreate(button_task, "button", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "running. GPIO9: tap = next radio low-contention, hold = all radios");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(REPORT_MS));
#if RT_STAGE == 0
        // No radios to report on yet - just prove the board is alive and stays alive.
        ESP_LOGI(TAG, "alive %lus (stage 0, no radios)",
                 (unsigned long)(rt_ms() / 1000));
#else
        rt_report();
#endif
#if RT_STAGE >= 4
        rt_ui_notify();
#endif
    }
}

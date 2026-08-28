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
#include <string.h>

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

#define REPORT_MS    2000
#define BUTTON_GPIO  9
#define LONG_MS      800     // release inside this window and LR_HOLD_MS: reset to all-on
#define LR_HOLD_MS   3000    // held past this (no release needed): toggle LR
#define WIFI_CHAN    1

// g_lr / rt_set_lr / rt_apply_lc_radios / rt_wifi_active live outside the RT_STAGE>=1 guard
// below because button_task (and rt_set_lc, which it drives) run at every stage - a board on
// RT_STAGE=0 still has a button task that can call rt_set_lc, which always calls
// rt_apply_lc_radios. Below stage 1 there is no Wi-Fi driver to start or stop, so rt_set_lr
// just tracks the flag and rt_wifi_active is always false.
volatile bool g_lr;

#if RT_STAGE >= 1
// The default protocol set ESP-IDF would pick for 2.4GHz on a chip with 11ax support - kept
// explicit rather than read back, since read-modify-write against a driver that is about to
// be stopped is more moving parts than just stating what we want.
#define WIFI_PROTO_BASE (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX)

// What the driver is actually doing right now, so wifi_apply() can tell a real change from a
// repeat and leave a working AP (and any phone on it) alone when nothing needs to move.
static bool s_wifi_on;
static bool s_wifi_lr;

// Which modes need the Wi-Fi radio at all.
//
// This is the fix for the thing low contention never actually did. "Isolate 802.15.4" used to
// mean "stop calling esp_now_send()", which changes nothing about the antenna: the SoftAP goes
// on beaconing every ~100ms and keeps its receiver up continuously to hear probes, and both of
// those outrank 802.15.4 in the coex arbiter - which hardcodes ordinary 802.15.4 tx/rx at its
// lowest priority tier. The result was ESP_IEEE802154_TX_ERR_COEXIST on every single frame,
// in a mode whose entire purpose was to give 802.15.4 a clear run. So the driver stops.
//
// ESP-NOW rides the same radio, so this is also what makes "espnow" and "wifi+phone" the two
// modes Wi-Fi stays up for - and it is why nothing here needs to care about the ESP-NOW tx
// gate: with the driver stopped there is no interface to send on in the first place.
static bool wifi_wanted(int lc)
{
    return lc == 0 || lc == CH_ESPNOW + 1 || lc == LC_WIFI_UI;
}

// Idempotent: brings the driver to whatever g_lc and g_lr currently ask for. Both a mode
// change and an LR toggle route through here, because both want the same stop/reconfigure/
// start sequence and doing it twice for one button press would bounce the AP for no reason.
//
// LR needs that full reinit rather than a live protocol change (CONTEXT.md, established by the
// owner's earlier testing). Merely having WIFI_PROTOCOL_LR in the list is enough to blind a
// phone to the SoftAP even though it does not stop other ESPs reaching it - so it is applied
// to both interfaces together, matching that they share one radio.
static void wifi_apply(void)
{
    const bool want    = wifi_wanted(g_lc);
    const bool want_lr = g_lr;

    if (s_wifi_on == want && (!want || s_wifi_lr == want_lr)) {
        return;
    }

    if (s_wifi_on) {
        RT_TRY(TAG, esp_wifi_stop());
        s_wifi_on = false;
    }

    if (!want) {
        // Not an error and not a failure to configure something: the radio is off on purpose,
        // and saying so plainly is the whole point - a silent Wi-Fi stack is exactly what the
        // old code looked like while it was still holding the antenna.
        ESP_LOGI(TAG, "wifi stopped - low contention on %s", rt_chan_name[g_lc - 1]);
        return;
    }

    const uint8_t proto = WIFI_PROTO_BASE | (want_lr ? WIFI_PROTOCOL_LR : 0);
    RT_TRY(TAG, esp_wifi_set_protocol(WIFI_IF_STA, proto));
    RT_TRY(TAG, esp_wifi_set_protocol(WIFI_IF_AP, proto));

    RT_TRY(TAG, esp_wifi_start());
    RT_TRY(TAG, esp_wifi_set_channel(WIFI_CHAN, WIFI_SECOND_CHAN_NONE));
    RT_TRY(TAG, esp_wifi_set_max_tx_power(78));  // 0.25dBm units, ~19.5dBm

    s_wifi_on = true;
    s_wifi_lr = want_lr;
    rt_espnow_resume();  // no-op until rt_espnow_start() has run

    ESP_LOGI(TAG, "wifi started: lr=%s", want_lr ? "on" : "off");
}
#endif

bool rt_wifi_active(void)
{
#if RT_STAGE >= 1
    return s_wifi_on;
#else
    return false;
#endif
}

void rt_set_lr(bool lr)
{
    if (lr == g_lr) {
        return;
    }
    g_lr = lr;
#if RT_STAGE >= 1
    wifi_apply();
#endif
    // A PHY change makes prior RSSI/loss numbers incomparable to what comes next.
    rt_stats_reset();
}

// Called by rt_set_lc() after every mode change - see the comment on the declaration in rt.h
// for what a mode change is actually supposed to mean at the radio level.
void rt_apply_lc_radios(int lc)
{
    // LC_WIFI_UI exists specifically so a phone can reach the AP, which only works with LR
    // off, so entering it forces LR off regardless of where the mode change came from. Done
    // before wifi_apply() below rather than after, so the driver is brought up once with the
    // right protocol list instead of started and immediately restarted.
    if (lc == LC_WIFI_UI && g_lr) {
        rt_set_lr(false);
    }
#if RT_STAGE >= 1
    wifi_apply();
#endif
    rt_154_set_lc(lc);
}

#if RT_STAGE >= 1
static void wifi_start(void)
{
    RT_TRY(TAG, esp_netif_init());
    RT_TRY(TAG, esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    RT_TRY(TAG, esp_wifi_init(&cfg));
    RT_TRY(TAG, esp_wifi_set_storage(WIFI_STORAGE_RAM));
    // APSTA, not STA: ESP-NOW keeps using the STA side exactly as before, and the AP side
    // exists so a phone has something to see and reach - only useful once LR is off, which is
    // why LC_WIFI_UI (below) is the mode that forces that.
    RT_TRY(TAG, esp_wifi_set_mode(WIFI_MODE_APSTA));

    wifi_config_t ap = { 0 };
    char ssid[sizeof(ap.ap.ssid) + 1];
    snprintf(ssid, sizeof(ssid), "ESPRT-%02X", rt_node_id());
    memcpy(ap.ap.ssid, ssid, strlen(ssid));
    ap.ap.ssid_len       = strlen(ssid);
    ap.ap.channel        = WIFI_CHAN;
    ap.ap.authmode       = WIFI_AUTH_OPEN;   // throwaway instrument, not a product
    ap.ap.max_connection = 4;
    ap.ap.ftm_responder  = true;             // costs nothing to set now; FTM itself is later
    RT_TRY(TAG, esp_wifi_set_config(WIFI_IF_AP, &ap));

    wifi_apply();  // boots non-LR (g_lr is still false); LR is opt-in, never silently on
}
#endif

// Tap cycles low-contention mode. Held past LONG_MS and released: reset to all-on - unchanged
// from before. Held past LR_HOLD_MS (fires immediately, no release needed): toggle LR. Three
// tiers off one button, same debounce-by-polling approach as always.
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
        } else if (now_down && down && !fired && (rt_ms() - t_down) > LR_HOLD_MS) {
            fired = true;
            rt_set_lr(!g_lr);
        } else if (!now_down && down) {
            down = false;
            if (!fired) {
                const uint32_t held = rt_ms() - t_down;
                if (held > LONG_MS) {
                    rt_set_lc(0);
                } else {
                    rt_set_lc((g_lc + 1) % LC_COUNT);
                }
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

    ESP_LOGI(TAG, "running. GPIO9: tap = next low-contention mode, hold = all radios, "
                  "long-hold (%dms) = toggle LR", LR_HOLD_MS);

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

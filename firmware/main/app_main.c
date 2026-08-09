// EspRangeTest - M0: observable skeleton.
//
// No radios yet. What exists here is the spine everything else hangs off: the observation
// table, the mode state machine, the GPIO9 button, and a serial dump of the peer x channel
// table. That last part is the point of M0 - a milestone with no way to see its numbers is
// a milestone that cannot be tested, so the rig becomes observable before any radio work
// starts.
//
// Boot also reports what this chip and build can actually do. The boards in play are not
// identical (WROOM-1 and MINI-1, different antennas and flash sizes, another board
// arriving, plus C3/S3 spares with no 802.15.4 at all), so capabilities are probed and
// printed rather than assumed.

#include <stdio.h>
#include <string.h>

#include "esp_chip_info.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"

#include "rt_button.h"
#include "rt_chan.h"
#include "rt_mode.h"
#include "rt_obs.h"
#include "rt_payload.h"
#include "rt_report.h"

static const char *TAG = "esprangetest";

#define REPORT_PERIOD_MS 2000
#define REPORT_BUF_LEN   2048

static rt_obs_table_t  s_obs;
static rt_mode_state_t s_mode;
static uint8_t         s_node_id;
static rt_chan_mask_t  s_chans;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// Channels this build can actually drive. C3 and S3 have no 802.15.4 radio, so those
// channels must never be offered rather than silently reporting nothing.
static rt_chan_mask_t probe_channels(void)
{
    rt_chan_mask_t m = RT_CHAN_BIT(RT_CHAN_BLE_ADV_CODED) | RT_CHAN_BIT(RT_CHAN_BLE_CONN) |
                       RT_CHAN_BIT(RT_CHAN_ESPNOW) | RT_CHAN_BIT(RT_CHAN_ESPNOW_LR) |
                       RT_CHAN_BIT(RT_CHAN_WIFI_BEACON) |
                       RT_CHAN_BIT(RT_CHAN_WIFI_LR_BEACON) | RT_CHAN_BIT(RT_CHAN_FTM);
#if SOC_IEEE802154_SUPPORTED
    m |= RT_CHAN_BIT(RT_CHAN_IEEE802154);
#endif
    return m;
}

static void log_identity(void)
{
    esp_chip_info_t info;
    esp_chip_info(&info);

    uint8_t mac[6] = { 0 };
    esp_read_mac(mac, ESP_MAC_BASE);

    // Only a handful of boards will ever be in play, so the low MAC byte is a perfectly
    // good node id - and the full MAC is printed alongside it so two boards can always be
    // told apart by eye.
    s_node_id = mac[5];

    ESP_LOGI(TAG, "=== EspRangeTest M0 ===");
    ESP_LOGI(TAG, "chip: %s, %d core(s), silicon revision v%d.%d",
             CONFIG_IDF_TARGET, info.cores, info.revision / 100, info.revision % 100);
    ESP_LOGI(TAG, "idf: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "mac: %02X:%02X:%02X:%02X:%02X:%02X -> node_id 0x%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], s_node_id);
    ESP_LOGI(TAG, "802.15.4 radio: %s",
#if SOC_IEEE802154_SUPPORTED
             "yes"
#else
             "no (channel disabled on this target)"
#endif
    );

    char chans[256];
    size_t off = 0;
    for (int c = 0; c < RT_CHAN_COUNT; c++) {
        if ((s_chans & RT_CHAN_BIT(c)) == 0u) {
            continue;
        }
        const int n = snprintf(chans + off, sizeof(chans) - off, "%s%s",
                               off ? " " : "", rt_chan_name((rt_chan_t)c));
        if (n > 0 && (size_t)n < sizeof(chans) - off) {
            off += (size_t)n;
        }
    }
    ESP_LOGI(TAG, "channels: %s", chans);
    ESP_LOGI(TAG, "payload: %d bytes/frame, v%d", RT_PAYLOAD_SIZE, RT_PAYLOAD_VER);
}

static void apply_mode(const char *why)
{
    const rt_radio_mask_t radios = rt_mode_radios(s_mode.mode, s_mode.keep_ui_ble);

    // This board's own contention state is part of what every sample means, so samples
    // taken under the previous mode must not be averaged into the new one.
    rt_obs_clear_all(&s_obs);

    ESP_LOGI(TAG, "mode -> %s (%s) epoch=%u ui_ble=%s radios=0x%03X",
             rt_mode_name(s_mode.mode), why, s_mode.epoch,
             s_mode.keep_ui_ble ? "on" : "off", (unsigned)radios);

    // M1+ : radio drivers read this mask and bring themselves up or down accordingly.
    // Crossing the LR boundary additionally needs a full Wi-Fi stop/start rather than a
    // live protocol change (see rt_mode_needs_wifi_reinit).
}

static void on_button(rt_button_event_t ev, void *ctx)
{
    (void)ctx;
    const uint32_t t = now_ms();

    switch (ev) {
    case RT_BUTTON_TAP:
        if (rt_mode_next(&s_mode, t)) {
            apply_mode("button tap");
        }
        break;
    case RT_BUTTON_LONG:
        if (rt_mode_home(&s_mode, t)) {
            apply_mode("button long press");
        } else {
            ESP_LOGI(TAG, "already in %s", rt_mode_name(s_mode.mode));
        }
        break;
    default:
        break;
    }
}

static void report_task(void *pv)
{
    (void)pv;
    static char buf[REPORT_BUF_LEN];

    for (;;) {
        const uint32_t t = now_ms();

        // Auto-revert exists for the board left at the far end: with the UI link off it
        // has no radio the phone can reach, and the button is out of walking range.
        if (rt_mode_tick(&s_mode, t)) {
            apply_mode("auto-revert (no UI contact)");
        }

        rt_report_render(&s_obs, &s_mode, s_node_id, t, buf, sizeof(buf));
        fputs(buf, stdout);
#if CONFIG_RT_SIM_PEER
        // Peer 0xEE is fabricated. Say so on every single report, so a screenshot or a
        // scrollback taken out of context can never be mistaken for measured data.
        fputs("  ** peer 0xEE is SIMULATED (CONFIG_RT_SIM_PEER) - not measured **\n",
              stdout);
#endif
        fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(REPORT_PERIOD_MS));
    }
}

#if CONFIG_RT_SIM_PEER
// Synthetic peer, for bringing the rig up before any radio exists. It exercises the whole
// observation path - sequence gaps, RSSI aggregation, staleness, the serial table - on
// real hardware with a single board on the bench. Turn it off before collecting anything
// that matters; the report marks it, so simulated numbers cannot be mistaken for measured
// ones.
static void sim_task(void *pv)
{
    (void)pv;
    uint16_t seq = 0;
    uint32_t tick = 0;

    ESP_LOGW(TAG, "SIMULATED PEER ENABLED - readings below are fabricated, not measured");

    for (;;) {
        const uint32_t t = now_ms();

        // Drifts from strong to unreachable and back, dropping ever more packets as it
        // fades, so the report can be watched doing the thing it exists to do.
        const uint32_t phase = (tick / 10u) % 20u;
        const int8_t   rssi  = (int8_t)(-40 - (int)(phase * 3u));
        const uint32_t drop  = phase;  // 0 -> 19 packets dropped per send as it fades

        seq = (uint16_t)(seq + 1u + drop);

        const rt_obs_t o = {
            .t_ms = t, .seq = seq, .rssi = rssi, .mode_id = (uint8_t)s_mode.mode,
            .rx_phy = RT_PHY_BLE_CODED_S8, .tx_phy = RT_PHY_BLE_CODED_S8, .tx_dbm = 9,
            .noise_floor = RT_NOISE_NA, .lqi = RT_LQI_NA,
        };
        rt_obs_add(&s_obs, 0xEE, RT_CHAN_BLE_ADV_CODED, s_mode.epoch, &o);

        tick++;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    s_chans = probe_channels();
    log_identity();

    rt_obs_table_init(&s_obs);
    rt_mode_init(&s_mode, now_ms());
    apply_mode("boot");

    rt_button_start(on_button, NULL);

    xTaskCreate(report_task, "rt_report", 4096, NULL, 4, NULL);
#if CONFIG_RT_SIM_PEER
    xTaskCreate(sim_task, "rt_sim", 3072, NULL, 3, NULL);
#endif
}

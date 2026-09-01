// BLE extended advertising on the coded PHY, plus a coded-PHY scanner.
//
// This is the one that matters. Coded PHY S=8 is the long-range hypothesis, but S=8 is only
// reliably negotiated after a connection is established - and a connection can only be
// established while close. If you walk out and lose it, and adverts are not actually going
// out at S=8, that loss is permanent. So the question is whether adverts *alone* carry data
// at S=8 range, with no connection involved. That is what this file tests.
//
// Caveat worth knowing when reading the numbers: the C6 is Bluetooth 5.3, and the feature
// that lets a receiver read back S=2 versus S=8 (Advertising Coding Selection) is 5.4. We
// ask the controller for coded and it reports "coded" - which of the two codings it actually
// picked is not visible. Measured range is the ground truth here, not a status field.

#include <string.h>

#include "esp_bt.h"   // esp_ble_tx_power_set - controller-level, works under NimBLE
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nimble/hci_common.h"  // BLE_HCI_LE_PHY_CODED, BLE_HCI_SCAN_FILT_NO_WL
#include "host/ble_hs.h"
#include "host/util/util.h"

#include "rt.h"

static const char *TAG = "ble";

#define ADV_INSTANCE  0
#define ADV_PERIOD_MS 500

// A range, not a point - same reasoning as the UI timings in rt_ui.c. The beacon's measured
// rate is set by ADV_PERIOD_MS (one new sequence number per refresh), not by this, so letting
// the controller pick anywhere in the window costs the measurement nothing and gives it
// somewhere to go when the antenna is busy.
#define ADV_ITVL_MIN  0x0100  // 0.625ms units -> 160ms
#define ADV_ITVL_MAX  0x0180  // -> 240ms

// Scan duty cycle. window == interval is a 100% duty receiver - the antenna claimed
// permanently, so nothing below it in the coex arbiter (which is to say 802.15.4) ever sees a
// gap. Measured: in all-radios mode that was 100.0% of 802.15.4 transmits refused, and with
// the scanner off it was 0%.
//
// Continuous is right while ble_adv is the channel under test: it is the thing being measured
// and nothing else is transmitting. It is wrong in all-radios mode, where the point is to
// measure all three *at the same instant* - which is the only way to compare them without
// aiming, multipath and where you happened to be standing varying between runs. A duty-cycled
// scanner loses coded adverts it would otherwise have heard; that loss is the price of the
// comparison, not a defect, and it is cheaper than a channel that cannot transmit at all.
#define SCAN_ITVL_SOLO   0x0060  // 0.625ms units -> 60ms, window == interval
#define SCAN_ITVL_SHARED 0x0140  // -> 200ms
#define SCAN_WIN_SHARED  0x0080  // -> 80ms, so 40% duty and 60% left for everyone else

static uint8_t s_own_addr_type;
static bool    s_ready;

// What the controller actually gave us when asked, which is what goes into the packet - it
// picks the nearest power it supports, not the one requested.
static int8_t s_adv_power;

// Set from whatever context changed the level; acted on by adv_task. Advertising power is
// fixed at ble_gap_ext_adv_configure() time, so changing it means reconfiguring the instance,
// which is not something to do from the button task or a GATT write callback.
static volatile bool s_pwr_dirty;

// Cleared until rt_ble_apply_power() has run, or set immediately if BLE never starts. Read by
// app_main before it powers the antenna switch.
static volatile bool s_pwr_applied;

bool rt_ble_power_ready(void)
{
    return s_pwr_applied;
}

// dBm to the controller's power index. Levels are 3dB apart from -15 (index 3) to +20
// (index 15) - see esp_power_level_t in esp_bt.h.
static esp_power_level_t pwr_level(int dbm)
{
    int idx = 3 + (dbm + 15) / 3;
    if (idx < ESP_PWR_LVL_N15) idx = ESP_PWR_LVL_N15;
    if (idx > ESP_PWR_LVL_P20) idx = ESP_PWR_LVL_P20;
    return (esp_power_level_t)idx;
}

void rt_ble_apply_power(void)
{
    // Advertising power is a configure-time parameter of the instance, so the beacon and the
    // phone-UI advert are rebuilt on their own cycles - flagged here, acted on there.
    s_pwr_dirty = true;

    // ...but advertising is not the only thing BLE transmits. A connection transmits too, and
    // its power comes from the controller, not from any advertising instance. Left alone it
    // defaults to +3dBm (documented in esp_bt.h: "If none of power type is set, system will
    // use ESP_PWR_LVL_P3"), which is neither the level asked for nor visible anywhere. That
    // was a genuine hole: every phone connection this project has ever made ran at +3dBm
    // regardless of what the beacon was set to.
    //
    // DEFAULT covers future connections; the CONN_HDL entries cover any already open. SCAN is
    // set for completeness - scanning here is passive, so it never actually transmits.
    const esp_power_level_t lvl = pwr_level(rt_power_dbm(CH_BLE_ADV));
    RT_TRY(TAG, esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, lvl));
    RT_TRY(TAG, esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, lvl));
    RT_TRY(TAG, esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, lvl));
    for (int h = ESP_BLE_PWR_TYPE_CONN_HDL0; h <= ESP_BLE_PWR_TYPE_CONN_HDL8; h++) {
        esp_ble_tx_power_set((esp_ble_power_type_t)h, lvl);
    }
    s_pwr_applied = true;
}
static bool    s_scanning;
static bool    s_scan_solo;   // which duty cycle s_scanning is currently running at

// Advertising payload: one manufacturer-specific AD structure wrapping rt_pkt_t.
static int set_adv_data(void)
{
    rt_pkt_t p;
    rt_fill(&p, CH_BLE_ADV, s_adv_power);

    uint8_t ad[2 + sizeof(rt_pkt_t)];
    ad[0] = 1 + sizeof(rt_pkt_t);  // length of what follows
    ad[1] = 0xFF;                  // manufacturer specific data
    memcpy(&ad[2], &p, sizeof(p));

    struct os_mbuf *buf = os_msys_get_pkthdr(sizeof(ad), 0);
    if (buf == NULL) {
        return BLE_HS_ENOMEM;
    }
    int rc = os_mbuf_append(buf, ad, sizeof(ad));
    if (rc != 0) {
        os_mbuf_free_chain(buf);
        return rc;
    }
    return ble_gap_ext_adv_set_data(ADV_INSTANCE, buf);
}

static int adv_gap_cb(struct ble_gap_event *event, void *arg)
{
    (void)event; (void)arg;
    return 0;
}

static int start_adv(void)
{
    struct ble_gap_ext_adv_params params = { 0 };

    // Non-connectable, non-scannable: this is a beacon, and anything that invites a
    // connection would change what is being measured.
    params.connectable   = 0;
    params.scannable     = 0;
    params.legacy_pdu    = 0;  // legacy PDUs cannot carry coded PHY
    params.own_addr_type = s_own_addr_type;
    params.primary_phy   = BLE_HCI_LE_PHY_CODED;
    params.secondary_phy = BLE_HCI_LE_PHY_CODED;
    params.itvl_min      = ADV_ITVL_MIN;
    params.itvl_max      = ADV_ITVL_MAX;
    params.tx_power      = rt_power_dbm(CH_BLE_ADV);
    params.sid           = 0;

    int8_t selected_tx_power = 0;
    int rc = ble_gap_ext_adv_configure(ADV_INSTANCE, &params, &selected_tx_power,
                                       adv_gap_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ext_adv_configure rc=%d", rc);
        return rc;
    }

    rc = set_adv_data();
    if (rc != 0) {
        ESP_LOGE(TAG, "ext_adv_set_data rc=%d", rc);
        return rc;
    }

    rc = ble_gap_ext_adv_start(ADV_INSTANCE, 0, 0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ext_adv_start rc=%d", rc);
        return rc;
    }

    s_adv_power = selected_tx_power;
    rt_power_set_actual(CH_BLE_ADV, selected_tx_power);
    ESP_LOGI(TAG, "coded-PHY adverts up, asked %d dBm, got %d dBm",
             rt_power_dbm(CH_BLE_ADV), selected_tx_power);
    return 0;
}

// Pull our payload back out of the manufacturer-specific AD structure.
static void handle_adv_report(const struct ble_gap_ext_disc_desc *d)
{
    const uint8_t *p   = d->data;
    int            rem = d->length_data;

    while (rem >= 2) {
        const int len  = p[0];
        const int type = p[1];
        if (len < 1 || len > rem - 1) {
            return;
        }
        if (type == 0xFF && len - 1 >= (int)sizeof(rt_pkt_t)) {
            rt_rx(&p[2], len - 1, CH_BLE_ADV, d->rssi, 0);
            return;
        }
        p   += len + 1;
        rem -= len + 1;
    }
}

static int scan_gap_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_EXT_DISC) {
        handle_adv_report(&event->ext_disc);
    }
    return 0;
}

// solo: ble_adv is the channel under test, so listen continuously. Otherwise duty-cycle, so
// 802.15.4 and Wi-Fi have somewhere to go.
static void start_scan(bool solo)
{
    struct ble_gap_ext_disc_params coded = { 0 };
    coded.itvl    = solo ? SCAN_ITVL_SOLO : SCAN_ITVL_SHARED;
    coded.window  = solo ? SCAN_ITVL_SOLO : SCAN_WIN_SHARED;
    coded.passive = 1;

    // uncoded params are required by the API but disabled here - scanning 1M as well would
    // halve the time spent listening on coded, which is the channel under test.
    const int rc = ble_gap_ext_disc(s_own_addr_type, 0, 0, 0, BLE_HCI_SCAN_FILT_NO_WL, 0,
                                    NULL, &coded, scan_gap_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ext_disc rc=%d", rc);
        return;
    }
    s_scanning  = true;
    s_scan_solo = solo;
    ESP_LOGI(TAG, "scanning coded PHY, %s",
             solo ? "continuously (ble_adv is the channel under test)"
                  : "40% duty (sharing the antenna with the other radios)");
}

// Quiet: callers say why, since this is used both to stop scanning altogether and to drop the
// scan before restarting it at a different duty cycle.
static void stop_scan(void)
{
    if (!s_scanning) {
        return;
    }
    ble_gap_disc_cancel();
    s_scanning = false;
}

// Refresh the advert so each one carries a new sequence number. Data cannot be swapped while
// an instance is running, so this stops and restarts it.
static void adv_task(void *pv)
{
    (void)pv;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(rt_jitter_ms(ADV_PERIOD_MS)));
        if (!s_ready) {
            continue;
        }
        if (rt_tx_enabled(CH_BLE_ADV)) {
            ble_gap_ext_adv_stop(ADV_INSTANCE);
            if (s_pwr_dirty) {
                // Power is a configure-time parameter, so the whole instance is rebuilt.
                s_pwr_dirty = false;
                start_adv();
            } else if (set_adv_data() == 0) {
                ble_gap_ext_adv_start(ADV_INSTANCE, 0, 0);
            }
            // Also catches a mode change between all-radios and the ble_adv test, which needs
            // the same scan restarted at a different duty cycle.
            const bool solo = (g_lc == CH_BLE_ADV + 1);
            if (!s_scanning || s_scan_solo != solo) {
                stop_scan();
                start_scan(solo);
            }
        } else {
            ble_gap_ext_adv_stop(ADV_INSTANCE);
            if (s_scanning) {
                stop_scan();
                ESP_LOGI(TAG, "coded PHY scan stopped (low contention on another channel)");
            }
        }
    }
}

static void on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    if (ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) {
        ESP_LOGE(TAG, "no usable address");
        return;
    }
    // Before anything advertises or connects. The controller's own default is +3dBm and
    // nothing else sets it, so without this a connection would transmit at +3 no matter what
    // the beacon was configured for - which is the state every previous build shipped in.
    rt_ble_apply_power();
    s_pwr_dirty = false;  // start_adv() below already picks up the current level

    if (start_adv() == 0) {
        s_ready = true;
    }
    // Same gate adv_task applies, applied here too rather than left for its first tick: the
    // scanner is the one continuous claim on the antenna this board makes, and half a second
    // of it is half a second of a channel under test not getting what the mode promised. In
    // practice the board always boots at lc=0, so this only matters if that ever stops
    // being true - which is exactly the kind of assumption that put 802.15.4 at 100% loss.
    if (rt_tx_enabled(CH_BLE_ADV)) {
        start_scan(g_lc == CH_BLE_ADV + 1);
    }
#if RT_STAGE >= 4
    rt_ui_on_sync(s_own_addr_type);
#endif
}

static void host_task(void *pv)
{
    (void)pv;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void rt_ble_start(void)
{
    // The adv_task below stops and restarts the coded-PHY instance every 500ms so each
    // advert carries a fresh sequence number - that restart is how loss gets measured, not a
    // bug. NimBLE logs each stop/start at INFO, which drowns the report in the same noise;
    // silence just that tag.
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    const esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        // Losing BLE costs the coded-PHY channel and the phone UI, but ESP-NOW and 802.15.4
        // can still produce a useful walk, so this must not be fatal.
        ESP_LOGE(TAG, "nimble_port_init -> %s; BLE disabled", esp_err_to_name(err));
        s_pwr_applied = true;  // nothing will transmit, so nothing is waiting on us
        return;
    }
#if RT_STAGE >= 4
    rt_ui_init();  // GATT services must be registered before the host starts
#endif
    ble_hs_cfg.sync_cb = on_sync;
    nimble_port_freertos_init(host_task);
    xTaskCreate(adv_task, "ble_adv", 4096, NULL, 4, NULL);
}

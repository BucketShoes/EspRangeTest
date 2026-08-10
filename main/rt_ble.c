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
#define ADV_ITVL      0x0100  // 0.625ms units -> 160ms

static uint8_t s_own_addr_type;
static bool    s_ready;

// Advertising payload: one manufacturer-specific AD structure wrapping rt_pkt_t.
static int set_adv_data(void)
{
    rt_pkt_t p;
    rt_fill(&p, CH_BLE_ADV, 9);

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
    params.itvl_min      = ADV_ITVL;
    params.itvl_max      = ADV_ITVL;
    params.tx_power      = 9;
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

    ESP_LOGI(TAG, "coded-PHY adverts up, tx power %d dBm", selected_tx_power);
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

static void start_scan(void)
{
    struct ble_gap_ext_disc_params coded = { 0 };
    coded.itvl   = 0x0060;
    coded.window = 0x0060;  // window == interval: listen continuously
    coded.passive = 1;

    // uncoded params are required by the API but disabled here - scanning 1M as well would
    // halve the time spent listening on coded, which is the channel under test.
    const int rc = ble_gap_ext_disc(s_own_addr_type, 0, 0, 0, BLE_HCI_SCAN_FILT_NO_WL, 0,
                                    NULL, &coded, scan_gap_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ext_disc rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "scanning coded PHY");
    }
}

// Refresh the advert so each one carries a new sequence number. Data cannot be swapped while
// an instance is running, so this stops and restarts it.
static void adv_task(void *pv)
{
    (void)pv;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ADV_PERIOD_MS));
        if (!s_ready) {
            continue;
        }
        if (rt_tx_enabled(CH_BLE_ADV)) {
            ble_gap_ext_adv_stop(ADV_INSTANCE);
            if (set_adv_data() == 0) {
                ble_gap_ext_adv_start(ADV_INSTANCE, 0, 0);
            }
        } else {
            ble_gap_ext_adv_stop(ADV_INSTANCE);
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
    if (start_adv() == 0) {
        s_ready = true;
    }
    start_scan();
    rt_ui_on_sync(s_own_addr_type);
}

static void host_task(void *pv)
{
    (void)pv;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void rt_ble_start(void)
{
    ESP_ERROR_CHECK(nimble_port_init());
    rt_ui_init();  // GATT services must be registered before the host starts
    ble_hs_cfg.sync_cb = on_sync;
    nimble_port_freertos_init(host_task);
    xTaskCreate(adv_task, "ble_adv", 4096, NULL, 4, NULL);
}

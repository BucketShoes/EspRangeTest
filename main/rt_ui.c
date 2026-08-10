// The phone-facing side: a connectable legacy advert and a tiny GATT service that streams
// the results table as text.
//
// Legacy advertising, on 1M, deliberately: Chrome's scanner cannot see extended or coded
// adverts at all, so the coded beacon in rt_ble.c is invisible to a browser. That beacon is
// the thing being measured; this is just the window onto it, and the two run as separate
// advertising instances.

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "rt.h"

static const char *TAG = "ui";

#define UI_INSTANCE  1
#define UI_ITVL_FAST 0x00A0  // 0.625ms units -> 100ms - normal operation, phone walking a live test
#define UI_ITVL_SLOW 0x0640  // -> 1000ms - low contention on some other channel: this link just
                              // needs to still work, not be snappy

// Connection parameters requested once a phone connects. Peripheral-preferred, so the phone
// may not honor them exactly, but it is what we ask for. "Fast" keeps the live-walk UI
// responsive; "slow" trades that for airtime back to whichever channel is under test - a
// laggy link and a slow initial connection are fine, this is a bench-test mode.
#define CONN_ITVL_FAST_MIN 0x0018  // 30ms
#define CONN_ITVL_FAST_MAX 0x0028  // 50ms
#define CONN_ITVL_SLOW_MIN 0x0140  // 400ms
#define CONN_ITVL_SLOW_MAX 0x0190  // 500ms
#define CONN_LATENCY_SLOW  4       // plus up to 4 skipped events - up to ~2.5s of quiet when idle
// Supervision timeout must clear (1 + latency) * itvl_max * 2 or the link drops on its own.
#define CONN_TIMEOUT_FAST 400  // 10ms units -> 4s
#define CONN_TIMEOUT_SLOW 800  // -> 8s, generous given the slow interval and added latency

// 9c7a0001-1b2c-4a7e-9a1e-5f6b2c3d4e5f and friends.
#define UUID_BASE(b1)                                                              \
    BLE_UUID128_INIT(0x5f, 0x4e, 0x3d, 0x2c, 0x6b, 0x5f, 0x1e, 0x9a,               \
                     0x7e, 0x4a, 0x2c, 0x1b, (b1), 0x00, 0x7a, 0x9c)

static const ble_uuid128_t UUID_SVC = UUID_BASE(0x01);
static const ble_uuid128_t UUID_TX  = UUID_BASE(0x02);  // notify: one CSV line per packet
static const ble_uuid128_t UUID_CMD = UUID_BASE(0x03);  // write: 1 byte, low-contention channel

static uint16_t s_tx_handle;
static uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static bool     s_subscribed;
static uint8_t  s_own_addr_type;
static char     s_name[16];
static int      s_last_lc = -1;  // forces the first apply_lc_ble_params() to actually run

static int start_adv(void);

// Ask the current connection (if any) for slower or faster parameters. Peripheral-preferred
// only - the central can decline - but it is what we ask for.
static void apply_conn_params(bool slow)
{
    if (s_conn == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    struct ble_gap_upd_params p = { 0 };
    p.itvl_min = slow ? CONN_ITVL_SLOW_MIN : CONN_ITVL_FAST_MIN;
    p.itvl_max = slow ? CONN_ITVL_SLOW_MAX : CONN_ITVL_FAST_MAX;
    p.latency  = slow ? CONN_LATENCY_SLOW : 0;
    p.supervision_timeout = slow ? CONN_TIMEOUT_SLOW : CONN_TIMEOUT_FAST;
    const int rc = ble_gap_update_params(s_conn, &p);
    if (rc != 0) {
        ESP_LOGW(TAG, "update_params rc=%d", rc);
    }
}

// Called whenever g_lc changes. Low contention on some other channel means this link only
// needs to keep working, not be responsive - see the UI_ITVL_SLOW / CONN_*_SLOW comments.
static void apply_lc_ble_params(void)
{
    const bool slow = g_lc != 0;
    if (s_conn == BLE_HS_CONN_HANDLE_NONE) {
        // Only touch the advertising instance while nothing is connected on it - restarting
        // it while connected would open a second, unwanted connection slot rather than
        // change anything about the link already up.
        ble_gap_ext_adv_stop(UI_INSTANCE);
        start_adv();
    } else {
        apply_conn_params(slow);
    }
}

static int cmd_write(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;

    uint8_t b = 0;
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, &b, sizeof(b), &len) != 0 || len < 1) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    rt_set_lc(b);
    return 0;
}

// Notify-only: nothing to read or write here, but NimBLE's sanity check rejects a NULL
// access_cb outright regardless of which flags are set, so this has to exist.
static int tx_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)ctxt; (void)arg;
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &UUID_SVC.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid       = &UUID_TX.u,
                .access_cb  = tx_access_cb,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_tx_handle,
            },
            {
                .uuid      = &UUID_CMD.u,
                .access_cb = cmd_write,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 },
        },
    },
    { 0 },
};

static int gap_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn = event->connect.conn_handle;
            ESP_LOGI(TAG, "phone connected");
            // Ask for coded PHY. If the phone declines, the link simply stays on 1M - this
            // must never be allowed to cost us the UI connection, so the result is logged
            // and otherwise ignored.
            const int rc = ble_gap_set_prefered_le_phy(s_conn, BLE_GAP_LE_PHY_CODED_MASK,
                                                       BLE_GAP_LE_PHY_CODED_MASK,
                                                       BLE_GAP_LE_PHY_CODED_S8);
            ESP_LOGI(TAG, "requested coded S=8 on the connection, rc=%d", rc);
            apply_conn_params(g_lc != 0);
        } else {
            start_adv();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "phone disconnected (reason %d)", event->disconnect.reason);
        s_conn       = BLE_HS_CONN_HANDLE_NONE;
        s_subscribed = false;
        start_adv();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_handle) {
            s_subscribed = event->subscribe.cur_notify;
        }
        return 0;

    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
        // 1 = 1M, 2 = 2M, 3 = coded. Which coding (S=2 or S=8) is not reported: that needs
        // Bluetooth 5.4 Advertising Coding Selection and the C6 is 5.3.
        ESP_LOGI(TAG, "connection PHY now tx=%d rx=%d",
                 event->phy_updated.tx_phy, event->phy_updated.rx_phy);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_adv();
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU %d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

static int start_adv(void)
{
    struct ble_gap_ext_adv_params p = { 0 };
    p.connectable   = 1;
    p.scannable     = 1;
    p.legacy_pdu    = 1;
    p.own_addr_type = s_own_addr_type;
    p.primary_phy   = BLE_HCI_LE_PHY_1M;
    p.secondary_phy = BLE_HCI_LE_PHY_1M;
    p.itvl_min      = g_lc != 0 ? UI_ITVL_SLOW : UI_ITVL_FAST;
    p.itvl_max      = p.itvl_min;
    p.tx_power      = 9;
    p.sid           = 1;

    int8_t pwr = 0;
    int rc = ble_gap_ext_adv_configure(UI_INSTANCE, &p, &pwr, gap_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ui adv configure rc=%d", rc);
        return rc;
    }

    // Flags + complete local name. The 128-bit service UUID will not fit alongside the name
    // in a 31-byte legacy advert, so the browser filters on the name prefix instead and
    // names the service in optionalServices.
    const size_t nlen = strlen(s_name);
    uint8_t ad[3 + 2 + sizeof(s_name)];
    size_t  n = 0;
    ad[n++] = 2;
    ad[n++] = 0x01;  // flags
    ad[n++] = 0x06;  // LE General Discoverable, BR/EDR not supported
    ad[n++] = (uint8_t)(nlen + 1);
    ad[n++] = 0x09;  // complete local name
    memcpy(&ad[n], s_name, nlen);
    n += nlen;

    struct os_mbuf *buf = os_msys_get_pkthdr(n, 0);
    if (buf == NULL) {
        return BLE_HS_ENOMEM;
    }
    if (os_mbuf_append(buf, ad, n) != 0) {
        os_mbuf_free_chain(buf);
        return BLE_HS_ENOMEM;
    }
    rc = ble_gap_ext_adv_set_data(UI_INSTANCE, buf);
    if (rc != 0) {
        ESP_LOGE(TAG, "ui adv set_data rc=%d", rc);
        return rc;
    }

    rc = ble_gap_ext_adv_start(UI_INSTANCE, 0, 0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ui adv start rc=%d", rc);
    }
    return rc;
}

void rt_ui_init(void)
{
    snprintf(s_name, sizeof(s_name), "ESPRT-%02X", rt_node_id());

    ble_svc_gap_init();
    ble_svc_gatt_init();

    // NimBLE return codes, not esp_err_t - and a GATT registration failure should disable
    // the phone UI, not take the whole rig down with it.
    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_count_cfg rc=%d - phone UI disabled", rc);
        return;
    }
    rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_add_svcs rc=%d - phone UI disabled", rc);
        return;
    }
    ble_svc_gap_device_name_set(s_name);
}

void rt_ui_on_sync(uint8_t own_addr_type)
{
    s_own_addr_type = own_addr_type;
    if (start_adv() == 0) {
        ESP_LOGI(TAG, "advertising as %s (legacy 1M, so Chrome can see it)", s_name);
    }
}

// One notification per line. Each line is complete and independent, so a marginal link
// gives a partial update rather than nothing at all.
void rt_ui_notify(void)
{
    // Polled here rather than driven from rt_set_lc() directly: this runs on the ui task,
    // not whatever task/ISR context a low-contention change was requested from (button task,
    // or the NimBLE host task via cmd_write). Every REPORT_MS is plenty responsive for a
    // deliberate, operator-commanded mode switch.
    if (g_lc != s_last_lc) {
        s_last_lc = g_lc;
        apply_lc_ble_params();
    }

    if (s_conn == BLE_HS_CONN_HANDLE_NONE || !s_subscribed) {
        return;
    }

    static char lines[1 + RT_MAX_PEERS * CH_COUNT][RT_LINE_MAX];
    const int   n = rt_snapshot_lines(lines, (int)(sizeof(lines) / sizeof(lines[0])));

    for (int i = 0; i < n; i++) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(lines[i], strlen(lines[i]));
        if (om == NULL) {
            return;  // out of buffers; the next cycle will carry the same state anyway
        }
        if (ble_gatts_notify_custom(s_conn, s_tx_handle, om) != 0) {
            return;
        }
    }
}

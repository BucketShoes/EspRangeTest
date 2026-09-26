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

// Advertising and connection timings are given as ranges, never as a single value, and the
// ranges are deliberately wide.
//
// A controller handed itvl_min == itvl_max has exactly one instant it may use, so every board
// running this firmware picks the same one and then holds it against everything else that
// wants the antenna at that moment - which is the whole problem this project is chasing. Given
// room, the controller can slide its events into gaps instead of colliding with them, and two
// boards that start out aligned drift apart on their own. Nothing here needs an event to
// happen at a particular time; it only needs it to happen.
#define UI_ITVL_FAST_MIN 0x00A0  // 0.625ms units -> 100ms
#define UI_ITVL_FAST_MAX 0x0140  // -> 200ms
// Slow, but still findable. 800-1600ms could not be reconnected to in any reasonable time -
// Chrome scans on its own duty cycle and two sparse schedules miss each other for a long while.
// These are the owner's numbers; do not "improve" them in either direction.
#define UI_ITVL_SLOW_MIN 0x03C0  // -> 600ms
#define UI_ITVL_SLOW_MAX 0x0640  // -> 1000ms

// Slow while some test other than BLE is on and ble_adv is not; fast otherwise.
//
// Slow is for handing airtime to a channel that is not BLE: every millisecond this link does
// not use is one espnow, 802.15.4 or FTM can. Fast everywhere else, because slowing this link
// where nothing is waiting for the antenna buys nothing and costs something real:
//
//   - with every test off, this link is the only thing on the air. Nothing gains from it
//     being slow, and it is the state the button restores to - the one that has to be
//     easiest to reconnect to. Every reconnect *starts* with seeing an advert, so a 1s advert
//     interval is a 1s-plus stall on each one.
//   - ble_adv is the BLE test itself, which is partly about whether a connection can be
//     established and held at all and asks for coded S=8 on it. Throttling the UI there
//     handicaps the thing under test.
//   - a slow connection interval means nothing can be sent *at all* until the next connection
//     event. Slowing it anywhere the results are wanted promptly does not make the reports
//     cheaper, it makes them late or absent - and a report not seen is a test not run.
//
// (This is the UI advert on UI_INSTANCE, the 1M control link. The coded-PHY measurement beacon
// is ADV_INSTANCE in rt_ble.c and nothing here has ever touched its interval.)
#define UI_SLOW() ((g_tests & ~RT_TEST_BLE_ADV) != 0 && (g_tests & RT_TEST_BLE_ADV) == 0)

bool rt_ui_slow(void)
{
    return UI_SLOW();
}

// Connection parameters requested once a phone connects. Peripheral-preferred, so the phone
// may not honor them exactly, but it is what we ask for. "Fast" keeps the live-walk UI
// responsive; "slow" trades that for airtime back to whichever channel is under test - a
// laggy link and a slow initial connection are fine, this is a bench-test mode.
#define CONN_ITVL_FAST_MIN 0x0010  // 20ms
#define CONN_ITVL_FAST_MAX 0x0050  // 100ms
// 350-500ms, the owner's numbers. 500ms is the ceiling because nothing can be sent until a
// connection event, and the report has to arrive while it is still worth reading.
//
// Latency stays at zero. Note what it actually costs, which is not what an earlier comment here
// claimed: a slave with data queued uses the next anchor point regardless of latency, so
// reports were never delayed by it. What latency delays is the *other* direction - a command
// from the phone can wait up to (1+latency) intervals before the board hears it. At latency 4
// and 500ms that is 2.5s of dead control, which is the part worth not having.
#define CONN_ITVL_SLOW_MIN 0x0118  // 1.25ms units -> 350ms
#define CONN_ITVL_SLOW_MAX 0x0190  // -> 500ms
#define CONN_LATENCY_SLOW  0
// Supervision timeout must clear (1 + latency) * itvl_max * 2 or the link drops on its own.
#define CONN_TIMEOUT_FAST 400  // 10ms units -> 4s
#define CONN_TIMEOUT_SLOW 1000 // -> 10s. Must clear (1+4)*750ms*2 = 7.5s; 8s left no margin.

// The packet log rides behind the report, as fast as the link will actually take it.
//
// Two cursors. The live one follows the block being written, so what is happening now reaches
// the phone within a report period whatever else is queued - connect to a grounded board and
// the drone's latest position is on the map at once, not after the board has finished
// describing the last twenty minutes. The backfill one works through whatever the page is
// missing, oldest first. Every block stands alone (see rt.h), which is what makes sending them
// out of order safe.
//
// How fast is decided by NimBLE's own answer, not by a guess at the link. The host never drops
// a notification it has accepted: it queues it and the link layer retransmits until it lands.
// What it does when the link cannot keep up is run out of buffers and refuse the next one - so a
// refusal is the overflow signal, and it counts whichever notification got it, report or log.
// The budget per report grows by half again every report that goes through clean and full, and
// halves on any refusal. A refused notification is never lost either: its cursor does not move,
// and the same bytes go next time.
//
// LOG_QUEUE_MAX bounds what can be waiting at once, so that even a clean run never lines up
// seconds of backlog in front of the next report on a link that is barely holding at range.
#define LOG_BUDGET_MIN  1
#define LOG_BUDGET_MAX  40    // 40 x 244 bytes a second - far past what coded S=8 can carry
#define LOG_QUEUE_MAX   12    // buffers in use beyond what was in use when the stream began
#define LOG_FREE_MIN    6     // and never below this many left for anything else

static int s_log_budget = 4;

// The report is never skipped, in any mode. A run whose numbers were not delivered did not
// happen, and a low-contention mode whose results never arrive is the most expensive kind of
// nothing - so airtime is bought by *slowing* this link (interval, advert rate), never by
// dropping data off it. Carrying the results is the cadence's job, not an optional extra.

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
static int      s_last_tests = -1;  // forces the first apply_test_ble_params() to actually run

// Bumped on every connection. A log request belongs to the connection it arrived on; one left
// over from a phone that has since gone is not something to keep streaming for.
static volatile uint32_t s_conn_gen;

// A log request, from the host task, picked up by rt_ui_notify() on the report task - which owns
// the stream and is the only thing that touches it.
static volatile bool     s_log_req;
static volatile uint32_t s_log_req_gen, s_log_req_session, s_log_req_block;
static volatile uint16_t s_log_req_off;

static struct {
    bool         on;
    uint32_t     gen;
    rt_log_cur_t live, back;
    uint32_t     back_end;    // backfill stops here; the live cursor started from it
    int          free0;       // NimBLE buffers free when it began, before any of ours queued
} s_log;

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

// ---- control-link PHY ----------------------------------------------------------------------
//
// Coded S=8 by default, 2M on request. See RT_CMD_PHY_* in rt.h for why those two and nothing
// between them.
volatile bool g_conn_2m;

// What the controller actually settled on, from BLE_GAP_EVENT_PHY_UPDATE_COMPLETE. Requested
// and achieved are both shipped in the report: the phone is free to decline, and the
// difference between coded S=8 and 2M is about 16x the airtime per byte - far too large a
// thing to assume went through because it was asked for.
static volatile uint8_t s_conn_phy_actual;  // 0 unknown, 1 = 1M, 2 = 2M, 3 = coded

uint8_t rt_conn_phy_actual(void)
{
    return (s_conn == BLE_HS_CONN_HANDLE_NONE) ? 0 : s_conn_phy_actual;
}

// Ask the current connection to move. Logged and otherwise ignored on failure: losing the UI
// connection over a PHY preference would be a far worse outcome than carrying the report at
// the wrong rate.
static void apply_conn_phy(void)
{
    if (s_conn == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    const uint8_t mask = g_conn_2m ? BLE_GAP_LE_PHY_2M_MASK : BLE_GAP_LE_PHY_CODED_MASK;
    // The S=8 preference only means anything when the coded mask is the one being asked for;
    // it is ignored for 2M, and passing it anyway keeps the call in one place.
    const int rc = ble_gap_set_prefered_le_phy(s_conn, mask, mask, BLE_GAP_LE_PHY_CODED_S8);
    ESP_LOGI(TAG, "requested %s on the connection, rc=%d", g_conn_2m ? "2M" : "coded S=8", rc);
}

void rt_set_conn_phy(bool two_m)
{
    if (two_m == g_conn_2m) {
        return;
    }
    g_conn_2m = two_m;
    s_conn_phy_actual = 0;  // unknown until the controller says otherwise
    apply_conn_phy();
}

// Called whenever g_tests changes. This link is never switched off - there is no test that
// does it - only slowed while another channel wants the antenna. See the UI_ITVL_SLOW /
// CONN_*_SLOW comments.
static void apply_test_ble_params(void)
{
    const bool slow = UI_SLOW();
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

    // One byte for the mode and LR commands, two for a power set - a dBm value does not fit
    // usefully in the spare bits of the first byte, and signed - and eleven for a log request.
    uint8_t  b[16] = { 0 };
    uint16_t len   = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, b, sizeof(b), &len) != 0 || len < 1) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (b[0] == RT_CMD_LOG_FROM) {
        if (len < 11) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        uint32_t session, block;
        uint16_t off;
        memcpy(&session, &b[1], 4);
        memcpy(&block, &b[5], 4);
        memcpy(&off, &b[9], 2);
        s_log_req_session = session;
        s_log_req_block   = block;
        s_log_req_off     = off;
        s_log_req_gen     = s_conn_gen;
        s_log_req         = true;
    } else if (b[0] == RT_CMD_LR_OFF || b[0] == RT_CMD_LR_ON) {
        rt_set_lr(b[0] == RT_CMD_LR_ON);
    } else if (b[0] == RT_CMD_ANT_INT || b[0] == RT_CMD_ANT_EXT) {
        rt_set_antenna(b[0] == RT_CMD_ANT_EXT);
    } else if (b[0] == RT_CMD_TX_UNMUTE || b[0] == RT_CMD_TX_MUTE) {
        rt_set_tx_mute(b[0] == RT_CMD_TX_MUTE);
    } else if (b[0] >= RT_CMD_GEO_FIX && b[0] <= RT_CMD_GEO_MOMENTUM) {
        rt_gnss_set_mode(b[0] - RT_CMD_GEO_FIX);
    } else if (b[0] >= RT_CMD_LED_OFF && b[0] <= RT_CMD_LED_BLINK) {
        rt_set_led(b[0] - RT_CMD_LED_OFF);
    } else if (b[0] == RT_CMD_PHY_CODED || b[0] == RT_CMD_PHY_2M) {
        rt_set_conn_phy(b[0] == RT_CMD_PHY_2M);
    } else if (b[0] == RT_CMD_STATS_RESET) {
        rt_stats_reset();
    } else if (b[0] >= RT_CMD_PWR_SET && b[0] <= RT_CMD_PWR_SET + CH_COUNT) {
        if (len < 2) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        rt_set_power(b[0] - RT_CMD_PWR_SET, (int8_t)b[1]);  // clamps per radio
    } else if (b[0] == RT_CMD_TESTS) {
        if (len < 2) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        rt_set_tests(b[1]);
    } else if (b[0] <= RT_CMD_LEGACY_LC_MAX) {
        // A page from before the switches: 0 was every radio, 1..3 one channel on its own, 4
        // ESP-NOW with the AP up for a phone.
        static const uint8_t legacy[] = {
            RT_TEST_ESPNOW | RT_TEST_BLE_ADV | RT_TEST_154 | RT_TEST_AP,
            RT_TEST_ESPNOW, RT_TEST_BLE_ADV, RT_TEST_154, RT_TEST_ESPNOW | RT_TEST_AP,
        };
        rt_set_tests(legacy[b[0]]);
    }
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
            s_conn_phy_actual = 0;
            s_conn_gen++;
            ESP_LOGI(TAG, "phone connected");
            // Whichever PHY is currently selected. If the phone declines, the link simply
            // stays on 1M - this must never be allowed to cost us the UI connection, so the
            // result is logged and otherwise ignored.
            apply_conn_phy();
            apply_conn_params(UI_SLOW());
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
        //
        // Kept, not just logged. This is the single biggest factor in what the control link
        // costs the channel under test - roughly 16x between 2M and coded S=8 - and serial is
        // exactly where nobody is looking during a range walk.
        s_conn_phy_actual = event->phy_updated.tx_phy;
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
    p.itvl_min      = UI_SLOW() ? UI_ITVL_SLOW_MIN : UI_ITVL_FAST_MIN;
    p.itvl_max      = UI_SLOW() ? UI_ITVL_SLOW_MAX : UI_ITVL_FAST_MAX;
    // The phone link shares the BLE level: it is the same radio, and a separate knob for the
    // control advert would be a fourth thing to get wrong for no measurement it enables.
    p.tx_power      = rt_power_dbm(CH_BLE_ADV);
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
    snprintf(s_name, sizeof(s_name), "%s", rt_node_name());

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

// Start streaming from where the page says it is up to. A request from another life of this
// board - it rebooted, and the page is holding a cursor into a log that no longer exists - or
// from before the oldest block still held, gets everything there is instead.
static void log_begin(uint32_t session, uint32_t block, uint16_t off)
{
    rt_log_st_t ls;
    rt_log_status(&ls);
    if (ls.cap == 0) {
        s_log.on = false;
        return;
    }
    if (session != ls.session || block > ls.newest || block < ls.oldest) {
        block = ls.oldest;
        off   = 0;
    }
    s_log.on       = true;
    s_log.free0    = os_msys_num_free();
    s_log.gen      = s_conn_gen;
    s_log.back_end = ls.newest;
    s_log.back     = (rt_log_cur_t){ block, off };
    s_log.live     = (rt_log_cur_t){ ls.newest, block == ls.newest ? off : 0 };
    ESP_LOGI(TAG, "log to phone from block %lu+%u, %lu blocks to backfill",
             (unsigned long)block, off, (unsigned long)(ls.newest - block));
}

// A notification out, and whether NimBLE took it. The one place anything is sent, so a refusal
// is seen the same way for the report and the log.
static bool send(const uint8_t *buf, int len)
{
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, (uint16_t)len);
    if (om == NULL) {
        return false;
    }
    // notify_custom frees the mbuf itself, whether or not it succeeds.
    return ble_gatts_notify_custom(s_conn, s_tx_handle, om) == 0;
}

static void log_backoff(void)
{
    s_log_budget /= 2;
    if (s_log_budget < LOG_BUDGET_MIN) {
        s_log_budget = LOG_BUDGET_MIN;
    }
}

static void log_stream(int cap)
{
    if (s_log_req) {
        s_log_req = false;
        if (s_log_req_gen == s_conn_gen) {
            log_begin(s_log_req_session, s_log_req_block, s_log_req_off);
        }
    }
    if (!s_log.on || s_log.gen != s_conn_gen) {
        s_log.on = false;
        return;
    }

    // Live first, backfill with whatever of the budget is left.
    uint8_t buf[RT_RPT_CHUNK_MAX];
    int sent = 0;
    while (sent < s_log_budget) {
        const int free_now = os_msys_num_free();
        if (s_log.free0 - free_now > LOG_QUEUE_MAX || free_now < LOG_FREE_MIN) {
            return;   // plenty in flight already: not a refusal, just not piling on
        }
        rt_log_cur_t *c = NULL, adv;
        int len = rt_log_chunk(&s_log.live, buf, cap, &adv);
        if (len > 0) {
            c = &s_log.live;
        } else if (s_log.back.block < s_log.back_end
                   && (len = rt_log_chunk(&s_log.back, buf, cap, &adv)) > 0) {
            c = &s_log.back;
        } else {
            return;   // caught up; the budget stays where it is
        }
        if (!send(buf, len)) {
            log_backoff();
            return;
        }
        *c = adv;
        sent++;
    }
    // Used it all and every one went: the link has room to spare.
    s_log_budget += s_log_budget / 2 + 1;
    if (s_log_budget > LOG_BUDGET_MAX) {
        s_log_budget = LOG_BUDGET_MAX;
    }
}

// One notification per line. Each line is complete and independent, so a marginal link
// gives a partial update rather than nothing at all.
void rt_ui_notify(void)
{
    // Polled here rather than driven from rt_set_tests() directly: this runs on the ui task,
    // not whatever task/ISR context a switch was requested from (button task, or the NimBLE
    // host task via cmd_write). Every REPORT_MS is plenty responsive for a deliberate,
    // operator-commanded change.
    if (g_tests != s_last_tests) {
        s_last_tests = g_tests;
        apply_test_ble_params();
    }

    // Advertising power is fixed when the instance is configured, so a level change needs the
    // advert rebuilding. Only safe while nothing is connected on it - restarting a connectable
    // instance mid-connection would open a second slot rather than change the existing link,
    // which is the same reason apply_lc_ble_params() leaves it alone when connected. A phone
    // that is already connected keeps the power it connected at until it drops.
    static int s_last_pwr = -128;
    if (g_pwr_dbm[CH_BLE_ADV] != s_last_pwr) {
        s_last_pwr = g_pwr_dbm[CH_BLE_ADV];
        if (s_conn == BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_ext_adv_stop(UI_INSTANCE);
            start_adv();
        }
    }

    if (s_conn == BLE_HS_CONN_HANDLE_NONE || !s_subscribed) {
        return;
    }

    // What this connection can actually carry in one notification, rather than what we asked
    // for at build time. A phone that negotiated a smaller MTU gets more, smaller chunks
    // instead of a silently truncated report.
    int cap = (int)ble_att_mtu(s_conn) - 3;
    if (cap > RT_RPT_CHUNK_MAX) {
        cap = RT_RPT_CHUNK_MAX;
    }

    // Wraps at 256, which is all the page needs: it only ever asks "is this the same report as
    // the chunk before it".
    static uint8_t s_gen;
    s_gen++;

    uint8_t         buf[RT_RPT_CHUNK_MAX];
    rt_rpt_state_t  st = { 0 };
    int             len;

    while ((len = rt_snapshot_chunk(buf, cap, s_gen, &st)) > 0) {
        if (!send(buf, len)) {
            // Out of buffers; the next cycle carries the same state anyway. Whatever filled
            // them, the log backs off - a report refused is the clearest sign the link is full.
            log_backoff();
            return;
        }
    }

    // After the report, never before it.
    log_stream(cap);
}

// FTM initiator (802.11mc fine timing measurement): find the other boards' APs, range to each.
//
// Every board's SoftAP is an FTM responder whenever its Wi-Fi driver is up (main.c). This is the
// other end: while RT_TEST_FTM is on, scan our own channel for APs named like ours that say they
// answer FTM, and range to them one at a time, round robin, with a pause between sessions.
//
// Not on a fixed rate like the measurement packets. An FTM session is a burst exchange lasting
// a good fraction of a second, only one can run at a time, and what it answers - how far away -
// changes at walking or flying pace, not at 4Hz. So sessions simply follow one another with a
// randomised gap, which is what leaves room on the antenna for the control link and whatever
// other test is on. The gap is the rate cap.
//
// A session that fails is a result, not a retry: it is counted and logged exactly like a
// success, because "tried, and nothing came back" is the range measurement.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "rt.h"

static const char *TAG = "ftm";

// Scanning: every SCAN_MS once something has been found, every SCAN_EMPTY_MS while nothing has.
// A responder is forgotten FORGET_MS after it was last seen in a scan or last answered a session,
// so a board that is switched off stops being tried - but only after a couple of minutes, because
// at the edge of range a probe response is as easily lost as anything else, and dropping a board
// the first time one scan misses it would stop measuring it exactly where it gets interesting.
#define SCAN_MS          30000
#define SCAN_EMPTY_MS    5000
#define SCAN_ACTIVE_MIN  50       // per-channel dwell for the active scan, ms - one channel only
#define SCAN_ACTIVE_MAX  120
#define SCAN_WAIT_MS     2000     // backstop on the scan-done event
#define SCAN_MAX_APS     20
#define FORGET_MS        120000

// Sessions. FRM_COUNT and BURST_PERIOD are what the initiator asks for (see
// wifi_ftm_initiator_cfg_t: 16/24/32/64 frames, bursts in 100ms units) - more frames average
// more noise away, and cost more airtime. GAP is the pause after each session, randomised so
// it does not settle into step with anything periodic.
#define FRM_COUNT        16
#define BURST_PERIOD     2        // 200ms between bursts
#define SESSION_WAIT_MS  3000     // backstop on the report event
#define GAP_MIN_MS       300
#define GAP_MAX_MS       900
#define IDLE_MS          500      // how often to look again while FTM is off

// "ESPRT-" and six hex digits - rt_node_name()'s pattern.
#define SSID_PREFIX      "ESPRT-"
#define SSID_PREFIX_LEN  6
#define SSID_LEN         (SSID_PREFIX_LEN + 6)

typedef struct {
    bool     used;
    uint32_t node;
    uint8_t  mac[6];
    uint8_t  chan;
    uint32_t seen_ms;        // last seen in a scan, or last answered

    bool     any;            // at least one session has ended, either way
    uint8_t  last_st;
    uint16_t dist_dm;        // last success
    uint16_t hist[RT_FTM_AVG];
    uint8_t  nhist, hhead;
    int8_t   rssi;           // last success
    uint16_t bits;           // last 16 sessions, bit0 newest: 1 = success
    uint8_t  nbits;
    uint32_t ok, fail;
    uint32_t last_ms;
} resp_t;

static portMUX_TYPE      s_mux = portMUX_INITIALIZER_UNLOCKED;
static resp_t            s_r[RT_MAX_PEERS];
static SemaphoreHandle_t s_scan_done, s_ftm_done;

// The last report, as the event handler copied it. Read by the task once s_ftm_done is given.
static volatile uint8_t  s_rep_mac[6];
static volatile uint8_t  s_rep_st;
static volatile uint32_t s_rep_dist_cm;
static volatile uint8_t  s_rep_n;

// For the serial report: what the last scan found, so "no responders" can say whether that was
// "heard nothing at all" or "heard APs, none of them ours".
static uint32_t s_scan_ms;
static uint16_t s_scan_aps, s_scan_ours;
static bool     s_scanned;

static const char *st_name(uint8_t st)
{
    switch (st) {
    case FTM_STATUS_SUCCESS:       return "ok";
    case FTM_STATUS_UNSUPPORTED:   return "unsupported";
    case FTM_STATUS_CONF_REJECTED: return "rejected";
    case FTM_STATUS_NO_RESPONSE:   return "no-response";
    case FTM_STATUS_FAIL:          return "fail";
    case FTM_STATUS_NO_VALID_MSMT: return "no-valid-msmt";
    case FTM_STATUS_USER_TERM:     return "ended";
    case RT_FTM_ST_NOSTART:        return "not-started";
    case RT_FTM_ST_TIMEOUT:        return "timeout";
    default:                       return "?";
    }
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base;
    if (id == WIFI_EVENT_SCAN_DONE) {
        xSemaphoreGive(s_scan_done);
    } else if (id == WIFI_EVENT_FTM_REPORT) {
        const wifi_event_ftm_report_t *r = data;
        for (int i = 0; i < 6; i++) {
            s_rep_mac[i] = r->peer_mac[i];
        }
        s_rep_st      = (uint8_t)r->status;
        s_rep_dist_cm = r->dist_est;
        s_rep_n       = r->ftm_report_num_entries;
        xSemaphoreGive(s_ftm_done);
    }
}

static int hexval(char c)
{
    return c >= '0' && c <= '9' ? c - '0'
         : c >= 'A' && c <= 'F' ? c - 'A' + 10
         : c >= 'a' && c <= 'f' ? c - 'a' + 10
                                : -1;
}

// The node id from an SSID of ours, or 0 if it is not one.
static uint32_t node_of(const uint8_t *ssid)
{
    if (memcmp(ssid, SSID_PREFIX, SSID_PREFIX_LEN) != 0 || ssid[SSID_LEN] != '\0') {
        return 0;
    }
    uint32_t v = 0;
    for (int i = SSID_PREFIX_LEN; i < SSID_LEN; i++) {
        const int h = hexval((char)ssid[i]);
        if (h < 0) {
            return 0;
        }
        v = (v << 4) | (uint32_t)h;
    }
    return v;
}

// A responder seen in a scan: refreshed if known, else added - over the stalest if full.
static void note_responder(uint32_t node, const uint8_t *mac, uint8_t chan, uint32_t now)
{
    portENTER_CRITICAL(&s_mux);
    resp_t *r = NULL, *spare = NULL;
    for (int i = 0; i < RT_MAX_PEERS && !r; i++) {
        if (s_r[i].used && s_r[i].node == node) {
            r = &s_r[i];
        } else if (!s_r[i].used) {
            if (!spare || spare->used) spare = &s_r[i];
        } else if (!spare || (spare->used && s_r[i].seen_ms < spare->seen_ms)) {
            spare = &s_r[i];   // the stalest, until an unused slot turns up
        }
    }
    if (!r) {
        r = spare;
        memset(r, 0, sizeof(*r));
        r->rssi = -128;
        r->used = true;
        r->node = node;
    }
    memcpy(r->mac, mac, 6);
    r->chan    = chan;
    r->seen_ms = now;
    portEXIT_CRITICAL(&s_mux);
}

static void scan(void)
{
    uint8_t prim = 1;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&prim, &sec);

    // Our own channel only. Every board sits on the same one, and hopping would take this
    // board's AP and ESP-NOW off it for the length of the scan.
    wifi_scan_config_t cfg = { 0 };
    cfg.channel              = prim;
    cfg.show_hidden          = false;
    cfg.scan_type            = WIFI_SCAN_TYPE_ACTIVE;
    cfg.scan_time.active.min = SCAN_ACTIVE_MIN;
    cfg.scan_time.active.max = SCAN_ACTIVE_MAX;

    xSemaphoreTake(s_scan_done, 0);
    const esp_err_t err = esp_wifi_scan_start(&cfg, false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "scan_start -> %s", esp_err_to_name(err));
        return;
    }
    if (xSemaphoreTake(s_scan_done, pdMS_TO_TICKS(SCAN_WAIT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "scan never finished; stopping it");
        esp_wifi_scan_stop();
        return;
    }

    uint16_t n = SCAN_MAX_APS;
    wifi_ap_record_t *ap = calloc(n, sizeof(*ap));
    if (ap == NULL) {
        esp_wifi_clear_ap_list();
        return;
    }
    // Copies up to n and frees the driver's whole list, however long it was.
    if (esp_wifi_scan_get_ap_records(&n, ap) != ESP_OK) {
        n = 0;
    }

    const uint32_t now = rt_ms();
    uint16_t ours = 0;
    for (int i = 0; i < n; i++) {
        const uint32_t node = node_of(ap[i].ssid);
        // Ours, not ourselves, and saying it answers FTM - a board without the responder built
        // in would only ever answer "unsupported", and nothing else is ours to range to.
        if (node == 0 || node == rt_node_id() || !ap[i].ftm_responder) {
            continue;
        }
        ours++;
        note_responder(node, ap[i].bssid, ap[i].primary, now);
    }
    free(ap);

    s_scan_ms   = now;
    s_scan_aps  = n;
    s_scan_ours = ours;
    s_scanned   = true;
}

static void forget_stale(uint32_t now)
{
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        if (s_r[i].used && now - s_r[i].seen_ms > FORGET_MS) {
            s_r[i].used = false;
        }
    }
    portEXIT_CRITICAL(&s_mux);
}

static int known(void)
{
    int k = 0;
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        k += s_r[i].used;
    }
    return k;
}

static void record(int idx, uint32_t node, uint8_t st, uint32_t dist_cm, int8_t rssi)
{
    const uint32_t now = rt_ms();
    const uint32_t dm  = (dist_cm + 5) / 10;
    const uint16_t d16 = st == FTM_STATUS_SUCCESS ? (uint16_t)(dm > 0xFFFF ? 0xFFFF : dm) : 0;

    portENTER_CRITICAL(&s_mux);
    resp_t *r = &s_r[idx];
    if (r->used && r->node == node) {
        r->any     = true;
        r->last_st = st;
        r->last_ms = now;
        r->bits    = (uint16_t)((r->bits << 1) | (st == FTM_STATUS_SUCCESS));
        if (r->nbits < 16) r->nbits++;
        if (st == FTM_STATUS_SUCCESS) {
            r->ok++;
            r->seen_ms = now;
            r->dist_dm = d16;
            r->rssi    = rssi;
            r->hist[r->hhead] = d16;
            r->hhead = (uint8_t)((r->hhead + 1) % RT_FTM_AVG);
            if (r->nhist < RT_FTM_AVG) r->nhist++;
        } else {
            r->fail++;
        }
    }
    portEXIT_CRITICAL(&s_mux);

    rt_log_ftm(node, st, d16, rssi);
}

// Mean RSSI of the FTM frames, from the report the driver kept for us. Asking for it is also
// what frees it - use_get_report_api is set - so it is asked for after every session that has one.
static int8_t report_rssi(uint8_t n)
{
    if (n == 0) {
        return -128;
    }
    wifi_ftm_report_entry_t *e = calloc(n, sizeof(*e));
    if (e == NULL) {
        return -128;
    }
    int8_t out = -128;
    if (esp_wifi_ftm_get_report(e, n) == ESP_OK) {
        int sum = 0, k = 0;
        for (int i = 0; i < n; i++) {
            if (e[i].rssi != 0) {
                sum += e[i].rssi;
                k++;
            }
        }
        if (k) {
            out = (int8_t)(sum / k);
        }
    }
    free(e);
    return out;
}

static void session(int idx)
{
    wifi_ftm_initiator_cfg_t cfg = { 0 };
    uint32_t node;
    portENTER_CRITICAL(&s_mux);
    memcpy(cfg.resp_mac, s_r[idx].mac, 6);
    cfg.channel = s_r[idx].chan;
    node        = s_r[idx].node;
    portEXIT_CRITICAL(&s_mux);
    cfg.frm_count          = FRM_COUNT;
    cfg.burst_period       = BURST_PERIOD;
    cfg.use_get_report_api = true;

    xSemaphoreTake(s_ftm_done, 0);   // a report left over from a session that timed out
    const esp_err_t err = esp_wifi_ftm_initiate_session(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%06lX: initiate_session -> %s", (unsigned long)node, esp_err_to_name(err));
        record(idx, node, RT_FTM_ST_NOSTART, 0, -128);
        return;
    }

    // Wait for the report for this responder. One for anything else is a straggler from a
    // session that already timed out, and is thrown away.
    const uint32_t deadline = rt_ms() + SESSION_WAIT_MS;
    for (;;) {
        const int32_t left = (int32_t)(deadline - rt_ms());
        if (left <= 0 || xSemaphoreTake(s_ftm_done, pdMS_TO_TICKS(left)) != pdTRUE) {
            esp_wifi_ftm_end_session();
            record(idx, node, RT_FTM_ST_TIMEOUT, 0, -128);
            return;
        }
        bool same = true;
        for (int i = 0; i < 6; i++) {
            same = same && s_rep_mac[i] == cfg.resp_mac[i];
        }
        if (same) {
            break;
        }
    }
    const uint8_t  st   = s_rep_st;
    const uint32_t dist = s_rep_dist_cm;
    const int8_t   rssi = report_rssi(s_rep_n);
    record(idx, node, st, dist, st == FTM_STATUS_SUCCESS ? rssi : -128);
}

static void ftm_task(void *pv)
{
    (void)pv;
    rt_sleeper_t *sl = rt_sleeper_new("ftm");
    uint32_t next_scan = 0;
    bool     was_on = false;
    int      rr = 0;

    for (;;) {
        // Off whenever the switch is, and whenever the driver is not up to do it with - which is
        // briefly true across every Wi-Fi restart.
        if (!(g_tests & RT_TEST_FTM) || !rt_wifi_active()) {
            if (was_on) {
                ESP_LOGI(TAG, "initiator stopped");
                was_on = false;
            }
            vTaskDelay(pdMS_TO_TICKS(IDLE_MS));
            continue;
        }
        const uint32_t now = rt_ms();
        if (!was_on) {
            ESP_LOGI(TAG, "initiator on: scanning for responders");
            was_on    = true;
            next_scan = now;   // look straight away rather than trying last time's list first
        }

        forget_stale(now);
        if ((int32_t)(now - next_scan) >= 0) {
            scan();
            next_scan = rt_ms() + (known() ? SCAN_MS : SCAN_EMPTY_MS);
        }

        int pick = -1;
        for (int k = 0; k < RT_MAX_PEERS; k++) {
            const int i = (rr + k) % RT_MAX_PEERS;
            if (s_r[i].used) {
                pick = i;
                break;
            }
        }
        if (pick >= 0) {
            rr = pick + 1;
            session(pick);
        }
        rt_sleep_rand(sl, GAP_MIN_MS, GAP_MAX_MS);
    }
}

void rt_ftm_start(void)
{
    s_scan_done = xSemaphoreCreateBinary();
    s_ftm_done  = xSemaphoreCreateBinary();
    if (s_scan_done == NULL || s_ftm_done == NULL) {
        ESP_LOGE(TAG, "no memory for semaphores - FTM initiator disabled");
        return;
    }
    RT_TRY(TAG, esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE,
                                                    on_wifi_event, NULL, NULL));
    RT_TRY(TAG, esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_FTM_REPORT,
                                                    on_wifi_event, NULL, NULL));
    xTaskCreate(ftm_task, "ftm", 4096, NULL, 3, NULL);
}

void rt_ftm_reset(void)
{
    // Results only. Which boards are out there, and how to reach them, is not a measurement.
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        resp_t *r = &s_r[i];
        r->any = false;
        r->last_st = 0;
        r->dist_dm = 0;
        r->nhist = r->hhead = 0;
        r->rssi = -128;
        r->bits = 0;
        r->nbits = 0;
        r->ok = r->fail = 0;
    }
    portEXIT_CRITICAL(&s_mux);
}

static uint16_t avg_dm(const resp_t *r)
{
    uint32_t s = 0;
    for (int i = 0; i < r->nhist; i++) {
        s += r->hist[i];
    }
    return r->nhist ? (uint16_t)(s / r->nhist) : 0;
}

static int pdr(const resp_t *r)
{
    if (!r->nbits) {
        return -1;
    }
    const uint16_t m = (uint16_t)(r->nbits >= 16 ? 0xFFFF : (1u << r->nbits) - 1);
    return __builtin_popcount(r->bits & m) * 100 / r->nbits;
}

static int put16(uint8_t *b, int n, uint32_t v)
{
    if (v > 0xFFFF) v = 0xFFFF;
    b[n] = (uint8_t)v;
    b[n + 1] = (uint8_t)(v >> 8);
    return n + 2;
}

static int put32(uint8_t *b, int n, uint32_t v)
{
    for (int i = 0; i < 4; i++) {
        b[n + i] = (uint8_t)(v >> (8 * i));
    }
    return n + 4;
}

int rt_ftm_rows(uint8_t *out, int cap, uint8_t gen)
{
    if (cap < RT_RPT_HDR + RT_RPT_FTM_ROW) {
        return 0;
    }
    resp_t snap[RT_MAX_PEERS];
    portENTER_CRITICAL(&s_mux);
    memcpy(snap, s_r, sizeof(snap));
    portEXIT_CRITICAL(&s_mux);

    const uint32_t now = rt_ms();
    int n = RT_RPT_HDR, rows = 0;
    for (int i = 0; i < RT_MAX_PEERS && n + RT_RPT_FTM_ROW <= cap; i++) {
        const resp_t *r = &snap[i];
        if (!r->used || !r->any) {
            continue;
        }
        out[n++] = (uint8_t)r->node;
        out[n++] = (uint8_t)(r->node >> 8);
        out[n++] = (uint8_t)(r->node >> 16);
        out[n++] = r->last_st;
        n = put16(out, n, r->dist_dm);
        n = put16(out, n, avg_dm(r));
        out[n++] = (uint8_t)(r->ok ? r->rssi : -128);
        out[n++] = (uint8_t)(int8_t)pdr(r);
        n = put32(out, n, r->ok);
        n = put32(out, n, r->fail);
        const int32_t age = (int32_t)(now - r->last_ms);
        n = put16(out, n, age > 0 ? ((uint32_t)age + 50) / 100 : 0);
        rows++;
    }
    if (!rows) {
        return 0;
    }
    out[0] = RT_RPT_TYPE_FTM;
    out[1] = gen;
    out[2] = (uint8_t)rows;
    out[3] = 0;
    return n;
}

void rt_ftm_report(void)
{
    if (!(g_tests & RT_TEST_FTM)) {
        // Results from before the switch went off stay in the table, like every other link's.
        bool any = false;
        for (int i = 0; i < RT_MAX_PEERS; i++) any = any || (s_r[i].used && s_r[i].any);
        if (!any) {
            return;
        }
    }
    resp_t snap[RT_MAX_PEERS];
    portENTER_CRITICAL(&s_mux);
    memcpy(snap, s_r, sizeof(snap));
    portEXIT_CRITICAL(&s_mux);

    const uint32_t now = rt_ms();
    int shown = 0;
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        const resp_t *r = &snap[i];
        if (!r->used) {
            continue;
        }
        shown++;
        if (!r->any) {
            printf("  ftm %06lX  found, no session yet\n", (unsigned long)r->node);
            continue;
        }
        const uint16_t a = avg_dm(r);
        printf("  ftm %06lX  %u.%um (avg %u.%u, rssi %d)  ok %lu fail %lu  %d%% of last %u"
               "  last %s %lums ago\n",
               (unsigned long)r->node, r->dist_dm / 10, r->dist_dm % 10, a / 10, a % 10,
               r->ok ? r->rssi : 0, (unsigned long)r->ok, (unsigned long)r->fail, pdr(r),
               r->nbits, st_name(r->last_st), (unsigned long)(now - r->last_ms));
    }
    if (!shown && (g_tests & RT_TEST_FTM)) {
        if (!rt_wifi_active()) {
            printf("  ftm: on, waiting for the Wi-Fi driver\n");
        } else if (!s_scanned) {
            printf("  ftm: on, not scanned yet\n");
        } else {
            printf("  ftm: no responders - last scan %lus ago heard %u APs, %u of them ours\n",
                   (unsigned long)((now - s_scan_ms) / 1000), s_scan_aps, s_scan_ours);
        }
    }
}

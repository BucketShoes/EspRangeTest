// The Wi-Fi scan, and the FTM initiator (802.11mc fine timing measurement) that uses it.
//
// One task owns the scan, which serves two tests (RT_TEST_* in rt.h):
//
//   AP_SCAN  the scan is the measurement. Each of our boards' APs is a link, heard or missed in
//            each scan, at some RSSI - reported exactly like a packet link, as AP rows.
//   FTM      the scan finds targets: our boards whose AP advertises the FTM responder bit, which
//            is their FTM_RESP test. Then range to them one at a time, round robin, with a pause
//            between sessions.
//
// The scan is passive, on our own channel only: listen for beacons for one beacon interval and
// a bit. An active scan would send probe requests and have every AP in range answer them - antenna
// time on every board around, for a question beacons already answer - and for AP_SCAN it would
// measure the probe exchange rather than whether the AP itself reaches.
//
// FTM is not on a fixed rate like the measurement packets. A session is a burst exchange lasting
// a good fraction of a second, only one can run at a time, and what it answers - how far away -
// changes at walking or flying pace. So sessions follow one another with a randomised gap, which
// is what leaves room on the antenna for the control link and whatever other test is on.
//
// A session that fails is a result, not a retry: it is counted and logged exactly like a
// success, because "tried, and nothing came back" is the range measurement.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "rt.h"

static const char *TAG = "ftm";

// Scanning. With AP_SCAN on, a scan every AP_SCAN_MS (randomised +/-50%, like the packet
// periods); otherwise only as FTM needs targets - every SCAN_MS once one is known, every
// SCAN_EMPTY_MS while none is. SCAN_DWELL_MS must cover one beacon interval (100 TU, 102ms, by
// default) or a scan can miss an AP that is plainly there.
#define AP_SCAN_MS       2000
#define SCAN_MS          30000
#define SCAN_EMPTY_MS    5000
#define SCAN_DWELL_MS    120
#define SCAN_WAIT_MS     2000     // backstop on the scan-done event
#define SCAN_MAX_APS     20

// An FTM target is forgotten FORGET_MS after it was last seen in a scan or last answered, so a
// board that is switched off stops being tried - but only after a couple of minutes, because at
// the edge of range a beacon is as easily lost as anything else, and dropping a board the first
// time one scan misses it would stop measuring it exactly where it gets interesting.
#define FORGET_MS        120000

// Sessions. FRM_COUNT and BURST_PERIOD are what the initiator asks for (see
// wifi_ftm_initiator_cfg_t: 16/24/32/64 frames, bursts in 100ms units) - more frames average
// more noise away, and cost more airtime, on both boards. GAP is the pause after each session,
// randomised so it does not settle into step with anything periodic.
#define FRM_COUNT        16
#define BURST_PERIOD     2        // 200ms between bursts
#define SESSION_WAIT_MS  3000     // backstop on the report event
#define GAP_MIN_MS       1000
#define GAP_MAX_MS       2000
#define IDLE_MS          500      // how often to look again while nothing is on

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
    int16_t  dist_dm;        // last success; signed - see record()
    int16_t  hist[RT_FTM_AVG];
    uint8_t  nhist, hhead;
    int8_t   rssi;           // last success
    uint16_t bits;           // last 16 sessions, bit0 newest: 1 = success
    uint8_t  nbits;
    uint32_t ok, fail;
    uint32_t last_ms;
} resp_t;

// Another board's AP as a link: heard or missed in each AP_SCAN scan.
typedef struct {
    bool     used;
    uint32_t node;
    uint32_t rx, missed;
    uint32_t last_seq;       // this board's scan number when last heard
    int8_t   rssi_last, rssi_min, rssi_max;
    int32_t  rssi_sum, rssi_n;
    uint16_t bits;           // last 16 scans, bit0 newest: 1 = heard
    uint8_t  nbits;
    uint32_t last_ms;
} aplink_t;

static portMUX_TYPE      s_mux = portMUX_INITIALIZER_UNLOCKED;
static resp_t            s_r[RT_MAX_PEERS];
static aplink_t          s_ap[RT_MAX_PEERS];
static SemaphoreHandle_t s_scan_done, s_ftm_done;

// The last report, as the event handler copied it. Read by the task once s_ftm_done is given.
static volatile uint8_t  s_rep_mac[6];
static volatile uint8_t  s_rep_st;
static volatile int32_t  s_rep_dist_cm;
static volatile uint8_t  s_rep_n;

// What the last scan found, for the report: "no results" has to be able to say whether that was
// "heard nothing at all", "heard APs, none ours" or "ours, but none answering FTM".
static uint32_t s_scan_ms;
static uint16_t s_scan_aps, s_scan_ours, s_scan_ftm;
static bool     s_scanned;
static uint32_t s_scan_seq;   // AP_SCAN scans, the seq of the AP links

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
        // Declared unsigned, but it is a difference of timestamps less a calibration: close up
        // and uncalibrated it goes below zero, and read unsigned that is 40 million km.
        s_rep_dist_cm = (int32_t)r->dist_est;
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

// An FTM target seen in a scan: refreshed if known, else added - over the stalest if full.
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

// One AP_SCAN scan's worth of AP links: the ones heard get a reception, every link already
// known that was not heard gets a miss. heard[] is node ids, rssi[] alongside.
static void ap_links(const uint32_t *heard, const int8_t *rssi, int n, uint32_t seq, uint32_t now)
{
    uint32_t log_node[RT_MAX_PEERS];
    uint32_t log_gap[RT_MAX_PEERS];
    int8_t   log_rssi[RT_MAX_PEERS];
    int      nlog = 0;

    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        if (!s_ap[i].used) continue;
        bool got = false;
        for (int k = 0; k < n; k++) got = got || heard[k] == s_ap[i].node;
        if (!got) {
            s_ap[i].missed++;
            s_ap[i].bits = (uint16_t)(s_ap[i].bits << 1);
            if (s_ap[i].nbits < 16) s_ap[i].nbits++;
        }
    }
    for (int k = 0; k < n; k++) {
        aplink_t *l = NULL;
        for (int i = 0; i < RT_MAX_PEERS && !l; i++) {
            if (s_ap[i].used && s_ap[i].node == heard[k]) l = &s_ap[i];
        }
        for (int i = 0; i < RT_MAX_PEERS && !l; i++) {
            if (!s_ap[i].used) {
                l = &s_ap[i];
                memset(l, 0, sizeof(*l));
                l->used = true;
                l->node = heard[k];
                l->rssi_min = 127;
                l->rssi_max = -128;
                l->last_seq = seq - 1;   // first sighting: nothing before it counts as missed
            }
        }
        if (!l) continue;                // table full; the board is simply not tracked
        const uint32_t gap = seq - l->last_seq;
        l->rx++;
        l->last_seq  = seq;
        l->rssi_last = rssi[k];
        l->rssi_sum += rssi[k];
        l->rssi_n++;
        if (rssi[k] < l->rssi_min) l->rssi_min = rssi[k];
        if (rssi[k] > l->rssi_max) l->rssi_max = rssi[k];
        l->bits = (uint16_t)((l->bits << 1) | 1);
        if (l->nbits < 16) l->nbits++;
        l->last_ms = now;
        if (nlog < RT_MAX_PEERS) {
            log_node[nlog] = l->node;
            log_gap[nlog]  = gap;
            log_rssi[nlog] = rssi[k];
            nlog++;
        }
    }
    portEXIT_CRITICAL(&s_mux);

    // Into the log on the same rule as packets: only while this board knows where it is.
    if (rt_gnss_had_fix()) {
        for (int k = 0; k < nlog; k++) {
            rt_log_rx(log_node[k], RT_RPT_ROW_AP, -128, seq, log_gap[k], log_rssi[k], 0, NULL);
        }
    }
}

static void scan(bool measure)
{
    uint8_t prim = 1;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&prim, &sec);

    // Our own channel only, passive. Every board sits on the same channel, hopping would take
    // this board's AP and ESP-NOW off it for the length of the scan, and passive means this
    // board transmits nothing and no AP is asked to answer - see the top of the file.
    wifi_scan_config_t cfg = { 0 };
    cfg.channel             = prim;
    cfg.show_hidden         = false;
    cfg.scan_type           = WIFI_SCAN_TYPE_PASSIVE;
    cfg.scan_time.passive   = SCAN_DWELL_MS;

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
    uint32_t heard[RT_MAX_PEERS];
    int8_t   hrssi[RT_MAX_PEERS];
    int      nh = 0;
    uint16_t ours = 0, ftm = 0;
    for (int i = 0; i < n; i++) {
        const uint32_t node = node_of(ap[i].ssid);
        // Ours and not ourselves - nothing else is looked at.
        if (node == 0 || node == rt_node_id()) {
            continue;
        }
        ours++;
        if (nh < RT_MAX_PEERS) {
            heard[nh] = node;
            hrssi[nh] = ap[i].rssi;
            nh++;
        }
        // An FTM target only if it says it answers - its FTM_RESP test. Ranging a board that
        // does not would be a session of requests nobody answers, on both boards' airtime.
        if (ap[i].ftm_responder) {
            ftm++;
            note_responder(node, ap[i].bssid, ap[i].primary, now);
        }
    }
    free(ap);

    if (measure) {
        ap_links(heard, hrssi, nh, ++s_scan_seq, now);
    }

    s_scan_ms   = now;
    s_scan_aps  = n;
    s_scan_ours = ours;
    s_scan_ftm  = ftm;
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

// dist_cm is signed: see on_wifi_event(). Decimetres, clamped to what an i16 holds.
static void record(int idx, uint32_t node, uint8_t st, int32_t dist_cm, int8_t rssi)
{
    const uint32_t now = rt_ms();
    const int32_t  dm  = (dist_cm >= 0 ? dist_cm + 5 : dist_cm - 5) / 10;
    const int16_t  d16 = st != FTM_STATUS_SUCCESS ? 0
                       : (int16_t)(dm > INT16_MAX ? INT16_MAX : dm < -INT16_MAX ? -INT16_MAX : dm);

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
    const int32_t  dist = s_rep_dist_cm;
    const int8_t   rssi = report_rssi(s_rep_n);
    record(idx, node, st, dist, st == FTM_STATUS_SUCCESS ? rssi : -128);
}

static uint32_t rand_ms(uint32_t lo, uint32_t hi)
{
    return lo + esp_random() % (hi - lo + 1);
}

static void scan_task(void *pv)
{
    (void)pv;
    rt_sleeper_t *sl = rt_sleeper_new("ftm");
    uint32_t next_scan = 0;
    bool     was_ftm = false, was_aps = false;
    int      rr = 0;

    for (;;) {
        const bool ftm = (g_tests & RT_TEST_FTM) != 0;
        const bool aps = (g_tests & RT_TEST_AP_SCAN) != 0;

        // Off whenever both switches are, and whenever the driver is not up to do it with -
        // which is briefly true across every Wi-Fi restart.
        if ((!ftm && !aps) || !rt_wifi_active()) {
            if (was_ftm || was_aps) {
                ESP_LOGI(TAG, "scanning stopped");
                was_ftm = was_aps = false;
            }
            vTaskDelay(pdMS_TO_TICKS(IDLE_MS));
            continue;
        }
        const uint32_t now = rt_ms();
        // Either switch coming on looks straight away rather than waiting out the old schedule.
        if ((ftm && !was_ftm) || (aps && !was_aps)) {
            ESP_LOGI(TAG, "scanning for %s%s%s", aps ? "AP_SCAN" : "", aps && ftm ? " and " : "",
                     ftm ? "FTM targets" : "");
            next_scan = now;
        }
        was_ftm = ftm;
        was_aps = aps;

        forget_stale(now);
        if ((int32_t)(now - next_scan) >= 0) {
            scan(aps);
            next_scan = rt_ms() + (aps     ? rand_ms(AP_SCAN_MS / 2, AP_SCAN_MS * 3 / 2)
                                   : known() ? SCAN_MS : SCAN_EMPTY_MS);
        }

        int pick = -1;
        for (int k = 0; ftm && k < RT_MAX_PEERS; k++) {
            const int i = (rr + k) % RT_MAX_PEERS;
            if (s_r[i].used) {
                pick = i;
                break;
            }
        }
        if (pick >= 0) {
            rr = pick + 1;
            session(pick);
            rt_sleep_rand(sl, GAP_MIN_MS, GAP_MAX_MS);
        } else {
            // Nothing to range: sleep to the next scan, but look at the switches at least every
            // IDLE_MS.
            int32_t wait = (int32_t)(next_scan - rt_ms());
            if (wait > IDLE_MS) wait = IDLE_MS;
            vTaskDelay(pdMS_TO_TICKS(wait > 10 ? wait : 10));
        }
    }
}

void rt_ftm_start(void)
{
    s_scan_done = xSemaphoreCreateBinary();
    s_ftm_done  = xSemaphoreCreateBinary();
    if (s_scan_done == NULL || s_ftm_done == NULL) {
        ESP_LOGE(TAG, "no memory for semaphores - Wi-Fi scan and FTM disabled");
        return;
    }
    RT_TRY(TAG, esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE,
                                                    on_wifi_event, NULL, NULL));
    RT_TRY(TAG, esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_FTM_REPORT,
                                                    on_wifi_event, NULL, NULL));
    xTaskCreate(scan_task, "ftm", 4096, NULL, 3, NULL);
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
    memset(s_ap, 0, sizeof(s_ap));
    portEXIT_CRITICAL(&s_mux);
}

void rt_scan_status(rt_scan_st_t *out)
{
    const uint32_t age = (rt_ms() - s_scan_ms + 50) / 100;
    out->aps      = (uint8_t)(s_scan_aps > 255 ? 255 : s_scan_aps);
    out->ours     = (uint8_t)(s_scan_ours > 255 ? 255 : s_scan_ours);
    out->ours_ftm = (uint8_t)(s_scan_ftm > 255 ? 255 : s_scan_ftm);
    out->known    = (uint8_t)known();
    out->age_ds   = !s_scanned ? 0xFFFF : age > 0xFFFE ? 0xFFFE : (uint16_t)age;
}

static int16_t avg_dm(const resp_t *r)
{
    int32_t s = 0;
    for (int i = 0; i < r->nhist; i++) {
        s += r->hist[i];
    }
    return r->nhist ? (int16_t)(s / r->nhist) : 0;
}

static int pct16(uint16_t bits, uint8_t nbits)
{
    if (!nbits) {
        return -1;
    }
    const uint16_t m = (uint16_t)(nbits >= 16 ? 0xFFFF : (1u << nbits) - 1);
    return __builtin_popcount(bits & m) * 100 / nbits;
}

static int put16(uint8_t *b, int n, uint16_t v)
{
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

static uint16_t age_ds(uint32_t now, uint32_t then)
{
    const int32_t a = (int32_t)(now - then);
    const uint32_t ds = a > 0 ? ((uint32_t)a + 50) / 100 : 0;
    return ds > 0xFFFF ? 0xFFFF : (uint16_t)ds;
}

// Which rows exist: AP links while AP_SCAN is on (their results are only worth the airtime while
// they are the test), then FTM targets with results, whatever the switch - a range from before
// it went off stays in the table, like every other link.
static bool ap_row(const aplink_t *l)  { return l->used && (g_tests & RT_TEST_AP_SCAN); }
static bool ftm_row(const resp_t *r)   { return r->used && r->any; }

int rt_wifi_rows(void)
{
    int n = 0;
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < RT_MAX_PEERS; i++) n += ap_row(&s_ap[i]) + ftm_row(&s_r[i]);
    portEXIT_CRITICAL(&s_mux);
    return n;
}

int rt_wifi_row(int want, uint8_t *out, int room)
{
    aplink_t l;
    resp_t   r;
    int      kind = 0, seen = 0;

    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < RT_MAX_PEERS && !kind; i++) {
        if (ap_row(&s_ap[i]) && seen++ == want) { l = s_ap[i]; kind = RT_RPT_ROW_AP; }
    }
    for (int i = 0; i < RT_MAX_PEERS && !kind; i++) {
        if (ftm_row(&s_r[i]) && seen++ == want) { r = s_r[i]; kind = RT_RPT_ROW_FTM; }
    }
    portEXIT_CRITICAL(&s_mux);

    const uint32_t now = rt_ms();
    int n = 0;
    if (kind == RT_RPT_ROW_AP) {
        // The link row's own layout - see the report in rt.h.
        if (room < RT_RPT_ROW) return 0;
        const uint32_t tot = l.rx + l.missed;
        out[n++] = (uint8_t)l.node;
        out[n++] = (uint8_t)(l.node >> 8);
        out[n++] = (uint8_t)(l.node >> 16);
        out[n++] = RT_RPT_ROW_AP;
        out[n++] = (uint8_t)l.rssi_last;
        out[n++] = (uint8_t)(l.rssi_n ? l.rssi_sum / l.rssi_n : 0);
        out[n++] = (uint8_t)l.rssi_min;
        out[n++] = (uint8_t)l.rssi_max;
        out[n++] = (uint8_t)(int8_t)pct16(l.bits, l.nbits);
        out[n++] = (uint8_t)(int8_t)(tot ? (int)(l.rx * 100 / tot) : -1);
        n = put32(out, n, l.rx);
        n = put32(out, n, l.missed);
        n = put16(out, n, age_ds(now, l.last_ms));
        out[n++] = (uint8_t)-128;   // no noise floor from a scan
        out[n++] = 0;               // no lqi
        out[n++] = (uint8_t)-128;   // the AP's power is not in its beacon
        return n;
    }
    if (kind == RT_RPT_ROW_FTM) {
        if (room < RT_RPT_FTM_ROW) return 0;
        out[n++] = (uint8_t)r.node;
        out[n++] = (uint8_t)(r.node >> 8);
        out[n++] = (uint8_t)(r.node >> 16);
        out[n++] = RT_RPT_ROW_FTM;
        out[n++] = r.last_st;
        n = put16(out, n, (uint16_t)r.dist_dm);
        n = put16(out, n, (uint16_t)avg_dm(&r));
        out[n++] = (uint8_t)(r.ok ? r.rssi : -128);
        out[n++] = (uint8_t)(int8_t)pct16(r.bits, r.nbits);
        n = put32(out, n, r.ok);
        n = put32(out, n, r.fail);
        n = put16(out, n, age_ds(now, r.last_ms));
        return n;
    }
    return -1;
}

// Decimetres as metres with one place, sign included, without the float formatter.
static void print_dm(int16_t dm)
{
    const int a = dm < 0 ? -dm : dm;
    printf("%s%d.%d", dm < 0 ? "-" : "", a / 10, a % 10);
}

void rt_ftm_report(void)
{
    resp_t   snap[RT_MAX_PEERS];
    aplink_t aps[RT_MAX_PEERS];
    portENTER_CRITICAL(&s_mux);
    memcpy(snap, s_r, sizeof(snap));
    memcpy(aps, s_ap, sizeof(aps));
    portEXIT_CRITICAL(&s_mux);

    const bool ftm = (g_tests & RT_TEST_FTM) != 0;
    const bool apo = (g_tests & RT_TEST_AP_SCAN) != 0;
    const uint32_t now = rt_ms();

    // Said every report while either scanning test is on: "nothing found" and "found, nothing
    // answered" are different faults and neither shows in the rows.
    if (ftm || apo) {
        if (!rt_wifi_active()) {
            printf("  scan: waiting for the Wi-Fi driver\n");
        } else if (!s_scanned) {
            printf("  scan: not scanned yet\n");
        } else {
            printf("  scan: %lus ago heard %u APs - %u ours, %u of them answering FTM; "
                   "%d FTM targets\n", (unsigned long)((now - s_scan_ms) / 1000),
                   s_scan_aps, s_scan_ours, s_scan_ftm, known());
        }
    }
    for (int i = 0; apo && i < RT_MAX_PEERS; i++) {
        const aplink_t *l = &aps[i];
        if (!l->used) continue;
        printf("  %06lX ap       rssi %4d (avg %4d, %d..%d)  %3d%% of last %u scans"
               "  rx %lu miss %lu  %lums ago\n", (unsigned long)l->node, l->rssi_last,
               l->rssi_n ? (int)(l->rssi_sum / l->rssi_n) : 0, l->rssi_min, l->rssi_max,
               pct16(l->bits, l->nbits), l->nbits, (unsigned long)l->rx,
               (unsigned long)l->missed, (unsigned long)(now - l->last_ms));
    }
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        const resp_t *r = &snap[i];
        if (!r->used || (!ftm && !r->any)) {
            continue;
        }
        if (!r->any) {
            printf("  ftm %06lX  found, no session yet\n", (unsigned long)r->node);
            continue;
        }
        printf("  ftm %06lX  ", (unsigned long)r->node);
        print_dm(r->dist_dm);
        printf("m (avg ");
        print_dm(avg_dm(r));
        printf(", rssi %d)  ok %lu fail %lu  %d%% of last %u  last %s %lums ago\n",
               r->ok ? r->rssi : 0, (unsigned long)r->ok, (unsigned long)r->fail,
               pct16(r->bits, r->nbits), r->nbits, st_name(r->last_st),
               (unsigned long)(now - r->last_ms));
    }
}

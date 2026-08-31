// The results table: one row per (peer, channel), printed over serial.

#include <stdio.h>
#include <string.h>

#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"

#include "rt.h"

const char *rt_chan_name[CH_COUNT] = { "espnow", "ble_adv", "154" };

volatile int g_lc = 0;

const char *rt_pwr_name[RT_PWR_COUNT] = { "min", "low", "mid", "max" };

// dBm per channel per level. Ranges differ by radio: Wi-Fi is 2..20 (and quantises to its own
// ladder, see esp_wifi.h), 802.15.4 is -15..20 in 3dB steps, BLE is whatever the controller
// offers near the request. All three top out at 20 - including BLE, so the open question of
// whether this chip is honest above 9dBm is something a button press can answer.
static const int8_t s_pwr_dbm[CH_COUNT][RT_PWR_COUNT] = {
    [CH_ESPNOW]  = {   2, 5,  11, 20 },
    [CH_BLE_ADV] = { -12, -6,  3, 20 },
    [CH_154]     = { -15, -6,  5, 20 },
};

volatile int g_pwr[CH_COUNT];  // all zero: every channel boots at "min"

int8_t rt_power_dbm(int chan)
{
    if (chan < 0 || chan >= CH_COUNT) {
        return 0;
    }
    const int lvl = g_pwr[chan];
    return s_pwr_dbm[chan][(lvl >= 0 && lvl < RT_PWR_COUNT) ? lvl : 0];
}

void rt_set_power(int chan, int level)
{
    if (chan < 0 || chan >= CH_COUNT || level < 0 || level >= RT_PWR_COUNT) {
        return;
    }
    g_pwr[chan] = level;

    // Only this channel's history is invalidated - the other two were not touched.
    rt_stats_reset_chan(chan);

    switch (chan) {
    case CH_ESPNOW:  rt_wifi_apply_power(); break;
    case CH_BLE_ADV: rt_ble_apply_power();  break;
    case CH_154:     rt_154_apply_power();  break;
    default: break;
    }

    printf("\n>>> tx power %s = %s (%d dBm)\n", rt_chan_name[chan], rt_pwr_name[level],
           rt_power_dbm(chan));
}

typedef struct {
    bool     seen;
    uint32_t rx, missed;      // totals since boot
    uint32_t wrx, wmissed;    // since the last report
    uint32_t last_seq;
    int32_t  rssi_sum;
    int32_t  rssi_n;
    int8_t   rssi_last, rssi_min, rssi_max;
    uint8_t  lqi;
    uint32_t last_ms;
} rt_link;

static struct {
    bool    used;
    uint8_t node;
    rt_link ch[CH_COUNT];
} s_peers[RT_MAX_PEERS];

static uint32_t s_tx_seq[CH_COUNT];
static uint32_t s_tx_count[CH_COUNT];
static uint32_t s_tx_ok[CH_COUNT];
static uint32_t s_tx_fail[CH_COUNT];
static uint8_t  s_node_id;

uint32_t rt_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

uint32_t rt_jitter_ms(uint32_t ms)
{
    const uint32_t span = ms / 10;  // +/-5% around ms
    if (span == 0) {
        return ms;
    }
    return ms - span / 2 + (esp_random() % (span + 1));
}

uint8_t rt_node_id(void)
{
    if (s_node_id == 0) {
        uint8_t mac[6] = { 0 };
        esp_read_mac(mac, ESP_MAC_BASE);
        s_node_id = mac[5] ? mac[5] : 1;
    }
    return s_node_id;
}

bool rt_tx_enabled(int chan)
{
    if (g_lc == 0) {
        return true;
    }
    if (g_lc == LC_WIFI_UI) {
        return chan == CH_ESPNOW;
    }
    return g_lc == chan + 1;
}

// LC_WIFI_UI isn't "channel (lc-1)" like the others, so callers that want to name the current
// mode (the report header, the >>> line) go through this instead of indexing rt_chan_name
// directly - that indexing is only valid for lc in 1..CH_COUNT.
static const char *lc_name(int lc)
{
    if (lc == 0) {
        return "off (all radios)";
    }
    if (lc == LC_WIFI_UI) {
        return "wifi+phone (BLE off, LR forced off)";
    }
    return rt_chan_name[lc - 1];
}

void rt_stats_reset_chan(int chan)
{
    if (chan < 0 || chan >= CH_COUNT) {
        return;
    }
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        memset(&s_peers[i].ch[chan], 0, sizeof(s_peers[i].ch[chan]));
    }
    s_tx_seq[chan]   = 0;
    s_tx_count[chan] = 0;
    s_tx_ok[chan]    = 0;
    s_tx_fail[chan]  = 0;
}

void rt_stats_reset(void)
{
    // Sequence numbers restart, so wipe what we have rather than let the restart read as a
    // huge run of losses.
    memset(s_peers, 0, sizeof(s_peers));
    memset(s_tx_seq, 0, sizeof(s_tx_seq));
    memset(s_tx_count, 0, sizeof(s_tx_count));
    memset(s_tx_ok, 0, sizeof(s_tx_ok));
    memset(s_tx_fail, 0, sizeof(s_tx_fail));
}

void rt_set_lc(int lc)
{
    if (lc < 0 || lc >= LC_COUNT) {
        return;
    }
    g_lc = lc;
    rt_stats_reset();
    // Set g_lc first: rt_apply_lc_radios() and everything it reaches decide what to do by
    // reading g_lc, not the argument.
    rt_apply_lc_radios(lc);

    printf("\n>>> low contention = %s\n", lc_name(lc));
}

void rt_fill(rt_pkt_t *p, int chan, int8_t txdbm)
{
    p->magic = RT_MAGIC;
    p->node  = rt_node_id();
    p->txdbm = txdbm;
    p->seq   = s_tx_seq[chan]++;
    s_tx_count[chan]++;
}

void rt_tx_ok(int chan)
{
    if (chan >= 0 && chan < CH_COUNT) {
        s_tx_ok[chan]++;
    }
}

void rt_tx_failed(int chan)
{
    if (chan >= 0 && chan < CH_COUNT) {
        s_tx_fail[chan]++;
    }
}

void rt_rx(const void *data, int len, int chan, int8_t rssi, uint8_t lqi)
{
    if (len < (int)sizeof(rt_pkt_t) || chan < 0 || chan >= CH_COUNT) {
        return;
    }
    rt_pkt_t p;
    memcpy(&p, data, sizeof(p));
    if (p.magic != RT_MAGIC || p.node == rt_node_id()) {
        return;
    }

    int slot = -1;
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        if (s_peers[i].used && s_peers[i].node == p.node) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        for (int i = 0; i < RT_MAX_PEERS; i++) {
            if (!s_peers[i].used) {
                s_peers[i].used = true;
                s_peers[i].node = p.node;
                slot = i;
                break;
            }
        }
    }
    if (slot < 0) {
        return;
    }

    rt_link *l = &s_peers[slot].ch[chan];

    if (l->seen) {
        const uint32_t gap = p.seq - l->last_seq;
        // Sane gaps only; a reboot or a mode change restarts numbering and would otherwise
        // inject a phantom loss run.
        if (gap > 1 && gap < 1000) {
            l->missed  += gap - 1;
            l->wmissed += gap - 1;
        }
    } else {
        l->seen     = true;
        l->rssi_min = 127;
        l->rssi_max = -128;
    }

    l->last_seq  = p.seq;
    l->rx++;
    l->wrx++;
    l->rssi_last = rssi;
    l->rssi_sum += rssi;
    l->rssi_n++;
    if (rssi < l->rssi_min) l->rssi_min = rssi;
    if (rssi > l->rssi_max) l->rssi_max = rssi;
    l->lqi     = lqi;
    l->last_ms = rt_ms();
}

int rt_snapshot_lines(char out[][RT_LINE_MAX], int max)
{
    const uint32_t now = rt_ms();
    int            n   = 0;

    if (n < max) {
        snprintf(out[n++], RT_LINE_MAX, "S,%02X,%lu,%d,%d,%d,%d,%d", rt_node_id(),
                 (unsigned long)(now / 1000), g_lc, g_lr ? 1 : 0,
                 g_pwr[CH_ESPNOW], g_pwr[CH_BLE_ADV], g_pwr[CH_154]);
    }

    for (int i = 0; i < RT_MAX_PEERS && n < max; i++) {
        if (!s_peers[i].used) {
            continue;
        }
        for (int c = 0; c < CH_COUNT && n < max; c++) {
            rt_link *l = &s_peers[i].ch[c];
            if (!l->seen) {
                continue;
            }
            const uint32_t wtot = l->wrx + l->wmissed;
            const int      wpdr = wtot ? (int)((l->wrx * 100) / wtot) : -1;
            const uint32_t tot  = l->rx + l->missed;
            const int      tpdr = tot ? (int)((l->rx * 100) / tot) : -1;
            const int      mean = l->rssi_n ? (int)(l->rssi_sum / l->rssi_n) : 0;

            snprintf(out[n++], RT_LINE_MAX, "R,%02X,%s,%d,%d,%d,%d,%d,%d,%lu,%lu,%lu",
                     s_peers[i].node, rt_chan_name[c], l->rssi_last, mean,
                     l->rssi_min, l->rssi_max, wpdr, tpdr,
                     (unsigned long)l->rx, (unsigned long)l->missed,
                     (unsigned long)(now - l->last_ms));
        }
    }
    return n;
}

void rt_report(void)
{
    const uint32_t now = rt_ms();

    // wifi= is the Wi-Fi *driver*, not the ESP-NOW tx gate on the line below. The two are
    // separate on purpose: a mode that mutes ESP-NOW while leaving the driver up is the exact
    // failure this rig kept measuring, so the report has to be able to show that state.
    printf("\n== node %02X  up %lus  lc=%s  lr=%s  wifi=%s ==\n", rt_node_id(),
           (unsigned long)(now / 1000), lc_name(g_lc), g_lr ? "on" : "off",
           rt_wifi_active() ? "on" : "off");
    printf("  pwr: ");
    for (int c = 0; c < CH_COUNT; c++) {
        printf("%s=%s(%ddBm)  ", rt_chan_name[c], rt_pwr_name[g_pwr[c]], rt_power_dbm(c));
    }
    printf("\n");
    // "queued" is what we asked the radio to send; "ok" and "rejected" are what its own
    // completion callback said happened. ble_adv has no completion callback to report, so it
    // shows a queued count only - absence of ok/rejected there is the API, not a result.
    printf("  tx: ");
    for (int c = 0; c < CH_COUNT; c++) {
        printf("%s=%lu%s", rt_chan_name[c], (unsigned long)s_tx_count[c],
               rt_tx_enabled(c) ? "" : "(off)");
        if (s_tx_ok[c] || s_tx_fail[c]) {
            printf("[%lu ok, %lu rejected]", (unsigned long)s_tx_ok[c],
                   (unsigned long)s_tx_fail[c]);
        }
        printf("  ");
    }
    printf("\n");

    bool any = false;
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        if (!s_peers[i].used) {
            continue;
        }
        for (int c = 0; c < CH_COUNT; c++) {
            rt_link *l = &s_peers[i].ch[c];
            if (!l->seen) {
                continue;
            }
            any = true;

            const uint32_t wtot = l->wrx + l->wmissed;
            const int      wpdr = wtot ? (int)((l->wrx * 100) / wtot) : -1;
            const uint32_t tot  = l->rx + l->missed;
            const int      tpdr = tot ? (int)((l->rx * 100) / tot) : -1;
            const int      mean = l->rssi_n ? (int)(l->rssi_sum / l->rssi_n) : 0;

            printf("  %02X %-8s rssi %4d (avg %4d, %d..%d)  pdr %3d%% now / %3d%% all"
                   "  rx %lu miss %lu  %lums ago",
                   s_peers[i].node, rt_chan_name[c], l->rssi_last, mean,
                   l->rssi_min, l->rssi_max, wpdr, tpdr,
                   (unsigned long)l->rx, (unsigned long)l->missed,
                   (unsigned long)(now - l->last_ms));
            if (c == CH_154) {
                printf(" lqi %u", l->lqi);
            }
            printf("\n");

            l->wrx = l->wmissed = 0;
        }
    }
    if (!any) {
        printf("  (nothing heard yet)\n");
    }
}

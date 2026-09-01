// The results table: one row per (peer, channel), printed over serial.

#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"

#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"

#include "rt.h"

const char *rt_chan_name[CH_COUNT] = { "espnow", "ble_adv", "154" };

volatile int g_lc = 0;

// Each radio's real range, from its own API - see the note in rt.h. Wi-Fi's floor of +2dBm is
// not a choice; it is where esp_wifi_set_max_tx_power's valid range starts.
static const rt_pwr_range_t s_range[CH_COUNT] = {
    // Wi-Fi's ceiling is taken from the PHY cap rather than hardcoded, so asking for a power
    // the PHY will not deliver is impossible. esp_wifi_get_max_tx_power() reports what the
    // driver stored, not what the PHY allows, so without this the read-back could claim 20dBm
    // while the radio transmitted 10 - a lie in exactly the field the packet carries.
    [CH_ESPNOW]  = {   2, CONFIG_ESP_PHY_MAX_WIFI_TX_POWER, 1 },
    [CH_BLE_ADV] = { -15, 20, 3 },
    [CH_154]     = { -15, 20, 3 },
};

// Boot at each radio's minimum, deliberately. See rt.h.
volatile int8_t g_pwr_dbm[CH_COUNT] = {
    [CH_ESPNOW]  =   2,
    [CH_BLE_ADV] = -15,
    [CH_154]     = -15,
};

static int8_t s_pwr_actual[CH_COUNT];

const rt_pwr_range_t *rt_power_range(int chan)
{
    return &s_range[(chan >= 0 && chan < CH_COUNT) ? chan : 0];
}

int8_t rt_power_dbm(int chan)
{
    return (chan >= 0 && chan < CH_COUNT) ? g_pwr_dbm[chan] : 0;
}

int8_t rt_power_actual(int chan)
{
    return (chan >= 0 && chan < CH_COUNT) ? s_pwr_actual[chan] : 0;
}

void rt_power_set_actual(int chan, int8_t dbm)
{
    if (chan >= 0 && chan < CH_COUNT) {
        s_pwr_actual[chan] = dbm;
    }
}

static void apply_one(int chan, int dbm)
{
    const rt_pwr_range_t *r = rt_power_range(chan);
    if (dbm < r->min) dbm = r->min;
    if (dbm > r->max) dbm = r->max;
    g_pwr_dbm[chan] = (int8_t)dbm;

    // Only this channel's history is invalidated - the other two were not touched.
    rt_stats_reset_chan(chan);

    switch (chan) {
    case CH_ESPNOW:  rt_wifi_apply_power(); break;
    case CH_BLE_ADV: rt_ble_apply_power();  break;
    case CH_154:     rt_154_apply_power();  break;
    default: break;
    }
}

// chan == CH_COUNT means all three, each clamped to its own range.
void rt_set_power(int chan, int dbm)
{
    if (chan == CH_COUNT) {
        for (int c = 0; c < CH_COUNT; c++) {
            apply_one(c, dbm);
        }
    } else if (chan >= 0 && chan < CH_COUNT) {
        apply_one(chan, dbm);
    } else {
        return;
    }

    printf("\n>>> tx power:");
    for (int c = 0; c < CH_COUNT; c++) {
        printf("  %s=%ddBm", rt_chan_name[c], rt_power_dbm(c));
    }
    printf("\n");
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
    // Set g_lc first: rt_apply_lc_radios() and everything it reaches decide what to do by
    // reading g_lc, not the argument.
    g_lc = lc;

    // Radios first, table second. Stopping a radio is not instant - esp_wifi_stop() unwinds a
    // driver, and callbacks already queued still land - so wiping the table first left a window
    // where in-flight packets were recorded into the freshly cleared table. That is how an
    // espnow row appeared under a report header saying 154 was isolated.
    rt_apply_lc_radios(lc);
    rt_stats_reset();

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
    // A channel this mode is not measuring does not belong in the table, even if the far end is
    // still transmitting it and even if our own radio has not finished stopping. rt_tx_enabled()
    // is the same set of channels the mode cares about, applied to the receive side: without
    // this, the ordering above only narrows the window rather than closing it.
    if (!rt_tx_enabled(chan)) {
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
        // Power fields are the achieved dBm, not the requested one - the UI should show what
        // the radio is actually doing.
        snprintf(out[n++], RT_LINE_MAX, "S,%02X,%lu,%d,%d,%d,%d,%d", rt_node_id(),
                 (unsigned long)(now / 1000), g_lc, g_lr ? 1 : 0,
                 rt_power_actual(CH_ESPNOW), rt_power_actual(CH_BLE_ADV),
                 rt_power_actual(CH_154));
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
    // Asked-for versus achieved. They differ whenever the radio quantised the request, which
    // is worth seeing rather than hiding behind the number that was typed.
    printf("  pwr: ");
    for (int c = 0; c < CH_COUNT; c++) {
        const int8_t want = rt_power_dbm(c), got = rt_power_actual(c);
        if (want == got) {
            printf("%s=%ddBm  ", rt_chan_name[c], got);
        } else {
            printf("%s=%ddBm(asked %d)  ", rt_chan_name[c], got, want);
        }
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

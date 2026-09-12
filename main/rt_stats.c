// The results table: one row per (peer, channel), printed over serial.

#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"

#include "esp_mac.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_timer.h"

#include "rt.h"

// The table itself prints with printf; this is only for the format-integrity complaint in
// rt_snapshot_chunk(), which has to be a log line so it cannot be mistaken for report output.
static const char *TAG = "stats";

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
    int8_t   noise;      // RT_NOISE_NONE where the radio does not measure one
    int8_t   peer_txdbm; // what the far end said it transmitted at, from the packet itself
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
static uint32_t s_rx_offmode[CH_COUNT];     // heard while this mode said the channel was off
static uint32_t s_rx_offmode_ms[CH_COUNT];  // and when the most recent one was
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
    // s_rx_offmode is deliberately NOT cleared here. It counts a fault, not a measurement, and
    // a mode change is exactly when the fault happens - clearing it on every mode change would
    // erase the evidence at the moment it was collected.
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

void rt_rx(const void *data, int len, int chan, int8_t rssi, uint8_t lqi, int8_t noise)
{
    // Exact length, on every channel. Our packets are always exactly this size, so anything
    // else is somebody else's - one more filter applied before the magic, for free.
    if (len != (int)sizeof(rt_pkt_t) || chan < 0 || chan >= CH_COUNT) {
        return;
    }
    // A reception on a channel this mode is not measuring is a bug caught red-handed, not
    // noise to be swallowed. We asked for that radio to be off; if a packet arrived anyway,
    // either it is not off or the mode is not what the report says - and either way it is
    // spending airtime that something else was promised.
    //
    // So it is counted and reported loudly, but deliberately not entered in the results table:
    // a row for a channel the header says is silent would corrupt the measurement while it
    // explains the fault. The counter is the evidence; the table stays honest.
    if (!rt_tx_enabled(chan)) {
        s_rx_offmode[chan]++;
        s_rx_offmode_ms[chan] = rt_ms();
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
    l->lqi        = lqi;
    l->noise      = noise;
    // The sender puts its own transmit power in every packet, and until now it was parsed and
    // thrown away. Kept, because it turns rssi into path loss: txdbm - rssi is the whole link
    // budget end to end, and unlike rssi it does not change when either side's power changes.
    // That is the number to compare against free-space loss when asking how good an antenna
    // is, and the number to watch when walking.
    l->peer_txdbm = p.txdbm;
    l->last_ms = rt_ms();
}

// ---- packed report -------------------------------------------------------------------------
//
// Hand-serialised, little-endian, one field at a time. Deliberately not a memcpy of a packed
// struct: the other end of this wire is a JavaScript DataView, and the only way to be certain
// the two agree is for both to name every offset explicitly. See the format in rt.h.

static int put_u8(uint8_t *b, int n, uint8_t v)
{
    b[n] = v;
    return n + 1;
}

static int put_u16(uint8_t *b, int n, uint32_t v)
{
    if (v > 0xFFFF) {
        v = 0xFFFF;  // clamp rather than wrap: a stuck-at-max age reads as "old", a wrapped
    }                // one reads as "just arrived", and only one of those is survivable.
    b[n]     = (uint8_t)(v & 0xFF);
    b[n + 1] = (uint8_t)(v >> 8);
    return n + 2;
}

static int put_u32(uint8_t *b, int n, uint32_t v)
{
    b[n]     = (uint8_t)(v & 0xFF);
    b[n + 1] = (uint8_t)((v >> 8) & 0xFF);
    b[n + 2] = (uint8_t)((v >> 16) & 0xFF);
    b[n + 3] = (uint8_t)((v >> 24) & 0xFF);
    return n + 4;
}

// Walk the peer table in the same order twice - once to count, once to emit - so the n_rows in
// the status block cannot disagree with the rows that follow it.
static rt_link *row_at(int want, uint8_t *peer_out, uint8_t *chan_out)
{
    int seen = 0;
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        if (!s_peers[i].used) {
            continue;
        }
        for (int c = 0; c < CH_COUNT; c++) {
            if (!s_peers[i].ch[c].seen) {
                continue;
            }
            if (seen == want) {
                *peer_out = s_peers[i].node;
                *chan_out = (uint8_t)c;
                return &s_peers[i].ch[c];
            }
            seen++;
        }
    }
    return NULL;
}

int rt_snapshot_rows(void)
{
    int n = 0;
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        if (!s_peers[i].used) {
            continue;
        }
        for (int c = 0; c < CH_COUNT; c++) {
            if (s_peers[i].ch[c].seen) {
                n++;
            }
        }
    }
    return n;
}

int rt_snapshot_chunk(uint8_t *out, int cap, uint8_t gen, rt_rpt_state_t *st)
{
    const uint32_t now   = rt_ms();
    const int      total = rt_snapshot_rows();

    if (cap > RT_RPT_CHUNK_MAX) {
        cap = RT_RPT_CHUNK_MAX;
    }
    // A connection that cannot carry the status block plus one row is not worth half a report.
    if (cap < RT_RPT_HDR + RT_RPT_STATUS) {
        return 0;
    }
    if (st->started && st->row_next >= total) {
        return 0;  // done
    }

    int n = RT_RPT_HDR;  // header is filled in last, once we know if this is the final chunk
    const uint8_t type = st->started ? RT_RPT_TYPE_ROWS : RT_RPT_TYPE_STATUS;

    if (!st->started) {
        uint32_t f154 = 0, o154 = 0, coex = 0;
        rt_154_counters(&f154, &o154, &coex);

        const uint8_t state = (uint8_t)((g_lr ? 1 : 0)
                                        | (g_ant_ext ? 2 : 0)
                                        | (rt_wifi_active() ? 4 : 0)
                                        | (g_conn_2m ? 8 : 0)
                                        | ((rt_conn_phy_actual() & 0x03) << 4));

        n = put_u8(out, n, RT_RPT_VER);
        n = put_u8(out, n, rt_node_id());
        n = put_u8(out, n, (uint8_t)g_lc);
        n = put_u8(out, n, state);
        n = put_u32(out, n, now);
        // Achieved dBm, not requested - the page should show what the radio is doing.
        for (int c = 0; c < CH_COUNT; c++) {
            n = put_u8(out, n, (uint8_t)rt_power_actual(c));
        }
        n = put_u8(out, n, rt_reset_code());
        n = put_u32(out, n, (uint32_t)esp_get_free_heap_size());
        n = put_u32(out, n, (uint32_t)esp_get_minimum_free_heap_size());
        n = put_u32(out, n, f154);
        n = put_u32(out, n, o154);
        n = put_u32(out, n, coex);
        n = put_u8(out, n, (uint8_t)(total > 255 ? 255 : total));

        for (int c = 0; c < CH_COUNT; c++) {
            n = put_u32(out, n, s_tx_count[c]);
            n = put_u32(out, n, s_tx_ok[c]);
            n = put_u32(out, n, s_tx_fail[c]);
            n = put_u16(out, n, s_rx_offmode[c]);
            n = put_u16(out, n, s_rx_offmode[c] ? (now - s_rx_offmode_ms[c]) / 1000 : 0);
        }
        // The page decodes this block at fixed offsets, so a field added here without updating
        // RT_RPT_STATUS - and the matching reader - would silently shift every row that
        // follows. Say so loudly instead; a wrong offset is not a thing to discover from a
        // graph that looks a bit odd.
        if (n - RT_RPT_HDR != RT_RPT_STATUS) {
            ESP_LOGE(TAG, "status block is %d bytes, RT_RPT_STATUS says %d - the page will "
                          "mis-decode every row", n - RT_RPT_HDR, RT_RPT_STATUS);
        }
        st->started = 1;
    }

    while (st->row_next < total && n + RT_RPT_ROW <= cap) {
        uint8_t  peer = 0, chan = 0;
        rt_link *l = row_at(st->row_next, &peer, &chan);
        if (l == NULL) {
            break;  // table changed under us; the next report carries the truth
        }
        st->row_next++;

        const uint32_t wtot = l->wrx + l->wmissed;
        const int      wpdr = wtot ? (int)((l->wrx * 100) / wtot) : -1;
        const uint32_t tot  = l->rx + l->missed;
        const int      tpdr = tot ? (int)((l->rx * 100) / tot) : -1;
        const int      mean = l->rssi_n ? (int)(l->rssi_sum / l->rssi_n) : 0;
        const int      snr  = (l->noise == RT_NOISE_NONE) ? -128 : (l->rssi_last - l->noise);

        n = put_u8(out, n, peer);
        n = put_u8(out, n, chan);
        n = put_u8(out, n, (uint8_t)l->rssi_last);
        n = put_u8(out, n, (uint8_t)mean);
        n = put_u8(out, n, (uint8_t)l->rssi_min);
        n = put_u8(out, n, (uint8_t)l->rssi_max);
        n = put_u8(out, n, (uint8_t)wpdr);
        n = put_u8(out, n, (uint8_t)tpdr);
        n = put_u32(out, n, l->rx);
        n = put_u32(out, n, l->missed);
        n = put_u16(out, n, now - l->last_ms);
        n = put_u8(out, n, (uint8_t)snr);
        n = put_u8(out, n, l->lqi);
        n = put_u8(out, n, (uint8_t)l->peer_txdbm);
    }

    const bool last = (st->row_next >= total);
    out[0] = type;
    out[1] = gen;
    out[2] = (uint8_t)st->chunk;
    out[3] = last ? 0x01 : 0x00;
    st->chunk++;
    return n;
}

// End of a report period. Clears the window counters that "pdr now" is computed from.
//
// This is called by app_main once, after *both* consumers have read them - and that ordering is
// the whole point. rt_report() used to clear them itself, as the last thing it did per link,
// and app_main calls rt_report() immediately before rt_ui_notify(). So the serial table got a
// real "pdr now" and the phone read the counters microseconds after they were zeroed, which
// made wtot zero, which made the page's pdr_now -1 - every report, since the column existed.
//
// Two readers of one window means neither of them may own clearing it.
void rt_snapshot_window_reset(void)
{
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        if (!s_peers[i].used) {
            continue;
        }
        for (int c = 0; c < CH_COUNT; c++) {
            s_peers[i].ch[c].wrx     = 0;
            s_peers[i].ch[c].wmissed = 0;
        }
    }
}

void rt_report(void)
{
    const uint32_t now = rt_ms();

    // wifi= is the Wi-Fi *driver*, not the ESP-NOW tx gate on the line below. The two are
    // separate on purpose: a mode that mutes ESP-NOW while leaving the driver up is the exact
    // failure this rig kept measuring, so the report has to be able to show that state.
    uint32_t f154 = 0, o154 = 0, coex = 0;
    rt_154_counters(&f154, &o154, &coex);

    // Everything that matters, every report. A one-off startup banner is invisible to anyone
    // who was not watching at the moment it scrolled past - and the moment worth watching is
    // always the one after something went wrong, by which time the banner is long gone.
    printf("\n== node %02X  up %lus  lc=%s  lr=%s  wifi=%s  ant=%s  rst=%s ==\n", rt_node_id(),
           (unsigned long)(now / 1000), lc_name(g_lc), g_lr ? "on" : "off",
           rt_wifi_active() ? "on" : "off", g_ant_ext ? "ext" : "int", rt_reset_reason());
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

    printf("  154 rx: %lu frames, %lu ours, %lu coex-refused tx\n",
           (unsigned long)f154, (unsigned long)o154, (unsigned long)coex);
    printf("  heap: %lu free, %lu min\n",
           (unsigned long)esp_get_free_heap_size(),
           (unsigned long)esp_get_minimum_free_heap_size());

    // Loud, and repeated for as long as it stands. A packet arriving on a channel this mode
    // says is off is a bug, and it is spending airtime that another channel was promised.
    // With the age of the most recent one, because a count on its own cannot say whether this
    // is still happening or happened once during a mode switch twenty minutes ago - and those
    // are completely different problems.
    for (int c = 0; c < CH_COUNT; c++) {
        if (s_rx_offmode[c]) {
            printf("  !! %s: %lu packets received while this mode says it is OFF"
                   " (most recent %lus ago)\n",
                   rt_chan_name[c], (unsigned long)s_rx_offmode[c],
                   (unsigned long)((now - s_rx_offmode_ms[c]) / 1000));
        }
    }

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
            // txdbm - rssi. Independent of either end's power setting, so it is the figure to
            // compare against free-space loss for the distance, and the one that says whether
            // an antenna change actually did anything.
            printf(" loss %ddB", l->peer_txdbm - l->rssi_last);
            if (l->noise != RT_NOISE_NONE) {
                printf(" snr %d (noise %d)", l->rssi_last - l->noise, l->noise);
            }
            if (c == CH_154) {
                printf(" lqi %u", l->lqi);
            }
            printf("\n");
            // Window counters are NOT cleared here - app_main does it after the phone report
            // has been built too. See rt_snapshot_window_reset().
        }
    }
    if (!any) {
        printf("  (nothing heard yet)\n");
    }
}

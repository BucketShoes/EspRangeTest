// The results table: one row per (peer, channel), printed over serial.

#include <stdio.h>
#include <string.h>

#include "esp_mac.h"
#include "esp_timer.h"

#include "rt.h"

const char *rt_chan_name[CH_COUNT] = { "espnow", "ble_adv", "154" };

volatile int g_solo = 0;

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
static uint32_t s_tx_fail[CH_COUNT];
static uint8_t  s_node_id;

uint32_t rt_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
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
    return g_solo == 0 || g_solo == chan + 1;
}

void rt_set_solo(int solo)
{
    if (solo < 0 || solo > CH_COUNT) {
        return;
    }
    g_solo = solo;
    // Sequence numbers restart, so wipe what we have rather than let the restart read as a
    // huge run of losses.
    memset(s_peers, 0, sizeof(s_peers));
    memset(s_tx_seq, 0, sizeof(s_tx_seq));
    memset(s_tx_count, 0, sizeof(s_tx_count));
    memset(s_tx_fail, 0, sizeof(s_tx_fail));

    printf("\n>>> solo = %s\n",
           solo == 0 ? "off (all radios)" : rt_chan_name[solo - 1]);
}

void rt_fill(rt_pkt_t *p, int chan, int8_t txdbm)
{
    p->magic = RT_MAGIC;
    p->node  = rt_node_id();
    p->txdbm = txdbm;
    p->seq   = s_tx_seq[chan]++;
    s_tx_count[chan]++;
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
        snprintf(out[n++], RT_LINE_MAX, "S,%02X,%lu,%d", rt_node_id(),
                 (unsigned long)(now / 1000), g_solo);
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

    printf("\n== node %02X  up %lus  solo=%s ==\n", rt_node_id(),
           (unsigned long)(now / 1000),
           g_solo == 0 ? "off" : rt_chan_name[g_solo - 1]);
    printf("  tx: ");
    for (int c = 0; c < CH_COUNT; c++) {
        printf("%s=%lu%s", rt_chan_name[c], (unsigned long)s_tx_count[c],
               rt_tx_enabled(c) ? "" : "(off)");
        if (s_tx_fail[c]) {
            printf("(%lu failed)", (unsigned long)s_tx_fail[c]);
        }
        printf(" ");
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

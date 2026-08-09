#include "rt_obs.h"

#include <string.h>

static rt_peer_t *peer_find(rt_obs_table_t *t, uint8_t node_id)
{
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        if (t->peers[i].used && t->peers[i].node_id == node_id) {
            return &t->peers[i];
        }
    }
    return NULL;
}

static rt_peer_t *peer_find_or_add(rt_obs_table_t *t, uint8_t node_id)
{
    rt_peer_t *p = peer_find(t, node_id);
    if (p != NULL) {
        return p;
    }
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        if (!t->peers[i].used) {
            memset(&t->peers[i], 0, sizeof(t->peers[i]));
            t->peers[i].used    = true;
            t->peers[i].node_id = node_id;
            return &t->peers[i];
        }
    }
    return NULL;  // full; see the note in rt_obs.h about not evicting
}

void rt_obs_table_init(rt_obs_table_t *t)
{
    if (t == NULL) {
        return;
    }
    memset(t, 0, sizeof(*t));
}

void rt_obs_clear_all(rt_obs_table_t *t)
{
    if (t == NULL) {
        return;
    }
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        if (!t->peers[i].used) {
            continue;
        }
        for (int c = 0; c < RT_CHAN_COUNT; c++) {
            rt_link_t *l = &t->peers[i].links[c];
            l->head       = 0;
            l->count      = 0;
            l->have_epoch = false;
        }
    }
}

bool rt_obs_add(rt_obs_table_t *t, uint8_t node_id, rt_chan_t chan, uint8_t epoch,
                const rt_obs_t *obs)
{
    if (t == NULL || obs == NULL || (unsigned)chan >= RT_CHAN_COUNT) {
        return false;
    }

    rt_peer_t *p = peer_find_or_add(t, node_id);
    if (p == NULL) {
        return false;
    }

    rt_link_t *l = &p->links[chan];

    // A sender epoch change means that sender switched mode and restarted its sequence
    // numbering. Keeping the old entries would both misread the restart as a huge loss run
    // and average together samples taken under two different radio configurations.
    const uint8_t e = (uint8_t)(epoch & 0x0Fu);
    if (!l->have_epoch || l->epoch != e) {
        l->head       = 0;
        l->count      = 0;
        l->epoch      = e;
        l->have_epoch = true;
    }

    l->ring[l->head] = *obs;
    l->head          = (uint8_t)((l->head + 1u) % RT_OBS_WINDOW);
    if (l->count < RT_OBS_WINDOW) {
        l->count++;
    }

    return true;
}

const rt_peer_t *rt_obs_peer(const rt_obs_table_t *t, uint8_t node_id)
{
    if (t == NULL) {
        return NULL;
    }
    return peer_find((rt_obs_table_t *)t, node_id);
}

uint8_t rt_obs_peer_count(const rt_obs_table_t *t)
{
    if (t == NULL) {
        return 0;
    }
    uint8_t n = 0;
    for (int i = 0; i < RT_MAX_PEERS; i++) {
        if (t->peers[i].used) {
            n++;
        }
    }
    return n;
}

void rt_obs_summary(const rt_obs_table_t *t, uint8_t node_id, rt_chan_t chan,
                    uint32_t now_ms, rt_link_summary_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->rssi_min = 127;
    out->rssi_max = -128;

    if (t == NULL || (unsigned)chan >= RT_CHAN_COUNT) {
        return;
    }
    const rt_peer_t *p = rt_obs_peer(t, node_id);
    if (p == NULL) {
        return;
    }
    const rt_link_t *l = &p->links[chan];
    if (l->count == 0) {
        return;
    }

    out->valid        = true;
    out->rx_in_window = l->count;

    // Walk the ring oldest-to-newest. The oldest valid entry sits count places behind head.
    const uint8_t oldest = (uint8_t)((l->head + RT_OBS_WINDOW - l->count) % RT_OBS_WINDOW);

    int32_t  rssi_sum = 0;
    uint32_t missed   = 0;
    bool     have_prev = false;
    uint16_t prev_seq  = 0;

    for (uint8_t i = 0; i < l->count; i++) {
        const rt_obs_t *o = &l->ring[(oldest + i) % RT_OBS_WINDOW];

        rssi_sum += o->rssi;
        if (o->rssi < out->rssi_min) {
            out->rssi_min = o->rssi;
        }
        if (o->rssi > out->rssi_max) {
            out->rssi_max = o->rssi;
        }

        if (have_prev) {
            // Unsigned 16-bit subtraction wraps correctly, so a sequence rollover costs
            // nothing special here.
            const uint16_t delta = (uint16_t)(o->seq - prev_seq);
            if (delta >= 1u && delta <= RT_SEQ_SANE_MAX) {
                missed += (uint32_t)(delta - 1u);
            }
            // delta == 0 is a duplicate and delta > RT_SEQ_SANE_MAX is garbage or a
            // backwards jump; neither is evidence of loss, so both contribute nothing.
        }
        prev_seq  = o->seq;
        have_prev = true;
    }

    const rt_obs_t *newest = &l->ring[(l->head + RT_OBS_WINDOW - 1u) % RT_OBS_WINDOW];

    out->rssi_last     = newest->rssi;
    out->rssi_mean_x10 = (int16_t)((rssi_sum * 10) / (int32_t)l->count);
    out->rx_phy_last   = newest->rx_phy;
    out->tx_phy_last   = newest->tx_phy;
    out->tx_dbm_last   = newest->tx_dbm;
    out->mode_id_last  = newest->mode_id;

    out->missed_in_window = (missed > 0xFFFFu) ? 0xFFFFu : (uint16_t)missed;

    // Unsigned subtraction, so this stays correct across the 49-day millisecond rollover.
    const uint32_t age = now_ms - newest->t_ms;
    out->age_ms = age;

    // A channel that has gone silent must read as loss, not as an absence of data - that
    // silence is the single most important observation on a range walk.
    const uint32_t interval = rt_chan_expected_interval_ms(chan);
    if (interval > 0u && age > interval) {
        const uint32_t stale = age / interval;
        out->stale_missed = (stale > 0xFFFFu) ? 0xFFFFu : (uint16_t)stale;
    }
}

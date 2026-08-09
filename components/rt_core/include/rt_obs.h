// The observation table - the spine of the system.
//
// Every board is dumb: it reports only what it directly hears, keyed on (peer, channel),
// and keeps the last RT_OBS_WINDOW receptions for each pair in RAM. There is no flash
// logging (flash writes stall the system, which would be its own contention source) and no
// history (history is useless without knowing where you were, so it lives in the browser
// alongside the GPS fix).
//
// The window slides continuously: it is "the last 20 receptions", not "everything since
// the last reset tick". A periodic hard reset would produce step changes in the reported
// numbers at the reset instant that correspond to nothing in the underlying signal.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "rt_chan.h"
#include "rt_payload.h"  // rt_phy_t: the rx_phy/tx_phy fields below carry those values

#define RT_OBS_WINDOW 20
#define RT_MAX_PEERS  8

// Sentinels for the best-effort extras, which several APIs simply do not provide.
#define RT_NOISE_NA ((int8_t)0x7F)
#define RT_LQI_NA   ((uint8_t)0xFF)

// A sequence delta larger than this is treated as garbage rather than as a colossal run of
// losses - a corrupt-but-CRC-valid id, a stale frame arriving late, or a peer that rebooted
// without bumping its epoch would otherwise inject a huge phantom loss count.
#define RT_SEQ_SANE_MAX 4096u

typedef struct {
    uint32_t t_ms;         // ms since boot on THIS board; no cross-node clock sync is attempted
    uint16_t seq;
    int8_t   rssi;
    uint8_t  mode_id;      // the RECEIVER's mode at reception - the contention annotation
    uint8_t  rx_phy;       // what the receive path reported, where the API exposes it
    uint8_t  tx_phy;       // what the sender claimed
    int8_t   tx_dbm;
    int8_t   noise_floor;  // RT_NOISE_NA when unavailable
    uint8_t  lqi;          // RT_LQI_NA when unavailable
} rt_obs_t;

typedef struct {
    rt_obs_t ring[RT_OBS_WINDOW];
    uint8_t  head;   // next write position
    uint8_t  count;  // valid entries, saturating at RT_OBS_WINDOW
    uint8_t  epoch;  // sender's epoch for this link; a change clears the window
    bool     have_epoch;
} rt_link_t;

typedef struct {
    bool      used;
    uint8_t   node_id;
    rt_link_t links[RT_CHAN_COUNT];
} rt_peer_t;

typedef struct {
    rt_peer_t peers[RT_MAX_PEERS];
} rt_obs_table_t;

// What gets pushed to the UI for each (peer, channel).
typedef struct {
    bool     valid;            // false when nothing has ever been heard on this pair
    int8_t   rssi_last;
    int16_t  rssi_mean_x10;    // fixed point: mean RSSI * 10, to stay off floating point
    int8_t   rssi_min;
    int8_t   rssi_max;
    uint16_t rx_in_window;     // receptions currently held, <= RT_OBS_WINDOW
    uint16_t missed_in_window; // sequence numbers absent BETWEEN the held receptions
    uint16_t stale_missed;     // presumed missed since the last reception, from its age
    uint32_t age_ms;           // time since the most recent reception
    uint8_t  rx_phy_last;
    uint8_t  tx_phy_last;
    int8_t   tx_dbm_last;
    uint8_t  mode_id_last;
} rt_link_summary_t;

void rt_obs_table_init(rt_obs_table_t *t);

// Record a reception. Peers are allocated on first sight; once RT_MAX_PEERS are known,
// further new peers are dropped rather than evicting an existing one (a peer that has been
// heard from is more valuable than one we have only just seen, and silently rotating the
// table mid-walk would corrupt the numbers).
// Returns false only if the peer slot could not be allocated.
bool rt_obs_add(rt_obs_table_t *t, uint8_t node_id, rt_chan_t chan, uint8_t epoch,
                const rt_obs_t *obs);

// Summarise one (peer, channel). now_ms is passed in rather than read from a clock so this
// stays pure and host-testable.
void rt_obs_summary(const rt_obs_table_t *t, uint8_t node_id, rt_chan_t chan,
                    uint32_t now_ms, rt_link_summary_t *out);

const rt_peer_t *rt_obs_peer(const rt_obs_table_t *t, uint8_t node_id);
uint8_t          rt_obs_peer_count(const rt_obs_table_t *t);

// Drop everything heard from every peer. Used when this board's own mode changes: the
// receiver's contention state is part of what each sample means, so samples taken under
// the previous mode must not be averaged into the new one.
void rt_obs_clear_all(rt_obs_table_t *t);

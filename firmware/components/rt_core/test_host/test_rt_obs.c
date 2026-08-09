#include <string.h>

#include "rt_obs.h"
#include "test_util.h"

#define PEER 0x11
#define CHAN RT_CHAN_ESPNOW

static rt_obs_table_t g_t;

static void add(uint8_t node, rt_chan_t chan, uint8_t epoch, uint16_t seq, int8_t rssi,
                uint32_t t_ms)
{
    const rt_obs_t o = {
        .t_ms        = t_ms,
        .seq         = seq,
        .rssi        = rssi,
        .mode_id     = 0,
        .rx_phy      = RT_PHY_WIFI_11B,
        .tx_phy      = RT_PHY_WIFI_11B,
        .tx_dbm      = 20,
        .noise_floor = RT_NOISE_NA,
        .lqi         = RT_LQI_NA,
    };
    CHECK(rt_obs_add(&g_t, node, chan, epoch, &o));
}

static void reset(void)
{
    rt_obs_table_init(&g_t);
    rt_chan_set_expected_interval_ms(CHAN, 250);
}

static void test_empty_pair_is_invalid(void)
{
    reset();
    rt_link_summary_t s;
    rt_obs_summary(&g_t, PEER, CHAN, 0, &s);
    CHECK(!s.valid);
    CHECK_EQ(rt_obs_peer_count(&g_t), 0);
}

static void test_contiguous_sequence_has_no_loss(void)
{
    reset();
    for (uint16_t i = 0; i < 10; i++) {
        add(PEER, CHAN, 0, (uint16_t)(100 + i), -50, (uint32_t)(i * 250u));
    }

    rt_link_summary_t s;
    rt_obs_summary(&g_t, PEER, CHAN, 9u * 250u, &s);
    CHECK(s.valid);
    CHECK_EQ(s.rx_in_window, 10);
    CHECK_EQ(s.missed_in_window, 0);
    CHECK_EQ(s.stale_missed, 0);
    CHECK_EQ(s.rssi_last, -50);
    CHECK_EQ(s.rssi_mean_x10, -500);
    CHECK_EQ(s.age_ms, 0);
}

static void test_gaps_are_counted(void)
{
    reset();
    // 1, 2, then 5 (3 and 4 lost), then 6, then 10 (7, 8, 9 lost) => 5 missing.
    const uint16_t seqs[] = { 1, 2, 5, 6, 10 };
    for (unsigned i = 0; i < 5; i++) {
        add(PEER, CHAN, 0, seqs[i], -60, (uint32_t)(i * 250u));
    }

    rt_link_summary_t s;
    rt_obs_summary(&g_t, PEER, CHAN, 4u * 250u, &s);
    CHECK_EQ(s.rx_in_window, 5);
    CHECK_EQ(s.missed_in_window, 5);
}

// The window is the last 20 receptions and slides continuously - there is no reset
// boundary at which the reported number steps without the signal having changed.
static void test_window_slides_and_evicts(void)
{
    reset();
    for (uint16_t i = 0; i < RT_OBS_WINDOW + 10u; i++) {
        add(PEER, CHAN, 0, (uint16_t)(i + 1u), -40, (uint32_t)(i * 250u));
    }

    rt_link_summary_t s;
    rt_obs_summary(&g_t, PEER, CHAN, (RT_OBS_WINDOW + 9u) * 250u, &s);
    CHECK_EQ(s.rx_in_window, RT_OBS_WINDOW);
    CHECK_EQ(s.missed_in_window, 0);
    CHECK_EQ(s.rssi_last, -40);

    // Losses that have slid out of the window must stop being reported.
    reset();
    add(PEER, CHAN, 0, 1, -40, 0);
    add(PEER, CHAN, 0, 500, -40, 250);  // a 498-packet hole
    rt_obs_summary(&g_t, PEER, CHAN, 250, &s);
    CHECK_EQ(s.missed_in_window, 498);

    for (uint16_t i = 0; i < RT_OBS_WINDOW; i++) {
        add(PEER, CHAN, 0, (uint16_t)(501u + i), -40, (uint32_t)(500u + i * 250u));
    }
    rt_obs_summary(&g_t, PEER, CHAN, 500u + (RT_OBS_WINDOW - 1u) * 250u, &s);
    CHECK_EQ(s.missed_in_window, 0);
}

static void test_sequence_wraparound_is_not_loss(void)
{
    reset();
    add(PEER, CHAN, 0, 65534, -55, 0);
    add(PEER, CHAN, 0, 65535, -55, 250);
    add(PEER, CHAN, 0, 0, -55, 500);
    add(PEER, CHAN, 0, 1, -55, 750);

    rt_link_summary_t s;
    rt_obs_summary(&g_t, PEER, CHAN, 750, &s);
    CHECK_EQ(s.rx_in_window, 4);
    CHECK_EQ(s.missed_in_window, 0);
}

static void test_duplicates_and_backwards_jumps_are_not_loss(void)
{
    reset();
    add(PEER, CHAN, 0, 10, -55, 0);
    add(PEER, CHAN, 0, 10, -55, 250);  // duplicate
    add(PEER, CHAN, 0, 11, -55, 500);
    add(PEER, CHAN, 0, 9, -55, 750);   // reordered/backwards - garbage, not 65534 losses

    rt_link_summary_t s;
    rt_obs_summary(&g_t, PEER, CHAN, 750, &s);
    CHECK_EQ(s.missed_in_window, 0);
}

// A mode change on the SENDER restarts its numbering. Without epoch handling that reads as
// a catastrophic loss run, and worse, averages together samples from two different radio
// configurations.
static void test_epoch_change_clears_the_window(void)
{
    reset();
    for (uint16_t i = 0; i < 10; i++) {
        add(PEER, CHAN, 3, (uint16_t)(1000 + i), -50, (uint32_t)(i * 250u));
    }

    add(PEER, CHAN, 4, 0, -70, 2500);  // new epoch, sequence restarts at 0

    rt_link_summary_t s;
    rt_obs_summary(&g_t, PEER, CHAN, 2500, &s);
    CHECK_EQ(s.rx_in_window, 1);
    CHECK_EQ(s.missed_in_window, 0);
    CHECK_EQ(s.rssi_last, -70);
    CHECK_EQ(s.rssi_mean_x10, -700);
}

// Silence is the single most important observation on a range walk, so it has to read as
// loss rather than as an absence of data.
static void test_staleness_becomes_presumed_loss(void)
{
    reset();
    add(PEER, CHAN, 0, 1, -80, 1000);

    rt_link_summary_t s;

    rt_obs_summary(&g_t, PEER, CHAN, 1100, &s);  // within one interval
    CHECK_EQ(s.age_ms, 100);
    CHECK_EQ(s.stale_missed, 0);

    rt_obs_summary(&g_t, PEER, CHAN, 6000, &s);  // 5000ms silent at a 250ms cadence
    CHECK_EQ(s.age_ms, 5000);
    CHECK_EQ(s.stale_missed, 20);

    // A channel with no regular cadence cannot have packets inferred as missing.
    rt_chan_set_expected_interval_ms(CHAN, 0);
    rt_obs_summary(&g_t, PEER, CHAN, 6000, &s);
    CHECK_EQ(s.stale_missed, 0);
    rt_chan_set_expected_interval_ms(CHAN, 250);
}

// A walk can easily outlast a 32-bit microsecond counter; milliseconds give 49 days, and
// unsigned subtraction has to stay correct across the rollover regardless.
static void test_millisecond_rollover(void)
{
    reset();
    const uint32_t near_max = 0xFFFFFF00u;
    add(PEER, CHAN, 0, 1, -60, near_max);

    rt_link_summary_t s;
    const uint32_t now = near_max + 1000u;  // wraps past zero
    rt_obs_summary(&g_t, PEER, CHAN, now, &s);
    CHECK_EQ(s.age_ms, 1000);
    CHECK_EQ(s.stale_missed, 4);
}

static void test_channels_and_peers_are_independent(void)
{
    reset();
    add(PEER, RT_CHAN_ESPNOW, 0, 1, -50, 0);
    add(PEER, RT_CHAN_BLE_ADV_CODED, 0, 77, -90, 0);
    add(0x22, RT_CHAN_ESPNOW, 0, 1, -30, 0);

    CHECK_EQ(rt_obs_peer_count(&g_t), 2);

    rt_link_summary_t s;
    rt_obs_summary(&g_t, PEER, RT_CHAN_ESPNOW, 0, &s);
    CHECK_EQ(s.rssi_last, -50);
    rt_obs_summary(&g_t, PEER, RT_CHAN_BLE_ADV_CODED, 0, &s);
    CHECK_EQ(s.rssi_last, -90);
    rt_obs_summary(&g_t, 0x22, RT_CHAN_ESPNOW, 0, &s);
    CHECK_EQ(s.rssi_last, -30);
    rt_obs_summary(&g_t, 0x22, RT_CHAN_BLE_ADV_CODED, 0, &s);
    CHECK(!s.valid);
}

// A peer that has been heard from is worth more than one just glimpsed, and silently
// rotating the table mid-walk would corrupt the numbers - so the table fills and stops.
static void test_peer_table_fills_without_evicting(void)
{
    reset();
    for (uint8_t i = 0; i < RT_MAX_PEERS; i++) {
        add((uint8_t)(1 + i), CHAN, 0, 1, -50, 0);
    }
    CHECK_EQ(rt_obs_peer_count(&g_t), RT_MAX_PEERS);

    const rt_obs_t o = { .t_ms = 0, .seq = 1, .rssi = -50, .noise_floor = RT_NOISE_NA,
                         .lqi = RT_LQI_NA };
    CHECK(!rt_obs_add(&g_t, 200, CHAN, 0, &o));
    CHECK_EQ(rt_obs_peer_count(&g_t), RT_MAX_PEERS);

    rt_link_summary_t s;
    rt_obs_summary(&g_t, 1, CHAN, 0, &s);
    CHECK(s.valid);  // the established peer survived
}

// This board's own mode change invalidates every sample, because the receiver's contention
// state is part of what each sample means.
static void test_clear_all_keeps_peers_but_drops_samples(void)
{
    reset();
    add(PEER, CHAN, 0, 1, -50, 0);
    add(0x22, CHAN, 0, 1, -50, 0);
    CHECK_EQ(rt_obs_peer_count(&g_t), 2);

    rt_obs_clear_all(&g_t);

    CHECK_EQ(rt_obs_peer_count(&g_t), 2);
    rt_link_summary_t s;
    rt_obs_summary(&g_t, PEER, CHAN, 0, &s);
    CHECK(!s.valid);
}

static void test_rssi_aggregates(void)
{
    reset();
    add(PEER, CHAN, 0, 1, -40, 0);
    add(PEER, CHAN, 0, 2, -60, 250);
    add(PEER, CHAN, 0, 3, -50, 500);

    rt_link_summary_t s;
    rt_obs_summary(&g_t, PEER, CHAN, 500, &s);
    CHECK_EQ(s.rssi_min, -60);
    CHECK_EQ(s.rssi_max, -40);
    CHECK_EQ(s.rssi_mean_x10, -500);
    CHECK_EQ(s.rssi_last, -50);
}

int main(void)
{
    printf("test_rt_obs\n");
    RUN(test_empty_pair_is_invalid);
    RUN(test_contiguous_sequence_has_no_loss);
    RUN(test_gaps_are_counted);
    RUN(test_window_slides_and_evicts);
    RUN(test_sequence_wraparound_is_not_loss);
    RUN(test_duplicates_and_backwards_jumps_are_not_loss);
    RUN(test_epoch_change_clears_the_window);
    RUN(test_staleness_becomes_presumed_loss);
    RUN(test_millisecond_rollover);
    RUN(test_channels_and_peers_are_independent);
    RUN(test_peer_table_fills_without_evicting);
    RUN(test_clear_all_keeps_peers_but_drops_samples);
    RUN(test_rssi_aggregates);
    TEST_MAIN_END();
}

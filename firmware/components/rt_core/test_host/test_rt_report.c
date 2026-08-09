#include <string.h>

#include "rt_report.h"
#include "test_util.h"

static rt_obs_table_t  g_t;
static rt_mode_state_t g_m;

static void add(uint8_t node, rt_chan_t chan, uint16_t seq, int8_t rssi, uint32_t t_ms,
                uint8_t rx_phy)
{
    const rt_obs_t o = {
        .t_ms = t_ms, .seq = seq, .rssi = rssi, .mode_id = 0,
        .rx_phy = rx_phy, .tx_phy = rx_phy, .tx_dbm = 9,
        .noise_floor = RT_NOISE_NA, .lqi = RT_LQI_NA,
    };
    rt_obs_add(&g_t, node, chan, 0, &o);
}

static void reset(void)
{
    rt_obs_table_init(&g_t);
    rt_mode_init(&g_m, 0);
}

static void test_empty_table_says_so(void)
{
    reset();
    char buf[512];
    const size_t n = rt_report_render(&g_t, &g_m, 0x42, 5000, buf, sizeof(buf));
    CHECK(n > 0);
    CHECK(strstr(buf, "no peers heard yet") != NULL);
    CHECK(strstr(buf, "normal") != NULL);
    CHECK(strstr(buf, "0x42") != NULL);
    CHECK_EQ(buf[n], '\0');
}

static void test_renders_rows_and_negative_mean(void)
{
    reset();
    add(0x11, RT_CHAN_BLE_ADV_CODED, 1, -70, 0, RT_PHY_BLE_CODED_S8);
    add(0x11, RT_CHAN_BLE_ADV_CODED, 2, -71, 500, RT_PHY_BLE_CODED_S8);

    char buf[1024];
    rt_report_render(&g_t, &g_m, 0x42, 500, buf, sizeof(buf));

    CHECK(strstr(buf, "ble_adv_coded") != NULL);
    CHECK(strstr(buf, "ble_coded_s8") != NULL);
    CHECK(strstr(buf, "-70.5") != NULL);  // mean of -70 and -71
}

// A mean between 0 and -1 must not print as "0.5" - the sign lives in a part of the number
// that integer division throws away.
static void test_small_negative_mean_keeps_its_sign(void)
{
    reset();
    add(0x11, RT_CHAN_ESPNOW, 1, 0, 0, RT_PHY_WIFI_11B);
    add(0x11, RT_CHAN_ESPNOW, 2, -1, 250, RT_PHY_WIFI_11B);

    char buf[1024];
    rt_report_render(&g_t, &g_m, 0x42, 250, buf, sizeof(buf));
    CHECK(strstr(buf, "-   0.5") != NULL || strstr(buf, "-0.5") != NULL);
    CHECK(strstr(buf, " 0.5") == NULL || strstr(buf, "-") != NULL);
}

// Pairs never heard from are absences, not zeroes, and must not be printed at all.
static void test_unheard_pairs_are_omitted(void)
{
    reset();
    add(0x11, RT_CHAN_ESPNOW, 1, -50, 0, RT_PHY_WIFI_11B);

    char buf[2048];
    rt_report_render(&g_t, &g_m, 0x42, 0, buf, sizeof(buf));
    CHECK(strstr(buf, "espnow") != NULL);
    CHECK(strstr(buf, "ieee802154") == NULL);
    CHECK(strstr(buf, "ftm") == NULL);
}

// Truncation must stay inside the buffer and stay NUL-terminated - this runs on a device
// where overrunning it corrupts something else entirely.
static void test_truncation_is_safe(void)
{
    reset();
    for (uint8_t p = 0; p < RT_MAX_PEERS; p++) {
        for (int c = 0; c < RT_CHAN_COUNT; c++) {
            add((uint8_t)(1 + p), (rt_chan_t)c, 1, -50, 0, RT_PHY_WIFI_11B);
        }
    }

    for (size_t cap = 1; cap < 200; cap++) {
        char buf[256];
        memset(buf, 0x7E, sizeof(buf));
        const size_t n = rt_report_render(&g_t, &g_m, 0x42, 0, buf, cap);
        CHECK(n < cap);
        CHECK_EQ(buf[n], '\0');
        CHECK_EQ((unsigned char)buf[cap], 0x7Eu);  // nothing written past the limit
    }
}

static void test_zero_length_buffer_is_survivable(void)
{
    reset();
    char buf[4] = { 'x', 'x', 'x', 'x' };
    CHECK_EQ(rt_report_render(&g_t, &g_m, 0x42, 0, buf, 0), 0);
    CHECK_EQ(buf[0], 'x');  // untouched
    CHECK_EQ(rt_report_render(&g_t, &g_m, 0x42, 0, NULL, 100), 0);
}

static void test_mode_line_tracks_state(void)
{
    reset();
    rt_mode_set(&g_m, RT_MODE_JUST_154, 1000);
    rt_mode_set_keep_ui_ble(&g_m, false, 1000);

    char buf[512];
    rt_report_render(&g_t, &g_m, 0x42, 61000, buf, sizeof(buf));
    CHECK(strstr(buf, "just_154") != NULL);
    CHECK(strstr(buf, "ui_ble=off") != NULL);
    CHECK(strstr(buf, "up 61s") != NULL);
}

int main(void)
{
    printf("test_rt_report\n");
    RUN(test_empty_table_says_so);
    RUN(test_renders_rows_and_negative_mean);
    RUN(test_small_negative_mean_keeps_its_sign);
    RUN(test_unheard_pairs_are_omitted);
    RUN(test_truncation_is_safe);
    RUN(test_zero_length_buffer_is_survivable);
    RUN(test_mode_line_tracks_state);
    TEST_MAIN_END();
}

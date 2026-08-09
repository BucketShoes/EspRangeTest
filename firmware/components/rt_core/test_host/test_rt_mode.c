#include "rt_mode.h"
#include "test_util.h"

static void test_defaults(void)
{
    rt_mode_state_t s;
    rt_mode_init(&s, 1000);
    CHECK_EQ(s.mode, RT_MODE_NORMAL);
    CHECK(s.keep_ui_ble);
    CHECK(s.auto_revert);
    CHECK_EQ(s.epoch, 0);
}

// Isolation is the whole point: "just X" must actually mean only X (plus the UI link, when
// that is deliberately kept).
static void test_isolation_masks(void)
{
    CHECK_EQ(rt_mode_radios(RT_MODE_JUST_154, false), RT_RADIO_IEEE802154);
    CHECK_EQ(rt_mode_radios(RT_MODE_JUST_BLE, false),
             RT_RADIO_BLE_ADV_CODED | RT_RADIO_BLE_ADV_LEGACY | RT_RADIO_BLE_CONN);

    // No isolation mode may leave 802.15.4 running unless it is the subject.
    for (int m = 0; m < RT_MODE_COUNT; m++) {
        if (m == RT_MODE_NORMAL || m == RT_MODE_JUST_154) {
            continue;
        }
        CHECK_EQ(rt_mode_radios((rt_mode_t)m, true) & RT_RADIO_IEEE802154, 0);
    }

    // Nor Wi-Fi, unless the mode is a Wi-Fi one.
    const rt_radio_mask_t wifi = RT_RADIO_WIFI_AP | RT_RADIO_WIFI_STA | RT_RADIO_ESPNOW;
    CHECK_EQ(rt_mode_radios(RT_MODE_JUST_BLE, true) & wifi, 0);
    CHECK_EQ(rt_mode_radios(RT_MODE_JUST_154, true) & wifi, 0);
}

static void test_keep_ui_ble_adds_only_ble(void)
{
    const rt_radio_mask_t off = rt_mode_radios(RT_MODE_JUST_154, false);
    const rt_radio_mask_t on  = rt_mode_radios(RT_MODE_JUST_154, true);

    CHECK_EQ(off, RT_RADIO_IEEE802154);
    CHECK(on & RT_RADIO_BLE_CONN);
    CHECK(on & RT_RADIO_BLE_ADV_LEGACY);  // without it Chrome cannot find the board
    CHECK_EQ(on & ~(off | RT_RADIO_BLE_ADV_CODED | RT_RADIO_BLE_ADV_LEGACY | RT_RADIO_BLE_CONN), 0);

    // NORMAL already has everything; the flag must not change it.
    CHECK_EQ(rt_mode_radios(RT_MODE_NORMAL, true), rt_mode_radios(RT_MODE_NORMAL, false));
}

static void test_ftm_needs_both_wifi_halves(void)
{
    const rt_radio_mask_t m = rt_mode_radios(RT_MODE_JUST_FTM, false);
    CHECK(m & RT_RADIO_WIFI_AP);   // responder
    CHECK(m & RT_RADIO_WIFI_STA);  // initiator
    CHECK(m & RT_RADIO_FTM);
}

// Merely having LR in the protocol list breaks the AP for phones, so crossing the LR
// boundary must always be a full Wi-Fi reinit, never a live protocol change.
static void test_lr_boundary_forces_reinit(void)
{
    CHECK(rt_mode_needs_wifi_reinit(RT_MODE_NORMAL, RT_MODE_LR));
    CHECK(rt_mode_needs_wifi_reinit(RT_MODE_LR, RT_MODE_NORMAL));
    CHECK(rt_mode_needs_wifi_reinit(RT_MODE_JUST_ESPNOW, RT_MODE_LR));
    CHECK(!rt_mode_needs_wifi_reinit(RT_MODE_NORMAL, RT_MODE_JUST_WIFI));
    CHECK(!rt_mode_needs_wifi_reinit(RT_MODE_LR, RT_MODE_LR));

    CHECK(rt_mode_radios(RT_MODE_LR, false) & RT_RADIO_LR);
    CHECK_EQ(rt_mode_radios(RT_MODE_NORMAL, false) & RT_RADIO_LR, 0);
}

static void test_epoch_bumps_only_on_real_change(void)
{
    rt_mode_state_t s;
    rt_mode_init(&s, 0);

    CHECK(rt_mode_set(&s, RT_MODE_JUST_BLE, 100));
    CHECK_EQ(s.epoch, 1);

    // Re-sending the current mode must not wipe the numbers.
    CHECK(!rt_mode_set(&s, RT_MODE_JUST_BLE, 200));
    CHECK_EQ(s.epoch, 1);

    // Toggling the UI link changes what is on the air just as surely as a mode change.
    CHECK(rt_mode_set_keep_ui_ble(&s, false, 300));
    CHECK_EQ(s.epoch, 2);
    CHECK(!rt_mode_set_keep_ui_ble(&s, false, 400));
    CHECK_EQ(s.epoch, 2);
}

static void test_epoch_wraps_within_a_nibble(void)
{
    rt_mode_state_t s;
    rt_mode_init(&s, 0);
    for (int i = 0; i < 20; i++) {
        rt_mode_next(&s, (uint32_t)i);
        CHECK(s.epoch <= 0x0F);  // must stay in the 4 bits the wire format allows
    }
}

static void test_button_cycles_and_goes_home(void)
{
    rt_mode_state_t s;
    rt_mode_init(&s, 0);

    rt_mode_next(&s, 1);
    CHECK_EQ(s.mode, RT_MODE_JUST_BLE);
    rt_mode_next(&s, 2);
    CHECK_EQ(s.mode, RT_MODE_JUST_ESPNOW);

    CHECK(rt_mode_home(&s, 3));
    CHECK_EQ(s.mode, RT_MODE_NORMAL);

    // Tapping through every mode returns to where it started.
    for (int i = 0; i < RT_MODE_COUNT; i++) {
        rt_mode_next(&s, (uint32_t)(10 + i));
    }
    CHECK_EQ(s.mode, RT_MODE_NORMAL);
}

// With the UI link off, a board left at the far end has no radio the phone can reach and
// the button is out of walking range. Auto-revert is the only way back.
static void test_auto_revert(void)
{
    rt_mode_state_t s;
    rt_mode_init(&s, 0);
    s.auto_revert_ms = 1000;

    rt_mode_set(&s, RT_MODE_JUST_154, 0);
    CHECK(!rt_mode_tick(&s, 999));
    CHECK(rt_mode_tick(&s, 1000));
    CHECK_EQ(s.mode, RT_MODE_NORMAL);

    // UI contact refreshes the deadline, so a board still in touch never reverts.
    rt_mode_set(&s, RT_MODE_JUST_154, 2000);
    rt_mode_touch(&s, 2500);
    CHECK(!rt_mode_tick(&s, 3200));
    rt_mode_touch(&s, 3300);
    CHECK(!rt_mode_tick(&s, 4000));
    CHECK_EQ(s.mode, RT_MODE_JUST_154);

    // NORMAL is home; it never reverts out from under you.
    rt_mode_home(&s, 5000);
    CHECK(!rt_mode_tick(&s, 100000));

    // And it can be switched off entirely.
    rt_mode_set(&s, RT_MODE_JUST_154, 6000);
    s.auto_revert = false;
    CHECK(!rt_mode_tick(&s, 100000));
    CHECK_EQ(s.mode, RT_MODE_JUST_154);
}

static void test_every_mode_has_a_name_and_some_radio(void)
{
    for (int m = 0; m < RT_MODE_COUNT; m++) {
        CHECK(rt_mode_name((rt_mode_t)m)[0] != '?');
        CHECK(rt_mode_radios((rt_mode_t)m, false) != 0);
    }
}

int main(void)
{
    printf("test_rt_mode\n");
    RUN(test_defaults);
    RUN(test_isolation_masks);
    RUN(test_keep_ui_ble_adds_only_ble);
    RUN(test_ftm_needs_both_wifi_halves);
    RUN(test_lr_boundary_forces_reinit);
    RUN(test_epoch_bumps_only_on_real_change);
    RUN(test_epoch_wraps_within_a_nibble);
    RUN(test_button_cycles_and_goes_home);
    RUN(test_auto_revert);
    RUN(test_every_mode_has_a_name_and_some_radio);
    TEST_MAIN_END();
}

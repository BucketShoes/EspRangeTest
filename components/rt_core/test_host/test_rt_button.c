#include "rt_button.h"
#include "test_util.h"

// Feeds a steady reading for `ms` milliseconds at a 10ms poll rate (matching the firmware
// task), returning the last non-NONE event seen.
static rt_button_event_t hold(rt_button_state_t *s, bool pressed, uint32_t *now, uint32_t ms)
{
    rt_button_event_t seen = RT_BUTTON_NONE;
    for (uint32_t elapsed = 0; elapsed < ms; elapsed += 10) {
        *now += 10;
        const rt_button_event_t ev = rt_button_update(s, pressed, *now);
        if (ev != RT_BUTTON_NONE) {
            seen = ev;
        }
    }
    return seen;
}

static void test_tap(void)
{
    rt_button_state_t s;
    uint32_t now = 0;
    rt_button_state_init(&s, now);

    CHECK_EQ(hold(&s, true, &now, 200), RT_BUTTON_NONE);   // held, but not long enough
    CHECK_EQ(hold(&s, false, &now, 100), RT_BUTTON_TAP);   // tap fires on release
}

static void test_long_press_fires_while_held_and_suppresses_the_tap(void)
{
    rt_button_state_t s;
    uint32_t now = 0;
    rt_button_state_init(&s, now);

    CHECK_EQ(hold(&s, true, &now, RT_BUTTON_LONG_PRESS_MS + 100), RT_BUTTON_LONG);
    // Releasing after a long press must not also emit a tap, or every long press would
    // change the mode twice.
    CHECK_EQ(hold(&s, false, &now, 200), RT_BUTTON_NONE);
}

static void test_long_press_fires_once(void)
{
    rt_button_state_t s;
    uint32_t now = 0;
    rt_button_state_init(&s, now);

    CHECK_EQ(hold(&s, true, &now, RT_BUTTON_LONG_PRESS_MS + 50), RT_BUTTON_LONG);
    CHECK_EQ(hold(&s, true, &now, 5000), RT_BUTTON_NONE);  // still held, stays quiet
    CHECK_EQ(hold(&s, false, &now, 100), RT_BUTTON_NONE);
}

// Contact bounce must not read as a burst of taps cycling several modes at once.
static void test_bounce_is_debounced(void)
{
    rt_button_state_t s;
    uint32_t now = 0;
    rt_button_state_init(&s, now);

    int taps = 0;
    for (int i = 0; i < 6; i++) {  // 60ms of 10ms-period chatter
        now += 10;
        if (rt_button_update(&s, (i % 2) == 0, now) == RT_BUTTON_TAP) {
            taps++;
        }
    }
    CHECK_EQ(taps, 0);

    CHECK_EQ(hold(&s, true, &now, 200), RT_BUTTON_NONE);
    CHECK_EQ(hold(&s, false, &now, 100), RT_BUTTON_TAP);
}

static void test_repeated_taps(void)
{
    rt_button_state_t s;
    uint32_t now = 0;
    rt_button_state_init(&s, now);

    for (int i = 0; i < 5; i++) {
        CHECK_EQ(hold(&s, true, &now, 200), RT_BUTTON_NONE);
        CHECK_EQ(hold(&s, false, &now, 200), RT_BUTTON_TAP);
    }
}

// The board can be up for weeks; a press spanning the 49-day rollover must still work.
static void test_millisecond_rollover(void)
{
    rt_button_state_t s;
    uint32_t now = 0xFFFFFF00u;
    rt_button_state_init(&s, now);

    CHECK_EQ(hold(&s, true, &now, RT_BUTTON_LONG_PRESS_MS + 100), RT_BUTTON_LONG);
    CHECK(now < 0x1000u);  // confirm the clock really did wrap during the press
}

int main(void)
{
    printf("test_rt_button\n");
    RUN(test_tap);
    RUN(test_long_press_fires_while_held_and_suppresses_the_tap);
    RUN(test_long_press_fires_once);
    RUN(test_bounce_is_debounced);
    RUN(test_repeated_taps);
    RUN(test_millisecond_rollover);
    TEST_MAIN_END();
}

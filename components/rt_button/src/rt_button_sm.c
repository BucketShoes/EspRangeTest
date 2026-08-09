// The button state machine, kept free of ESP-IDF so press timing can be tested on a host.

#include "rt_button.h"

#include <stddef.h>

void rt_button_state_init(rt_button_state_t *s, uint32_t now_ms)
{
    if (s == NULL) {
        return;
    }
    s->last_raw       = false;
    s->stable         = false;
    s->last_change_ms = now_ms;
    s->press_start_ms = 0;
    s->long_fired     = false;
}

rt_button_event_t rt_button_update(rt_button_state_t *s, bool raw_pressed, uint32_t now_ms)
{
    if (s == NULL) {
        return RT_BUTTON_NONE;
    }

    if (raw_pressed != s->last_raw) {
        s->last_raw       = raw_pressed;
        s->last_change_ms = now_ms;
        return RT_BUTTON_NONE;
    }

    // Unsigned subtraction, correct across the millisecond rollover.
    const bool settled = (now_ms - s->last_change_ms) >= RT_BUTTON_DEBOUNCE_MS;

    if (settled && raw_pressed != s->stable) {
        s->stable = raw_pressed;
        if (raw_pressed) {
            s->press_start_ms = now_ms;
            s->long_fired     = false;
        } else if (!s->long_fired) {
            // Released before the long-press threshold: that is a tap.
            return RT_BUTTON_TAP;
        }
        return RT_BUTTON_NONE;
    }

    // Fire the long press while the button is still held, rather than on release, so the
    // press is acknowledged the moment it qualifies.
    if (s->stable && !s->long_fired &&
        (now_ms - s->press_start_ms) >= RT_BUTTON_LONG_PRESS_MS) {
        s->long_fired = true;
        return RT_BUTTON_LONG;
    }

    return RT_BUTTON_NONE;
}

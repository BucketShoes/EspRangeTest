// GPIO9 mode button.
//
// GPIO9 is the BOOT/strapping pin on the C6 devkits: it is held high by an external
// pull-up and pulled to ground by the button, and is only sampled as a strapping pin at
// reset, so it is free to use as an ordinary input afterwards.
//
// Tap advances to the next mode; long press goes home to NORMAL. The long press matters
// more than it looks - it is the only way back on a board whose current mode has taken the
// UI link off the air.
//
// Polled rather than interrupt-driven: a button needs debouncing and press-duration
// timing either way, the sample rate is trivial, and keeping it off the ISR path means the
// mode change happens in ordinary task context where reconfiguring radios is safe.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define RT_BUTTON_GPIO          9
#define RT_BUTTON_DEBOUNCE_MS   30
#define RT_BUTTON_LONG_PRESS_MS 800

typedef enum {
    RT_BUTTON_NONE = 0,
    RT_BUTTON_TAP,
    RT_BUTTON_LONG,
} rt_button_event_t;

// Pure state machine, so press timing is host-testable without a GPIO.
// pressed: current debounced-input reading (true = button down).
typedef struct {
    bool     last_raw;
    bool     stable;
    uint32_t last_change_ms;
    uint32_t press_start_ms;
    bool     long_fired;  // a long press must not also emit a tap on release
} rt_button_state_t;

void              rt_button_state_init(rt_button_state_t *s, uint32_t now_ms);
rt_button_event_t rt_button_update(rt_button_state_t *s, bool raw_pressed, uint32_t now_ms);

// ESP-IDF side: configures GPIO9 and starts the polling task. The callback runs in that
// task's context.
typedef void (*rt_button_cb_t)(rt_button_event_t ev, void *ctx);
void rt_button_start(rt_button_cb_t cb, void *ctx);

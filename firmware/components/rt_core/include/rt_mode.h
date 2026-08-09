// Modes: operator-commanded radio isolation.
//
// There is no scheduler and no automatic cycling. A mode is locked in until it is changed
// from the UI or the button. The workflow this exists to serve is: command a switch, run a
// test, command another switch, run another test.
//
// The purpose is isolation - "does X work well on its own" versus "is Y the reason Z is
// bad". Every mode other than NORMAL is "just X", with an orthogonal keep-UI-BLE flag that
// leaves the phone link up at slowed intervals so the UI survives. That flag defaults on,
// because the intended method is to first establish whether BLE-on hurts X at all, and then
// do the real testing with BLE on.
//
// rt_mode_radios() is the single source of truth for what should be powered up. Radio
// drivers obey it rather than each deciding for themselves, which keeps the whole
// what-is-live question in one pure, testable function.

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RT_MODE_NORMAL = 0,  // everything on, fast intervals
    RT_MODE_JUST_BLE,
    RT_MODE_JUST_ESPNOW,
    RT_MODE_JUST_WIFI,
    RT_MODE_JUST_154,
    RT_MODE_JUST_FTM,
    RT_MODE_LR,          // Wi-Fi + ESP-NOW at LR; needs a Wi-Fi reinit, AP invisible to phones
    RT_MODE_COUNT
} rt_mode_t;

// Radios the current mode wants live.
#define RT_RADIO_BLE_ADV_CODED  (1u << 0)
#define RT_RADIO_BLE_ADV_LEGACY (1u << 1)  // Chrome discovery only, never a measurement
#define RT_RADIO_BLE_CONN       (1u << 2)
#define RT_RADIO_ESPNOW         (1u << 3)
#define RT_RADIO_WIFI_AP        (1u << 4)
#define RT_RADIO_WIFI_STA       (1u << 5)
#define RT_RADIO_IEEE802154     (1u << 6)
#define RT_RADIO_FTM            (1u << 7)
#define RT_RADIO_LR             (1u << 8)  // modifier: Wi-Fi/ESP-NOW run at LR, needs reinit

typedef uint16_t rt_radio_mask_t;

// Default auto-revert window. With keep-UI-BLE off, a board has no radio the phone can
// reach - and if that board is the one left behind at the far end, the GPIO9 button is out
// of walking range. Any UI contact refreshes the deadline, so this only ever fires on a
// board that has genuinely gone unreachable.
#define RT_MODE_AUTO_REVERT_MS_DEFAULT (10u * 60u * 1000u)

typedef struct {
    rt_mode_t mode;
    bool      keep_ui_ble;
    uint8_t   epoch;            // 4 bits on the wire; bumped on every mode change
    bool      auto_revert;
    uint32_t  auto_revert_ms;
    uint32_t  last_contact_ms;  // refreshed by UI contact and by any mode change
} rt_mode_state_t;

void rt_mode_init(rt_mode_state_t *s, uint32_t now_ms);

// Returns true if the mode actually changed, which is the caller's cue to bump radios and
// clear the observation table. Setting the same mode again is a no-op, so a UI that
// re-sends the current mode does not keep wiping the numbers.
bool rt_mode_set(rt_mode_state_t *s, rt_mode_t mode, uint32_t now_ms);

// Button: a tap advances to the next mode, a long press goes home to NORMAL.
bool rt_mode_next(rt_mode_state_t *s, uint32_t now_ms);
bool rt_mode_home(rt_mode_state_t *s, uint32_t now_ms);

bool rt_mode_set_keep_ui_ble(rt_mode_state_t *s, bool keep, uint32_t now_ms);

// Any contact from the UI. Refreshes the auto-revert deadline.
void rt_mode_touch(rt_mode_state_t *s, uint32_t now_ms);

// Call periodically. Returns true if it just auto-reverted to NORMAL.
bool rt_mode_tick(rt_mode_state_t *s, uint32_t now_ms);

rt_radio_mask_t rt_mode_radios(rt_mode_t mode, bool keep_ui_ble);

// True when switching between these two modes requires a full Wi-Fi stop/start rather than
// a live reconfigure. Merely having LR in the protocol list breaks the AP for phones, so LR
// is entered and left by reinit, never by a live protocol change.
bool rt_mode_needs_wifi_reinit(rt_mode_t from, rt_mode_t to);

const char *rt_mode_name(rt_mode_t mode);

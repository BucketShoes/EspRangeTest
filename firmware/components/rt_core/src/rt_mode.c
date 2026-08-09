#include "rt_mode.h"

#include <stddef.h>

static const char *const k_names[RT_MODE_COUNT] = {
    [RT_MODE_NORMAL]      = "normal",
    [RT_MODE_JUST_BLE]    = "just_ble",
    [RT_MODE_JUST_ESPNOW] = "just_espnow",
    [RT_MODE_JUST_WIFI]   = "just_wifi",
    [RT_MODE_JUST_154]    = "just_154",
    [RT_MODE_JUST_FTM]    = "just_ftm",
    [RT_MODE_LR]          = "lr",
};

const char *rt_mode_name(rt_mode_t mode)
{
    if ((unsigned)mode >= RT_MODE_COUNT || k_names[mode] == NULL) {
        return "?";
    }
    return k_names[mode];
}

// The UI link, when it is being kept alive during an isolation mode. Advertising and
// connection both stay up, but the caller runs them at slowed intervals - the point is to
// minimise the UI's airtime, not to pretend it is free. Legacy advertising rides along
// because without it Chrome cannot find the board at all.
#define RT_RADIO_UI_BLE (RT_RADIO_BLE_ADV_CODED | RT_RADIO_BLE_ADV_LEGACY | RT_RADIO_BLE_CONN)

rt_radio_mask_t rt_mode_radios(rt_mode_t mode, bool keep_ui_ble)
{
    rt_radio_mask_t m = 0;

    switch (mode) {
    case RT_MODE_NORMAL:
        m = RT_RADIO_BLE_ADV_CODED | RT_RADIO_BLE_ADV_LEGACY | RT_RADIO_BLE_CONN |
            RT_RADIO_ESPNOW | RT_RADIO_WIFI_AP | RT_RADIO_WIFI_STA | RT_RADIO_IEEE802154 |
            RT_RADIO_FTM;
        break;

    case RT_MODE_JUST_BLE:
        m = RT_RADIO_BLE_ADV_CODED | RT_RADIO_BLE_ADV_LEGACY | RT_RADIO_BLE_CONN;
        break;

    case RT_MODE_JUST_ESPNOW:
        // ESP-NOW needs the Wi-Fi driver up and parked on a fixed channel; the AP comes
        // with it and is what holds that channel.
        m = RT_RADIO_ESPNOW | RT_RADIO_WIFI_AP;
        break;

    case RT_MODE_JUST_WIFI:
        m = RT_RADIO_WIFI_AP | RT_RADIO_WIFI_STA;
        break;

    case RT_MODE_JUST_154:
        m = RT_RADIO_IEEE802154;
        break;

    case RT_MODE_JUST_FTM:
        // Responder lives on the AP, initiator on the STA - FTM needs both halves.
        m = RT_RADIO_WIFI_AP | RT_RADIO_WIFI_STA | RT_RADIO_FTM;
        break;

    case RT_MODE_LR:
        m = RT_RADIO_ESPNOW | RT_RADIO_WIFI_AP | RT_RADIO_WIFI_STA | RT_RADIO_LR;
        break;

    default:
        m = 0;
        break;
    }

    if (keep_ui_ble && mode != RT_MODE_NORMAL) {
        m |= RT_RADIO_UI_BLE;
    }

    return m;
}

bool rt_mode_needs_wifi_reinit(rt_mode_t from, rt_mode_t to)
{
    // Merely having LR in the protocol list breaks the AP for phones, so crossing the LR
    // boundary in either direction means a full esp_wifi_stop()/start() rather than a live
    // protocol change.
    const bool from_lr = (rt_mode_radios(from, false) & RT_RADIO_LR) != 0u;
    const bool to_lr   = (rt_mode_radios(to, false) & RT_RADIO_LR) != 0u;
    return from_lr != to_lr;
}

void rt_mode_init(rt_mode_state_t *s, uint32_t now_ms)
{
    if (s == NULL) {
        return;
    }
    s->mode            = RT_MODE_NORMAL;
    s->keep_ui_ble     = true;
    s->epoch           = 0;
    s->auto_revert     = true;
    s->auto_revert_ms  = RT_MODE_AUTO_REVERT_MS_DEFAULT;
    s->last_contact_ms = now_ms;
}

void rt_mode_touch(rt_mode_state_t *s, uint32_t now_ms)
{
    if (s == NULL) {
        return;
    }
    s->last_contact_ms = now_ms;
}

bool rt_mode_set(rt_mode_state_t *s, rt_mode_t mode, uint32_t now_ms)
{
    if (s == NULL || (unsigned)mode >= RT_MODE_COUNT) {
        return false;
    }
    // Re-sending the current mode must not wipe the numbers, so this is a genuine no-op.
    if (s->mode == mode) {
        s->last_contact_ms = now_ms;
        return false;
    }
    s->mode            = mode;
    s->epoch           = (uint8_t)((s->epoch + 1u) & 0x0Fu);
    s->last_contact_ms = now_ms;
    return true;
}

bool rt_mode_next(rt_mode_state_t *s, uint32_t now_ms)
{
    if (s == NULL) {
        return false;
    }
    const rt_mode_t next = (rt_mode_t)((s->mode + 1u) % RT_MODE_COUNT);
    return rt_mode_set(s, next, now_ms);
}

bool rt_mode_home(rt_mode_state_t *s, uint32_t now_ms)
{
    return rt_mode_set(s, RT_MODE_NORMAL, now_ms);
}

bool rt_mode_set_keep_ui_ble(rt_mode_state_t *s, bool keep, uint32_t now_ms)
{
    if (s == NULL) {
        return false;
    }
    s->last_contact_ms = now_ms;
    if (s->keep_ui_ble == keep) {
        return false;
    }
    s->keep_ui_ble = keep;
    // The set of live radios changed, so sequence continuity is broken just as surely as
    // by a mode change.
    s->epoch = (uint8_t)((s->epoch + 1u) & 0x0Fu);
    return true;
}

bool rt_mode_tick(rt_mode_state_t *s, uint32_t now_ms)
{
    if (s == NULL || !s->auto_revert || s->mode == RT_MODE_NORMAL) {
        return false;
    }
    // Unsigned subtraction keeps this correct across the millisecond rollover.
    if ((now_ms - s->last_contact_ms) < s->auto_revert_ms) {
        return false;
    }
    return rt_mode_home(s, now_ms);
}

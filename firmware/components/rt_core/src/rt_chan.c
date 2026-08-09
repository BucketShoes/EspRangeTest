#include "rt_chan.h"

#include <stddef.h>

static const char *const k_names[RT_CHAN_COUNT] = {
    [RT_CHAN_BLE_ADV_CODED]  = "ble_adv_coded",
    [RT_CHAN_BLE_CONN]       = "ble_conn",
    [RT_CHAN_ESPNOW]         = "espnow",
    [RT_CHAN_ESPNOW_LR]      = "espnow_lr",
    [RT_CHAN_WIFI_BEACON]    = "wifi_beacon",
    [RT_CHAN_WIFI_LR_BEACON] = "wifi_lr_beacon",
    [RT_CHAN_IEEE802154]     = "ieee802154",
    [RT_CHAN_FTM]            = "ftm",
};

// Starting cadences. Chosen to be slow enough that a channel is not itself the dominant
// airtime cost while others are being measured, and fast enough that a walk produces
// enough samples to fill a 20-deep window in well under a minute. Runtime-adjustable,
// because the right cadence is one of the things the rig exists to find out.
static uint32_t s_interval_ms[RT_CHAN_COUNT] = {
    [RT_CHAN_BLE_ADV_CODED]  = 500,
    [RT_CHAN_BLE_CONN]       = 0,  // driven by connection events, not a fixed cadence
    [RT_CHAN_ESPNOW]         = 250,
    [RT_CHAN_ESPNOW_LR]      = 250,
    [RT_CHAN_WIFI_BEACON]    = 100,  // ~1 beacon interval (100 TU)
    [RT_CHAN_WIFI_LR_BEACON] = 100,
    [RT_CHAN_IEEE802154]     = 250,
    [RT_CHAN_FTM]            = 0,  // bursts on request, no cadence to miss
};

const char *rt_chan_name(rt_chan_t chan)
{
    if ((unsigned)chan >= RT_CHAN_COUNT || k_names[chan] == NULL) {
        return "?";
    }
    return k_names[chan];
}

uint32_t rt_chan_expected_interval_ms(rt_chan_t chan)
{
    if ((unsigned)chan >= RT_CHAN_COUNT) {
        return 0;
    }
    return s_interval_ms[chan];
}

void rt_chan_set_expected_interval_ms(rt_chan_t chan, uint32_t interval_ms)
{
    if ((unsigned)chan >= RT_CHAN_COUNT) {
        return;
    }
    s_interval_ms[chan] = interval_ms;
}

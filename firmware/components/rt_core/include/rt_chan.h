// Channels: the axis everything in this project is keyed on.
//
// A "channel" here is a LINK KIND, not an RF channel. The whole point of the rig is to
// rank these against each other by how far each one still gets a nonzero amount of data
// through, so they are enumerated once, here, and every other module agrees on this list.
//
// Deliberately absent: BLE legacy advertising. It exists in the firmware purely so Chrome
// can discover and connect to a board, and is never a range measurement, so it gets no
// channel slot and never appears in a report.

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RT_CHAN_BLE_ADV_CODED = 0,  // the central experiment: coded adverts with no connection
    RT_CHAN_BLE_CONN,           // connection-oriented, PHY confirmed after renegotiation
    RT_CHAN_ESPNOW,
    RT_CHAN_ESPNOW_LR,
    RT_CHAN_WIFI_BEACON,
    RT_CHAN_WIFI_LR_BEACON,
    RT_CHAN_IEEE802154,
    RT_CHAN_FTM,
    RT_CHAN_COUNT
} rt_chan_t;

// Radios a build/chip actually has. C3 and S3 have no 802.15.4, so channel availability is
// gated rather than assumed; the UI must never offer a channel this board cannot do.
typedef uint16_t rt_chan_mask_t;

#define RT_CHAN_BIT(c) ((rt_chan_mask_t)1u << (c))

const char *rt_chan_name(rt_chan_t chan);

// Expected transmit interval for a channel, in milliseconds. Used to convert "time since
// last reception" into a count of presumed-missed packets, so a channel that has gone
// completely silent reads as loss rather than as an absence of data. 0 means the channel
// has no regular cadence (FTM, connection events) and no such inference is made.
uint32_t rt_chan_expected_interval_ms(rt_chan_t chan);
void     rt_chan_set_expected_interval_ms(rt_chan_t chan, uint32_t interval_ms);

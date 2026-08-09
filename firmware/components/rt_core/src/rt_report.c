#include "rt_report.h"

#include <stdio.h>

// Appends to buf, never past buf_len, and keeps *off pointing at the terminator.
// snprintf's return is "what it would have written", so it is clamped before use.
static void appendf(char *buf, size_t buf_len, size_t *off, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));

#include <stdarg.h>

static void appendf(char *buf, size_t buf_len, size_t *off, const char *fmt, ...)
{
    if (*off >= buf_len) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf + *off, buf_len - *off, fmt, ap);
    va_end(ap);

    if (n < 0) {
        return;
    }
    const size_t room = buf_len - *off - 1u;
    *off += ((size_t)n > room) ? room : (size_t)n;
}

size_t rt_report_render(const rt_obs_table_t *t, const rt_mode_state_t *mode,
                        uint8_t self_node_id, uint32_t now_ms, char *buf, size_t buf_len)
{
    if (buf == NULL || buf_len == 0u) {
        return 0;
    }
    buf[0] = '\0';
    size_t off = 0;

    appendf(buf, buf_len, &off,
            "=== node 0x%02X | mode=%s epoch=%u ui_ble=%s | up %lus ===\n",
            self_node_id,
            mode ? rt_mode_name(mode->mode) : "?",
            mode ? mode->epoch : 0u,
            (mode && mode->keep_ui_ble) ? "on" : "off",
            (unsigned long)(now_ms / 1000u));

    if (t == NULL || rt_obs_peer_count(t) == 0u) {
        appendf(buf, buf_len, &off, "  (no peers heard yet)\n");
        return off;
    }

    appendf(buf, buf_len, &off,
            "peer chan             last   mean   miss stale     age  rx  phy\n");

    for (int i = 0; i < RT_MAX_PEERS; i++) {
        if (!t->peers[i].used) {
            continue;
        }
        const uint8_t node = t->peers[i].node_id;

        for (int c = 0; c < RT_CHAN_COUNT; c++) {
            rt_link_summary_t s;
            rt_obs_summary(t, node, (rt_chan_t)c, now_ms, &s);
            if (!s.valid) {
                continue;  // never heard on this pair - not a zero, an absence
            }

            // Mean is fixed point (x10) and printed by hand rather than as a float: it
            // keeps the floating point printf formatter out of the firmware image
            // entirely, for a value that only ever needs one decimal place.
            const int mean_whole = s.rssi_mean_x10 / 10;
            int       mean_frac  = s.rssi_mean_x10 % 10;
            if (mean_frac < 0) {
                mean_frac = -mean_frac;
            }
            const char *mean_sign = (s.rssi_mean_x10 < 0 && mean_whole == 0) ? "-" : "";

            appendf(buf, buf_len, &off,
                    "0x%02X %-14s %5d %s%4d.%d %6u %5u %6lums %3u  %s\n",
                    node,
                    rt_chan_name((rt_chan_t)c),
                    s.rssi_last,
                    mean_sign, mean_whole, mean_frac,
                    s.missed_in_window,
                    s.stale_missed,
                    (unsigned long)s.age_ms,
                    s.rx_in_window,
                    rt_phy_name(s.rx_phy_last != RT_PHY_UNKNOWN ? s.rx_phy_last
                                                                : s.tx_phy_last));
        }
    }

    return off;
}

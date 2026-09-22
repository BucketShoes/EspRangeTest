// Optional GNSS: NMEA in on UART1, fixes out to the transmit path and the log.
//
// Wiring: the module's TX to GNSS_RX_GPIO, its RX to GNSS_TX_GPIO, both set just below. Only the
// first matters today - nothing is sent to the module - but the second is claimed for it so that
// asking the module for a faster fix rate later is a code change, not a rewiring.
//
// Nothing here is required. With no module fitted, the RX pin idles on its pull-up, no sentence
// ever validates, and the board behaves exactly as it did before this file existed.

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rt.h"

static const char *TAG = "gnss";

#define GNSS_UART    UART_NUM_1     // UART0 is the console, on GPIO16/17
#define GNSS_RX_GPIO 18             // <- module TX
#define GNSS_TX_GPIO 19             // -> module RX (unused so far)

// Baud rate is found, not configured. Modules ship at different defaults - 9600 for most u-blox
// and MTK parts, 38400 or 115200 for many sold for drones - and a wrong guess reads as "no
// module", which is the one failure here that sends someone to check their soldering. So each
// rate gets a dwell, and the first sentence with a valid checksum locks it in.
static const uint32_t BAUDS[] = { 9600, 38400, 115200, 57600, 4800, 19200 };
#define N_BAUDS (sizeof(BAUDS) / sizeof(BAUDS[0]))

// A module sends at least one sentence a second, several at 1Hz with default output, so 2.5s
// of nothing valid means this is not the rate.
#define DWELL_MS 2500
// Locked, but gone quiet this long: unplugged, or browned out and rebooted at another rate.
#define LOST_MS  5000
// No new fix this long - whether GGA says so or the module has gone quiet - and packets stop
// carrying a position. Until then they carry the tracked one (see Momentum in rt.h), which by
// now has coasted and eased back to the last real fix. Past this, a drone that has moved on is
// not where its last fix says, and saying nothing is more honest than saying that.
#define STALE_MS 10000

#define NMEA_MAX 100   // NMEA says 82; some modules exceed it with extra precision

static volatile uint8_t  s_state;
static volatile bool     s_had_fix;
static volatile uint8_t  s_sats;
static volatile uint8_t  s_hdop = 255;
static volatile uint32_t s_baud;          // 0 while hunting

static uint32_t s_last_nmea_ms;
static uint32_t s_fix_ms;
static uint32_t s_last_utc = UINT32_MAX;

// What packets carry between fixes; see Momentum in rt.h. Plain latest fix at boot.
static volatile uint8_t s_mode = RT_GEO_FIX;

void rt_gnss_set_mode(int mode)
{
    if (mode < 0 || mode >= RT_GEO_MODES || mode == s_mode) {
        return;
    }
    s_mode = (uint8_t)mode;
    ESP_LOGI(TAG, "packets carry: %s, from the next fix", rt_gnss_mode_name(mode));
}

const char *rt_gnss_mode_name(int mode)
{
    switch (mode) {
    case RT_GEO_SMOOTH:   return "smoothed";
    case RT_GEO_MOMENTUM: return "momentum";
    default:              return "fix";
    }
}

void rt_gnss_status(rt_gnss_st_t *out)
{
    out->state   = s_state;
    out->had_fix = s_had_fix;
    out->sats    = s_sats;
    out->hdop_ds = s_hdop;
    out->baud    = s_baud;
    out->mode    = s_mode;
    out->rx_gpio = GNSS_RX_GPIO;
}

bool rt_gnss_had_fix(void)
{
    return s_had_fix;
}

// ---- parsing ---------------------------------------------------------------------------------
//
// Integer only. The C6 has no FPU, and nothing here needs a float: degrees go straight to
// 1e-7 units, which is what the packet carries.

// "-123.4567" -> value x 10^scale, rounded. False on anything that is not a plain decimal.
static bool dec64(const char *s, int scale, int64_t *out)
{
    if (*s == '\0') {
        return false;
    }
    const bool neg = (*s == '-');
    if (*s == '-' || *s == '+') {
        s++;
    }
    int64_t v = 0;
    int     frac = -1;     // digits taken after the point, -1 before it
    int     next = -1;     // first digit past `scale`, for rounding
    for (; *s; s++) {
        if (*s == '.') {
            if (frac >= 0) {
                return false;
            }
            frac = 0;
            continue;
        }
        if (*s < '0' || *s > '9') {
            return false;
        }
        if (frac >= scale) {
            if (next < 0) {
                next = *s - '0';
            }
            continue;
        }
        v = v * 10 + (*s - '0');
        if (frac >= 0) {
            frac++;
        }
        if (v > 1000000000000000LL) {
            return false;
        }
    }
    for (int got = frac < 0 ? 0 : frac; got < scale; got++) {
        v *= 10;
    }
    if (next >= 5) {
        v++;
    }
    *out = neg ? -v : v;
    return true;
}

// "ddmm.mmmm" / "dddmm.mmmm" and a hemisphere letter -> degrees x 1e7.
static bool coord(const char *s, const char *hemi, int32_t *out)
{
    int64_t v;   // ddmm.mmmm x 1e7
    if (!dec64(s, 7, &v) || v < 0 || hemi[0] == '\0') {
        return false;
    }
    const int64_t deg = v / 1000000000LL;             // the digits before the minutes
    const int64_t min = v - deg * 1000000000LL;       // minutes x 1e7
    if (min >= 600000000LL) {
        return false;
    }
    int64_t e7 = deg * 10000000LL + (min + 30) / 60;
    if (hemi[0] == 'S' || hemi[0] == 'W') {
        e7 = -e7;
    }
    if (e7 > 1800000000LL || e7 < -1800000000LL) {
        return false;
    }
    *out = (int32_t)e7;
    return true;
}

// "hhmmss.ss" -> 0.1s since UTC midnight.
static bool utc(const char *s, uint32_t *out)
{
    for (int i = 0; i < 6; i++) {
        if (s[i] < '0' || s[i] > '9') {
            return false;
        }
    }
    const uint32_t hh = (s[0] - '0') * 10 + (s[1] - '0');
    const uint32_t mm = (s[2] - '0') * 10 + (s[3] - '0');
    const uint32_t ss = (s[4] - '0') * 10 + (s[5] - '0');
    uint32_t tenths = 0;
    if (s[6] == '.' && s[7] >= '0' && s[7] <= '9') {
        tenths = s[7] - '0';
    }
    if (hh > 23 || mm > 59 || ss > 60) {
        return false;
    }
    *out = ((hh * 60 + mm) * 60 + ss) * 10 + tenths;
    return true;
}

// ---- the track ------------------------------------------------------------------------------
//
// Velocity is an EMA of fix-to-fix motion with a 2s time constant, weighted by the actual time
// between fixes so a missed epoch counts for what it was. It is kept in floating point, relative
// to the first good fix since boot, so small motions are not lost to the size of the numbers;
// what goes out is integers (rt_geo_t), and everything downstream of that is exact.
//
// E, the other half of the track, is where the published track had got to when this fix
// arrived, minus the fix: so the track never jumps, and eases onto each fix instead.
#define TRACK_TAU_S  2.0f
#define TRACK_GAP_DS 100    // 10s: across a longer gap there is no velocity to speak of

static bool     s_ref_ok;
static int32_t  s_ref_lat, s_ref_lon;   // the first good fix: the origin of everything float
static bool     s_trk_ok;               // s_pub is a fix the track can continue from
static rt_geo_t s_pub;                  // the last fix published, exactly as published
static float    s_flat, s_flon;         // that fix, relative to s_ref
static float    s_vlat, s_vlon;         // 1e-7 degrees a second

static int16_t clamp16(float v)
{
    return (int16_t)(v > 32767.0f ? 32767 : v < -32767.0f ? -32767 : lroundf(v));
}

static void track(rt_geo_t *g)
{
    if (!s_ref_ok) {
        s_ref_lat = g->lat_e7;
        s_ref_lon = g->lon_e7;
        s_ref_ok  = true;
    }
    const float flat = (float)(g->lat_e7 - s_ref_lat);
    const float flon = (float)(g->lon_e7 - s_ref_lon);

    int32_t dt = s_trk_ok ? (int32_t)g->utc_ds - (int32_t)s_pub.utc_ds : 0;   // 0.1s
    if (dt < 0) {
        dt += 864000;   // midnight
    }
    if (s_trk_ok && dt > 0 && dt <= TRACK_GAP_DS) {
        const float dts = dt / 10.0f;
        const float a   = 1.0f - expf(-dts / TRACK_TAU_S);
        s_vlat += a * ((flat - s_flat) / dts - s_vlat);
        s_vlon += a * ((flon - s_flon) / dts - s_vlon);

        // Where the published track is now, on the same integer arithmetic packets use.
        int32_t tlat, tlon;
        rt_geo_at(&s_pub, (uint32_t)lroundf(dts * 4.0f), &tlat, &tlon);
        const int64_t elat = (int64_t)tlat - g->lat_e7;
        const int64_t elon = (int64_t)tlon - g->lon_e7;
        // A gap too big for the field is a jump, not an error to ease out: take the fix.
        if (elat >= -32767 && elat <= 32767 && elon >= -32767 && elon <= 32767) {
            g->elat = (int16_t)elat;
            g->elon = (int16_t)elon;
        }
    } else {
        s_vlat = s_vlon = 0.0f;
    }
    g->vlat = clamp16(s_vlat);
    g->vlon = clamp16(s_vlon);

    // The setting only decides what is published. Velocity is tracked whatever it is, so
    // switching to momentum mid-flight starts from a settled velocity, not from zero.
    if (s_mode != RT_GEO_MOMENTUM) {
        g->vlat = g->vlon = 0;
    }
    if (s_mode == RT_GEO_FIX) {
        g->elat = g->elon = 0;
    }
    s_flat  = flat;
    s_flon  = flon;
    s_pub   = *g;
    s_trk_ok = true;
}

static void fix_lost(const char *why)
{
    s_trk_ok = false;
    if (s_state == RT_GNSS_FIX) {
        s_state = RT_GNSS_NMEA;
        rt_geo_lost();
        ESP_LOGW(TAG, "fix lost (%s) - packets stop carrying a position", why);
    }
}

// Called with the time, from the reader's loop: a fix too old to use is withdrawn.
static void stale_check(uint32_t now)
{
    if (s_state == RT_GNSS_FIX && (int32_t)(now - s_fix_ms) > STALE_MS) {
        fix_lost("no new fix for 10s");
    }
}

// $xxGGA,hhmmss.ss,lat,N,lon,E,quality,sats,hdop,alt,M,...
//
// GGA alone carries everything wanted - position, altitude, quality, sats, hdop - and every
// module outputs it by default, whatever the constellation prefix (GP, GN, GL, BD, GA).
static void on_gga(char **f, int nf)
{
    if (nf < 10) {
        return;
    }
    s_sats = (uint8_t)atoi(f[7]);
    int64_t h;
    s_hdop = dec64(f[8], 1, &h) && h >= 0 ? (uint8_t)(h > 254 ? 254 : h) : 255;

    uint32_t t;
    if (!utc(f[1], &t)) {
        return;   // no time yet, so nothing to identify a fix by
    }
    rt_geo_t g = { 0 };
    const int q = atoi(f[6]);
    if (q == 0 || !coord(f[2], f[3], &g.lat_e7) || !coord(f[4], f[5], &g.lon_e7)) {
        // No fix this epoch. Packets go on carrying the track, which eases back to the last real
        // fix, until stale_check() says it has been too long.
        return;
    }
    if (t == s_last_utc) {
        return;   // the same epoch reported twice
    }
    s_last_utc = t;

    int64_t alt = 0;
    if (dec64(f[9], 0, &alt)) {
        g.alt_m = (int16_t)(alt > 32767 ? 32767 : alt < -32768 ? -32768 : alt);
    }
    g.utc_ds  = t;
    g.hdop_ds = s_hdop;
    g.sats    = s_sats;

    track(&g);

    if (s_state != RT_GNSS_FIX) {
        ESP_LOGI(TAG, "fix: %u sats - packets now carry a position", s_sats);
    }
    s_state   = RT_GNSS_FIX;
    s_had_fix = true;
    s_fix_ms  = rt_ms();
    rt_geo_publish(&g);
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// One line, '$' through the checksum, CR/LF already stripped. Returns whether it validated -
// which, and not whether it was a sentence we use, is what says the baud rate is right.
static bool on_line(char *s)
{
    char *star = strrchr(s, '*');
    if (s[0] != '$' || star == NULL || star - s < 6) {
        return false;
    }
    const int hi = hexval(star[1]), lo = hexval(star[2]);
    if (hi < 0 || lo < 0) {
        return false;
    }
    uint8_t x = 0;
    for (const char *p = s + 1; p < star; p++) {
        x ^= (uint8_t)*p;
    }
    if (x != (uint8_t)(hi << 4 | lo)) {
        return false;
    }
    *star = '\0';

    char *f[24];
    int   nf = 0;
    f[nf++] = s;
    for (char *p = s; *p && nf < 24; p++) {
        if (*p == ',') {
            *p = '\0';
            f[nf++] = p + 1;
        }
    }
    if (strlen(f[0]) == 6 && strcmp(f[0] + 3, "GGA") == 0) {
        on_gga(f, nf);
    }
    return true;
}

static void gnss_task(void *pv)
{
    (void)pv;
    uint8_t  buf[128];
    char     line[NMEA_MAX + 1];
    int      len   = 0;
    size_t   bi    = 0;
    bool     told  = false;
    uint32_t dwell = rt_ms();

    for (;;) {
        const int n = uart_read_bytes(GNSS_UART, buf, sizeof(buf), pdMS_TO_TICKS(100));
        const uint32_t now = rt_ms();

        for (int i = 0; i < n; i++) {
            const char c = (char)buf[i];
            if (c == '$') {
                len = 0;
            }
            if (c == '\r' || c == '\n') {
                if (len > 0) {
                    line[len] = '\0';
                    if (on_line(line)) {
                        s_last_nmea_ms = now;
                        if (s_baud == 0) {
                            s_baud = BAUDS[bi];
                            ESP_LOGI(TAG, "NMEA on GPIO%d at %lu baud", GNSS_RX_GPIO,
                                     (unsigned long)s_baud);
                        }
                        if (s_state == RT_GNSS_NONE) {
                            s_state = RT_GNSS_NMEA;
                        }
                    }
                }
                len = 0;
            } else if (len < NMEA_MAX) {
                line[len++] = c;
            } else {
                len = 0;   // overlong: garbage at the wrong rate, most likely
            }
        }

        if (s_baud == 0) {
            if ((int32_t)(now - dwell) > DWELL_MS) {
                bi = (bi + 1) % N_BAUDS;
                uart_set_baudrate(GNSS_UART, BAUDS[bi]);
                uart_flush_input(GNSS_UART);
                len   = 0;
                dwell = now;
                if (bi == 0 && !told) {
                    // Once, not every lap: with no module fitted this is the normal state.
                    ESP_LOGI(TAG, "no NMEA on GPIO%d at any rate - no module fitted?",
                             GNSS_RX_GPIO);
                    told = true;
                }
            }
        } else if ((int32_t)(now - s_last_nmea_ms) > LOST_MS) {
            fix_lost("NMEA stopped");
            ESP_LOGW(TAG, "NMEA stopped at %lu baud - searching again", (unsigned long)s_baud);
            s_baud  = 0;
            s_state = RT_GNSS_NONE;
            dwell   = now;
        }

        stale_check(now);
    }
}

void rt_gnss_start(void)
{
    const uart_config_t cfg = {
        .baud_rate  = (int)BAUDS[0],
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(GNSS_UART, 1024, 0, 0, NULL, 0);
    if (err == ESP_OK) err = uart_param_config(GNSS_UART, &cfg);
    if (err == ESP_OK) err = uart_set_pin(GNSS_UART, GNSS_TX_GPIO, GNSS_RX_GPIO,
                                          UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        // The rest of the board does not need this; say so and carry on without it.
        ESP_LOGE(TAG, "uart setup -> %s; GNSS disabled", esp_err_to_name(err));
        return;
    }
    // Idle high with nothing connected, so a board without a module reads silence rather than
    // whatever a floating pin picks up.
    gpio_pullup_en(GNSS_RX_GPIO);

    xTaskCreate(gnss_task, "gnss", 3584, NULL, 3, NULL);
    ESP_LOGI(TAG, "listening for NMEA on GPIO%d (module TX), GPIO%d to module RX",
             GNSS_RX_GPIO, GNSS_TX_GPIO);
}

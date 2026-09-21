// The packet log: a RAM ring of self-contained blocks, streamed to the phone on request. Format
// and reasoning are in rt.h, under "The packet log".

#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"

#include "rt.h"

static const char *TAG = "log";

#define MIN_BLOCKS    4

// A block closes when it is full, when its reference table is, or when it has been open this
// long. The last is only so the u16 TIME record can always reach; a block does not need to close
// for its contents to be sent - the open one streams live.
#define BLOCK_MAX_MS  60000

#define REFS_MAX      64    // the short records carry a 6-bit ref
#define NPOS_MAX      8

#define REC_TIME   0x01
#define REC_FIX    0x02
#define REC_NOFIX  0x03
#define REC_RX     0x04
#define REC_POS    0x05
#define REC_RXD    0x40
#define REC_RXS    0x80

#define LEN_TIME   3
#define LEN_FIX    34
#define LEN_NOFIX  14
#define LEN_RX     15
#define LEN_POS    10
#define LEN_RXD    8
#define LEN_RXS    5

// One lock for everything below: appenders run in the Wi-Fi task, the NimBLE host task and the
// 802.15.4 ISR, and the reader in the report task. Every hold is a few dozen bytes of copying.
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t  *s_ring;
static uint16_t *s_fill;       // bytes used, per slot
static uint32_t  s_nslots;
static uint32_t  s_newest;     // block number being written; slot = block % s_nslots
static uint32_t  s_held;       // blocks held, the open one included
static uint32_t  s_session;

// What the open block has said so far, so later records can refer back to it.
static struct {
    uint32_t clk;              // the clock the reader will have reconstructed so far
    uint32_t nref;
    struct {
        uint32_t node;
        uint32_t seq;          // last seq written on this ref
        uint8_t  ch;
        int8_t   ptx;
    } ref[REFS_MAX];
    uint32_t npos;
    struct {
        uint32_t node;
        rt_pos_t p;
    } pos[NPOS_MAX];           // each node's current POS, as the reader has it
} s_b;

static inline uint8_t *blk(void)
{
    return s_ring + (size_t)(s_newest % s_nslots) * RT_LOG_BLOCK;
}

static inline uint16_t *fill(void)
{
    return &s_fill[s_newest % s_nslots];
}

static void put(const void *src, int n)
{
    memcpy(blk() + *fill(), src, n);
    *fill() += n;
}

static void put8(uint8_t v)   { put(&v, 1); }
static void put16(uint16_t v) { put(&v, 2); }   // little-endian target, so these are the wire
static void put24(uint32_t v) { put(&v, 3); }
static void put32(uint32_t v) { put(&v, 4); }

static void open_block(uint32_t now, bool first)
{
    if (!first) {
        s_newest++;
        if (s_held < s_nslots) {
            s_held++;
        }
    }
    *fill() = 0;
    memset(&s_b, 0, sizeof(s_b));
    s_b.clk = now;
    put32(now);
}

// Make room for records totalling up to `need` bytes, plus a TIME record in front if one turns
// out to be necessary. Called before anything is written, so a record never straddles blocks.
static void room(uint32_t now, int need, bool new_ref)
{
    const bool full = *fill() + need + LEN_TIME > RT_LOG_BLOCK;
    const bool old  = (int32_t)(now - s_b.clk) > BLOCK_MAX_MS;
    if (full || old || (new_ref && s_b.nref >= REFS_MAX)) {
        open_block(now, false);
    }
}

// The dt byte for an event at `now`. Rounded against s_b.clk - the clock the reader will
// reconstruct - not against raw time, so rounding never accumulates.
static uint8_t tick(uint32_t now)
{
    int32_t d = (int32_t)(now - s_b.clk);
    if (d < 0) {
        d = 0;   // an appender on another task stamped `now` just before one that got here first
    }
    if (d > 255 * 4) {
        const uint16_t ms = (uint16_t)(d > 0xFFFF ? 0xFFFF : d);
        put8(REC_TIME);
        put16(ms);
        s_b.clk += ms;
        d -= ms;
    }
    const uint32_t q = ((uint32_t)d + 2) / 4;
    const uint8_t  dt = (uint8_t)(q > 255 ? 255 : q);
    s_b.clk += dt * 4u;
    return dt;
}

void rt_log_init(void)
{
    // One allocation, before anything else has touched the heap, so it is one clean block and
    // nothing the radios do later can fragment around it.
    const uint32_t n = RT_LOG_BYTES / RT_LOG_BLOCK;
    uint8_t  *ring = n >= MIN_BLOCKS ? heap_caps_malloc((size_t)n * RT_LOG_BLOCK, MALLOC_CAP_8BIT)
                                     : NULL;
    uint16_t *fl   = ring ? heap_caps_calloc(n, sizeof(uint16_t), MALLOC_CAP_8BIT) : NULL;
    if (ring == NULL || fl == NULL) {
        free(ring);
        // Not fatal: the board still measures and reports exactly as before, it just cannot
        // say where anything happened.
        ESP_LOGE(TAG, "no memory for a %d KB packet log - positions will not be recorded",
                 RT_LOG_BYTES / 1024);
        return;
    }

    s_session = esp_random() | 1;   // never 0: the page asks with 0 to mean "anything"
    portENTER_CRITICAL(&s_mux);
    s_fill   = fl;
    s_nslots = n;
    s_newest = 0;
    s_held   = 1;
    s_ring   = ring;
    open_block(rt_ms(), true);
    portEXIT_CRITICAL(&s_mux);

    ESP_LOGI(TAG, "%lu blocks of %d bytes (%d KB), session %08lX", (unsigned long)n,
             RT_LOG_BLOCK, RT_LOG_BYTES / 1024, (unsigned long)s_session);
}

void rt_log_fix(const rt_geo_t *g, const uint32_t seq_next[CH_COUNT])
{
    if (s_ring == NULL) {
        return;
    }
    const uint32_t now = rt_ms();
    portENTER_CRITICAL_SAFE(&s_mux);
    room(now, LEN_FIX, false);
    const uint8_t dt = tick(now);
    put8(REC_FIX);
    put8(dt);
    put32((uint32_t)g->lat_e7);
    put32((uint32_t)g->lon_e7);
    put16((uint16_t)g->alt_m);
    put16((uint16_t)g->elat);
    put16((uint16_t)g->elon);
    put16((uint16_t)g->vlat);
    put16((uint16_t)g->vlon);
    put8(g->hdop_ds);
    put8(g->sats);
    for (int c = 0; c < CH_COUNT; c++) {
        put32(seq_next[c]);
    }
    portEXIT_CRITICAL_SAFE(&s_mux);
}

void rt_log_nofix(const uint32_t seq_next[CH_COUNT])
{
    if (s_ring == NULL) {
        return;
    }
    const uint32_t now = rt_ms();
    portENTER_CRITICAL_SAFE(&s_mux);
    room(now, LEN_NOFIX, false);
    const uint8_t dt = tick(now);
    put8(REC_NOFIX);
    put8(dt);
    for (int c = 0; c < CH_COUNT; c++) {
        put32(seq_next[c]);
    }
    portEXIT_CRITICAL_SAFE(&s_mux);
}

// Wrap a difference of two 0..99999 fractions into -50000..49999, so a sender crossing a
// whole degree is a small step and not a jump of nearly a degree.
static int32_t wrapd(int32_t d)
{
    if (d >= RT_GEO_FRAC / 2)  d -= RT_GEO_FRAC;
    if (d < -RT_GEO_FRAC / 2)  d += RT_GEO_FRAC;
    return d;
}

void rt_log_rx(uint32_t node, int chan, int8_t ptx, uint32_t seq, uint32_t gap, int8_t rssi,
               uint8_t q, const rt_geo_wire_t *sender)
{
    if (s_ring == NULL) {
        return;
    }
    const uint32_t now = rt_ms();
    portENTER_CRITICAL_SAFE(&s_mux);

    // Worst case up front - a POS and a new ref - so nothing below can find the block full.
    room(now, LEN_POS + LEN_RX, true);

    // The ref for this link, if the block has one the reader can continue: same link, same
    // claimed power, and nothing heard on it since that went unlogged. The last match wins, as
    // that is the one whose seq the reader is tracking.
    int r = -1;
    for (int i = 0; i < (int)s_b.nref; i++) {
        if (s_b.ref[i].node == node && s_b.ref[i].ch == chan && s_b.ref[i].ptx == ptx) {
            r = i;
        }
    }
    const bool short_ok = r >= 0 && gap <= 255 && s_b.ref[r].seq + gap == seq;

    // Where the reader thinks this node is, and how far the packet says it has moved from there.
    // A moving sender moves a little every packet, so the usual case is a small delta on a
    // short record - 8 bytes a packet rather than a full POS and an RX.
    int      p = -1;
    bool     same = false, delta = false;
    int32_t  dlat = 0, dlon = 0, dalt = 0;
    rt_pos_t pos;
    if (sender != NULL) {
        rt_geo_unpack(sender->b, &pos);
        for (int i = 0; i < (int)s_b.npos; i++) {
            if (s_b.pos[i].node == node) {
                p = i;
            }
        }
        if (p >= 0) {
            dlat = wrapd(pos.latf - s_b.pos[p].p.latf);
            dlon = wrapd(pos.lonf - s_b.pos[p].p.lonf);
            dalt = pos.alt_m - s_b.pos[p].p.alt_m;
            same  = dlat == 0 && dlon == 0 && dalt == 0;
            delta = !same && short_ok
                 && dlat >= -128 && dlat <= 127 && dlon >= -128 && dlon <= 127
                 && dalt >= -128 && dalt <= 127;
        }
        if (!same && !delta) {
            // A full POS. With the table full the node is simply not remembered, and its next
            // packet writes a POS again: costs bytes, never correctness.
            if (p < 0 && s_b.npos < NPOS_MAX) {
                p = (int)s_b.npos++;
            }
            put8(REC_POS);
            put24(node);
            put(sender->b, 6);
        }
        if (p >= 0) {
            s_b.pos[p].node = node;
            s_b.pos[p].p    = pos;
        }
    }

    const bool    geo = sender != NULL;
    const uint8_t dt  = tick(now);

    if (delta) {
        put8((uint8_t)(REC_RXD | r));
        put8(dt);
        put8((uint8_t)gap);
        put8((uint8_t)rssi);
        put8(q);
        put8((uint8_t)dlat);
        put8((uint8_t)dlon);
        put8((uint8_t)dalt);
        s_b.ref[r].seq = seq;
    } else if (short_ok) {
        put8((uint8_t)(REC_RXS | (geo ? 0x40 : 0) | r));
        put8(dt);
        put8((uint8_t)gap);
        put8((uint8_t)rssi);
        put8(q);
        s_b.ref[r].seq = seq;
    } else {
        // room() already opened a new block if the ref table was full.
        r = (int)s_b.nref++;
        s_b.ref[r].node = node;
        s_b.ref[r].ch   = (uint8_t)chan;
        s_b.ref[r].ptx  = ptx;
        s_b.ref[r].seq  = seq;
        put8(REC_RX);
        put8(dt);
        put24(node);
        put8((uint8_t)(chan | (geo ? 0x80 : 0)));
        put8((uint8_t)ptx);
        put32(seq);
        put16((uint16_t)(gap > 0xFFFF ? 0 : gap));
        put8((uint8_t)rssi);
        put8(q);
    }
    portEXIT_CRITICAL_SAFE(&s_mux);
}

void rt_log_status(rt_log_st_t *out)
{
    memset(out, 0, sizeof(*out));
    out->bsize = RT_LOG_BLOCK;
    if (s_ring == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_mux);
    out->session = s_session;
    out->newest  = s_newest;
    out->oldest  = s_newest - (s_held - 1);
    out->cap     = (uint16_t)(s_nslots > 0xFFFF ? 0xFFFF : s_nslots);
    portEXIT_CRITICAL(&s_mux);
}

int rt_log_chunk(const rt_log_cur_t *c, uint8_t *out, int cap, rt_log_cur_t *adv)
{
    if (s_ring == NULL || cap <= RT_LOG_HDR) {
        return 0;
    }
    rt_log_cur_t cur = *c;

    portENTER_CRITICAL(&s_mux);
    const uint32_t oldest = s_newest - (s_held - 1);
    if (cur.block < oldest) {
        // Fell off the end of the ring while waiting. The page learns the same from log_oldest.
        cur.block = oldest;
        cur.off   = 0;
    }
    if (cur.block > s_newest) {
        portEXIT_CRITICAL(&s_mux);
        return 0;
    }
    const uint16_t f      = s_fill[cur.block % s_nslots];
    const bool     closed = cur.block != s_newest;
    if (cur.off > f) {
        cur.off = f;
    }
    int n = f - cur.off;
    if (n > cap - RT_LOG_HDR) {
        n = cap - RT_LOG_HDR;
    }
    if (n == 0 && !closed) {
        portEXIT_CRITICAL(&s_mux);
        return 0;
    }
    memcpy(out + RT_LOG_HDR, s_ring + (size_t)(cur.block % s_nslots) * RT_LOG_BLOCK + cur.off, n);
    portEXIT_CRITICAL(&s_mux);

    const bool end = closed && cur.off + n == f;
    out[0] = RT_LOG_TYPE;
    out[1] = end ? 0x01 : 0x00;
    memcpy(&out[2], &s_session, 4);
    memcpy(&out[6], &cur.block, 4);
    memcpy(&out[10], &cur.off, 2);

    if (end) {
        adv->block = cur.block + 1;
        adv->off   = 0;
    } else {
        adv->block = cur.block;
        adv->off   = (uint16_t)(cur.off + n);
    }
    return RT_LOG_HDR + n;
}

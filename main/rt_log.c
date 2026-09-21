// The packet log: a RAM ring of self-contained blocks, streamed to the phone on request. Format
// and reasoning are in rt.h, under "The packet log".

#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"

#include "rt.h"

static const char *TAG = "log";

// Up to this much, and never so much that less than HEAP_KEEP is left for everything else.
//
// HEAP_KEEP is not decoration. The Wi-Fi driver allocates its buffers on every esp_wifi_start()
// and a mode change restarts it; a phone connecting allocates too. A log that took the heap those
// need would turn "change mode" into "reboot", which is a lost run - so the log gets what is left
// after a generous margin, and says how much that was.
#define LOG_MAX_BYTES (128 * 1024)
#define HEAP_KEEP     (56 * 1024)
#define MIN_BLOCKS    4

// A block closes when it is full, when its reference table is, or when it has been open this
// long. The last is only so the u16 TIME record can always reach; a block does not need to close
// for its contents to be sent - the open one streams live.
#define BLOCK_MAX_MS  60000

#define REFS_MAX      64    // RXS carries a 6-bit ref
#define NFIX_MAX      8

#define REC_TIME   0x01
#define REC_FIX    0x02
#define REC_NOFIX  0x03
#define REC_RX     0x04
#define REC_PFIX   0x05
#define REC_RXS    0x80

#define LEN_TIME   3
#define LEN_FIX    29
#define LEN_NOFIX  14
#define LEN_RX     15
#define LEN_PFIX   18
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
    uint32_t nnf;
    struct {
        uint32_t node;
        uint32_t utc;
    } nf[NFIX_MAX];            // the PFIX this block last wrote for each node
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

// Make room for a record of up to `need` bytes, plus a TIME record in front of it if one turns
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

static void put_geo(const rt_geo_t *g)
{
    put32((uint32_t)g->lat_e7);
    put32((uint32_t)g->lon_e7);
    put16((uint16_t)g->alt_m);
    put24(g->utc_ds);
    put8(g->hdop_ds);
}

void rt_log_init(void)
{
    const size_t free_now = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    const size_t largest  = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

    size_t want = free_now > HEAP_KEEP ? free_now - HEAP_KEEP : 0;
    if (want > LOG_MAX_BYTES) want = LOG_MAX_BYTES;
    if (want > largest)       want = largest;
    uint32_t n = want / (RT_LOG_BLOCK + sizeof(uint16_t));

    uint8_t  *ring = NULL;
    uint16_t *fl   = NULL;
    while (n >= MIN_BLOCKS) {
        ring = heap_caps_malloc((size_t)n * RT_LOG_BLOCK, MALLOC_CAP_8BIT);
        fl   = heap_caps_calloc(n, sizeof(uint16_t), MALLOC_CAP_8BIT);
        if (ring && fl) {
            break;
        }
        free(ring);
        free(fl);
        ring = NULL;
        fl   = NULL;
        n    = n * 3 / 4;
    }
    if (ring == NULL) {
        // Not fatal: the board still measures and reports exactly as before, it just cannot
        // say where anything happened.
        ESP_LOGE(TAG, "no memory for a packet log (%u free) - positions will not be recorded",
                 (unsigned)free_now);
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

    ESP_LOGI(TAG, "%lu blocks of %d bytes (%lu KB), session %08lX, %u bytes of heap left",
             (unsigned long)n, RT_LOG_BLOCK, (unsigned long)(n * RT_LOG_BLOCK / 1024),
             (unsigned long)s_session, (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
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
    put_geo(g);
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

void rt_log_rx(uint32_t node, int chan, int8_t ptx, uint32_t seq, uint32_t gap, int8_t rssi,
               uint8_t q, const rt_geo_t *sender)
{
    if (s_ring == NULL) {
        return;
    }
    const uint32_t now = rt_ms();
    portENTER_CRITICAL_SAFE(&s_mux);

    // Worst case up front - a PFIX and a new ref - so nothing below can find the block full.
    room(now, LEN_PFIX + LEN_RX, true);

    if (sender != NULL) {
        int i = 0;
        while (i < (int)s_b.nnf && s_b.nf[i].node != node) {
            i++;
        }
        if (i == (int)s_b.nnf || s_b.nf[i].utc != sender->utc_ds) {
            put8(REC_PFIX);
            put24(node);
            put_geo(sender);
            if (i == (int)s_b.nnf && s_b.nnf < NFIX_MAX) {
                s_b.nnf++;
            }
            // With the table full the entry is simply not remembered, and the next packet
            // from this node writes its PFIX again. Costs bytes, never correctness.
            if (i < NFIX_MAX) {
                s_b.nf[i].node = node;
                s_b.nf[i].utc  = sender->utc_ds;
            }
        }
    }

    // A short record only when the reader can reconstruct the sequence number exactly: same
    // link, same claimed power, and nothing heard on it in between that was not logged. Anything
    // else is a full record, which also starts a fresh ref.
    int r = -1;
    for (int i = 0; i < (int)s_b.nref; i++) {
        if (s_b.ref[i].node == node && s_b.ref[i].ch == chan && s_b.ref[i].ptx == ptx) {
            r = i;
        }
    }
    const bool geo = sender != NULL;
    const uint8_t dt = tick(now);

    if (r >= 0 && gap >= 1 && gap <= 255 && s_b.ref[r].seq + gap == seq) {
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

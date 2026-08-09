#include <string.h>

#include "rt_payload.h"
#include "test_util.h"

static void test_round_trip(void)
{
    const rt_payload_t in = {
        .ver     = RT_PAYLOAD_VER,
        .epoch   = 0xD,
        .node_id = 0x2A,
        .seq     = 0xBEEF,
        .tx_phy  = RT_PHY_BLE_CODED_S8,
        .tx_dbm  = -4,
    };

    uint8_t buf[RT_PAYLOAD_SIZE];
    CHECK_EQ(rt_payload_pack(&in, buf, sizeof(buf)), RT_PAYLOAD_SIZE);

    rt_payload_t out;
    CHECK(rt_payload_unpack(buf, sizeof(buf), &out));
    CHECK_EQ(out.ver, in.ver);
    CHECK_EQ(out.epoch, in.epoch);
    CHECK_EQ(out.node_id, in.node_id);
    CHECK_EQ(out.seq, in.seq);
    CHECK_EQ(out.tx_phy, in.tx_phy);
    CHECK_EQ(out.tx_dbm, in.tx_dbm);
}

// The 9-byte budget is the reason the payload is shaped the way it is - a legacy BLE
// advert has ~26 usable bytes of manufacturer data. If this ever grows, that is a decision
// to be made deliberately, not discovered in the field.
static void test_wire_size_and_layout(void)
{
    const rt_payload_t in = {
        .ver = 1, .epoch = 0, .node_id = 0, .seq = 1, .tx_phy = 0, .tx_dbm = 0,
    };
    uint8_t buf[16];
    memset(buf, 0xAA, sizeof(buf));

    CHECK_EQ(rt_payload_pack(&in, buf, sizeof(buf)), 9);
    CHECK_EQ(buf[0], 0x54);  // magic low byte  ('T')
    CHECK_EQ(buf[1], 0x52);  // magic high byte ('R')
    CHECK_EQ(buf[9], 0xAA);  // nothing written past 9 bytes
}

static void test_negative_tx_power_survives(void)
{
    rt_payload_t in = { .ver = 1, .epoch = 0, .node_id = 7, .seq = 5, .tx_phy = 0 };
    uint8_t buf[RT_PAYLOAD_SIZE];
    rt_payload_t out;

    const int8_t powers[] = { -128, -40, -1, 0, 1, 20, 127 };
    for (unsigned i = 0; i < sizeof(powers) / sizeof(powers[0]); i++) {
        in.tx_dbm = powers[i];
        rt_payload_pack(&in, buf, sizeof(buf));
        CHECK(rt_payload_unpack(buf, sizeof(buf), &out));
        CHECK_EQ(out.tx_dbm, powers[i]);
    }
}

// ver and epoch share a byte; neither may bleed into the other.
static void test_nibble_packing_is_isolated(void)
{
    uint8_t buf[RT_PAYLOAD_SIZE];
    rt_payload_t out;

    for (uint8_t epoch = 0; epoch < 16; epoch++) {
        const rt_payload_t in = {
            .ver = RT_PAYLOAD_VER, .epoch = epoch, .node_id = 1, .seq = 0,
            .tx_phy = 0, .tx_dbm = 0,
        };
        rt_payload_pack(&in, buf, sizeof(buf));
        CHECK(rt_payload_unpack(buf, sizeof(buf), &out));
        CHECK_EQ(out.epoch, epoch);
        CHECK_EQ(out.ver, RT_PAYLOAD_VER);
    }
}

// On a shared broadcast medium, "not one of ours" is the common case, not an error.
static void test_rejects_foreign_and_short_frames(void)
{
    const rt_payload_t in = {
        .ver = RT_PAYLOAD_VER, .epoch = 0, .node_id = 1, .seq = 1, .tx_phy = 0, .tx_dbm = 0,
    };
    uint8_t buf[RT_PAYLOAD_SIZE];
    rt_payload_pack(&in, buf, sizeof(buf));

    rt_payload_t out;

    CHECK(!rt_payload_unpack(buf, RT_PAYLOAD_SIZE - 1, &out));  // truncated

    uint8_t bad_magic[RT_PAYLOAD_SIZE];
    memcpy(bad_magic, buf, sizeof(buf));
    bad_magic[0] ^= 0xFF;
    CHECK(!rt_payload_unpack(bad_magic, sizeof(bad_magic), &out));

    uint8_t bad_ver[RT_PAYLOAD_SIZE];
    memcpy(bad_ver, buf, sizeof(buf));
    bad_ver[2] = (uint8_t)((bad_ver[2] & 0xF0u) | 0x0Eu);  // version 14
    CHECK(!rt_payload_unpack(bad_ver, sizeof(bad_ver), &out));

    CHECK_EQ(rt_payload_pack(&in, buf, RT_PAYLOAD_SIZE - 1), 0);  // refuses to overflow
}

int main(void)
{
    printf("test_rt_payload\n");
    RUN(test_round_trip);
    RUN(test_wire_size_and_layout);
    RUN(test_negative_tx_power_survives);
    RUN(test_nibble_packing_is_isolated);
    RUN(test_rejects_foreign_and_short_frames);
    TEST_MAIN_END();
}

#include "rt_payload.h"

// Wire layout, little-endian, 9 bytes total:
//   [0..1] magic       uint16
//   [2]    ver:4 | epoch:4   (ver in the low nibble)
//   [3]    node_id     uint8
//   [4..5] seq         uint16
//   [6]    tx_phy      uint8
//   [7]    tx_dbm      int8
//   [8]    reserved    uint8  (zero; keeps the struct a round 9 and leaves one byte of
//                              headroom to add a field without a version bump)

static const char *const k_phy_names[RT_PHY_COUNT] = {
    [RT_PHY_UNKNOWN]      = "unknown",
    [RT_PHY_BLE_1M]       = "ble_1M",
    [RT_PHY_BLE_2M]       = "ble_2M",
    [RT_PHY_BLE_CODED_S2] = "ble_coded_s2",
    [RT_PHY_BLE_CODED_S8] = "ble_coded_s8",
    [RT_PHY_WIFI_11B]     = "wifi_11b",
    [RT_PHY_WIFI_11G]     = "wifi_11g",
    [RT_PHY_WIFI_11N]     = "wifi_11n",
    [RT_PHY_WIFI_LR]      = "wifi_lr",
    [RT_PHY_154_OQPSK]    = "154_oqpsk",
};

const char *rt_phy_name(uint8_t phy)
{
    if (phy >= RT_PHY_COUNT || k_phy_names[phy] == NULL) {
        return "?";
    }
    return k_phy_names[phy];
}

size_t rt_payload_pack(const rt_payload_t *in, uint8_t *out, size_t out_len)
{
    if (in == NULL || out == NULL || out_len < RT_PAYLOAD_SIZE) {
        return 0;
    }

    out[0] = (uint8_t)(RT_PAYLOAD_MAGIC & 0xFFu);
    out[1] = (uint8_t)((RT_PAYLOAD_MAGIC >> 8) & 0xFFu);
    out[2] = (uint8_t)((in->ver & 0x0Fu) | ((in->epoch & 0x0Fu) << 4));
    out[3] = in->node_id;
    out[4] = (uint8_t)(in->seq & 0xFFu);
    out[5] = (uint8_t)((in->seq >> 8) & 0xFFu);
    out[6] = in->tx_phy;
    out[7] = (uint8_t)in->tx_dbm;
    out[8] = 0;

    return RT_PAYLOAD_SIZE;
}

bool rt_payload_unpack(const uint8_t *in, size_t in_len, rt_payload_t *out)
{
    if (in == NULL || out == NULL || in_len < RT_PAYLOAD_SIZE) {
        return false;
    }

    const uint16_t magic = (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
    if (magic != RT_PAYLOAD_MAGIC) {
        return false;
    }

    const uint8_t ver = (uint8_t)(in[2] & 0x0Fu);
    if (ver != RT_PAYLOAD_VER) {
        return false;
    }

    out->ver     = ver;
    out->epoch   = (uint8_t)((in[2] >> 4) & 0x0Fu);
    out->node_id = in[3];
    out->seq     = (uint16_t)((uint16_t)in[4] | ((uint16_t)in[5] << 8));
    out->tx_phy  = in[6];
    out->tx_dbm  = (int8_t)in[7];

    return true;
}

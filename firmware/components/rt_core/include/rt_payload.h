// The 9-byte payload every channel transmits.
//
// Airtime is the scarce resource at range, so this carries only what the receiver cannot
// work out for itself:
//
//   - which channel it arrived on   -> implied by which receive callback fired
//   - who sent it                   -> a source address exists on every path, but a 1-byte
//                                      node id beats reconciling three address spaces
//   - the receiver's contention     -> known locally; a TX-side copy would be meaningless
//                                      anyway, since a packet that arrived was obviously
//                                      transmitted successfully
//
// What genuinely cannot be inferred, and is therefore here: the sequence number, the
// transmit power, and the advertising coding (the C6 is BT 5.3, so Advertising Coding
// Selection - the 5.4 feature that would let a receiver read S=2 vs S=8 out of the
// extended advertising report - is likely unavailable to us).
//
// No CRC: BLE, 802.15.4 and Wi-Fi each carry their own FCS and drop corrupt frames before
// we ever see them. Ours would cost airtime and catch nothing.
//
// Packed explicitly byte by byte, little-endian, rather than by memcpy of a struct, so the
// wire format cannot drift with compiler padding or host endianness. The host tests
// exercise the same code the firmware uses.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RT_PAYLOAD_SIZE  9
#define RT_PAYLOAD_MAGIC 0x5254u  // "RT" - rejects foreign frames sharing a broadcast path
#define RT_PAYLOAD_VER   1u

// PHY / coding claimed by the sender. On BLE this is the only S=2 vs S=8 signal we may
// get; everywhere else it is a cross-check against what the receiver independently reports.
typedef enum {
    RT_PHY_UNKNOWN = 0,
    RT_PHY_BLE_1M,
    RT_PHY_BLE_2M,
    RT_PHY_BLE_CODED_S2,
    RT_PHY_BLE_CODED_S8,
    RT_PHY_WIFI_11B,
    RT_PHY_WIFI_11G,
    RT_PHY_WIFI_11N,
    RT_PHY_WIFI_LR,
    RT_PHY_154_OQPSK,
    RT_PHY_COUNT
} rt_phy_t;

const char *rt_phy_name(uint8_t phy);

typedef struct {
    uint8_t  ver;      // 4 bits on the wire
    uint8_t  epoch;    // 4 bits on the wire; bumped on every mode change
    uint8_t  node_id;
    uint16_t seq;      // per (node, channel); the window is 20 deep, so 16 bits is ample
    uint8_t  tx_phy;   // rt_phy_t
    int8_t   tx_dbm;
} rt_payload_t;

// Returns RT_PAYLOAD_SIZE on success, 0 if out_len is too small.
size_t rt_payload_pack(const rt_payload_t *in, uint8_t *out, size_t out_len);

// Returns false if the buffer is too short, the magic does not match, or the version is
// one we do not understand - all three mean "not one of ours", which on a shared broadcast
// medium is the common case rather than an error.
bool rt_payload_unpack(const uint8_t *in, size_t in_len, rt_payload_t *out);

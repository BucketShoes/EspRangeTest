#pragma once
#include <stdint.h>

// All structs here are sent over the wire (ESP-NOW payload and BLE notify payload),
// so they're kept fixed-size and packed.

struct __attribute__((packed)) LinkStat {
  uint16_t rxCount;     // packets/samples received in the last reporting window
  uint8_t pdrPercent;   // 0-100 delivery ratio for the window; 0xFF = not applicable/no data
  int8_t rssiAvg;
  int8_t rssiMin;
  int8_t rssiMax;
  uint8_t mode;  // link-specific: BLE => negotiated PHY (1=1M,2=2M,3=Coded); ESP-NOW => 1 if LR mode
};

struct __attribute__((packed)) BoardSnapshot {
  uint32_t boardId;
  uint32_t uptimeMs;
  LinkStat espnow;    // this board's RX from peer over ESP-NOW
  LinkStat blePeer;   // this board's RX from peer over BLE
  LinkStat blePhone;  // this board's RX from phone over BLE (only meaningful if phoneConnected)
  uint8_t phoneConnected;
  int8_t txPowerDbm;
};

struct __attribute__((packed)) EspNowPacket {
  uint32_t magic;
  uint32_t boardId;
  uint32_t txSeq;
  BoardSnapshot snap;
};

// Accumulates rx samples over a rolling window; snapshot() reads out a LinkStat and
// resets the rssi/rxCount window (call once per reporting period, e.g. every second).
class RollingLink {
 public:
  // expectedIntervalMs: the link's known send interval (e.g. ESPNOW_TX_PERIOD_MS), used
  // to compute how many packets a completed window *should* have seen. 0 (default)
  // means this link has no PDR concept - pdrPercent always reads n/a (this is what BLE
  // links use, since RSSI there comes from connection polling, not a sequenced stream).
  explicit RollingLink(uint32_t expectedIntervalMs = 0) : m_expectedIntervalMs(expectedIntervalMs) {}
  void onRx(int8_t rssi, uint8_t mode);
  void onRxSeq(int8_t rssi, uint8_t mode, uint32_t seq);  // for links with a sender sequence number (loss detection)
  LinkStat snapshot();

 private:
  uint32_t m_rxCount = 0;
  int32_t m_rssiSum = 0;
  int8_t m_rssiMin = 127;
  int8_t m_rssiMax = -128;
  uint8_t m_mode = 0;

  // PDR mechanism history, kept factual (no ranking of versions) so this doesn't get
  // misread as settled requirements:
  //   v1: reset every ~500ms reporting tick.
  //   v2: separate ~5s reset window, decoupled from the rssi/rxCount tick. A window
  //     that had received nothing yet since its reset reported n/a rather than 0%.
  //   v3: replaced the periodic reset with decaying (halving) the accumulated counts
  //     on the same cadence, instead of zeroing them.
  //   v4 (current): kept the periodic reset from v2, but computes "expected" from
  //     elapsed wall-clock time rather than gaps between received packets, so a window
  //     with zero arrivals computes a real 0% instead of n/a.
  // Requested and not yet implemented: a continuously rolling window with no discrete
  // reset boundary (as opposed to v2/v4's periodic reset, or v3's decay). Two properties
  // worth naming precisely: a periodic hard reset (v2, v4) produces a step change in the
  // reported number at the reset instant that does not correspond to any change in the
  // underlying signal; decaying/smoothing (v3) reduces how quickly a real change in the
  // signal shows up in the reported number.
  uint32_t m_expectedIntervalMs;
  bool m_haveSeq = false;
  uint32_t m_pdrWindowStartMs = 0;
  uint32_t m_pdrRxCount = 0;
};

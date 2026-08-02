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
// resets the window (call once per reporting period, e.g. every second).
class RollingLink {
 public:
  void onRx(int8_t rssi, uint8_t mode);
  void onRxSeq(int8_t rssi, uint8_t mode, uint32_t seq);  // for links with a sender sequence number (loss detection)
  LinkStat snapshot();

 private:
  uint32_t m_rxCount = 0;
  int32_t m_rssiSum = 0;
  int8_t m_rssiMin = 127;
  int8_t m_rssiMax = -128;
  uint8_t m_mode = 0;

  // PDR is tracked over a longer, separately-reset accumulation than the rssi/rxCount
  // window above. At ~2 packets per reporting window, a per-window PDR could only ever
  // read 0%, 50%, or 100% - too coarse to mean anything. Accumulating over several
  // reporting periods before resetting gives enough packets for real percentages while
  // still staying responsive to changing conditions over the course of a walk.
  bool m_haveSeq = false;
  bool m_pdrWindowStarted = false;
  uint32_t m_pdrSeqStart = 0;
  uint32_t m_pdrSeqLast = 0;
  uint32_t m_pdrRxCount = 0;
  uint8_t m_snapshotsSincePdrReset = 0;
  static const uint8_t kPdrResetEveryNSnapshots = 10;
};

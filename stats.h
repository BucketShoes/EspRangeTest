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

  // PDR history: this went through several wrong iterations - noted here so nobody
  // (including future-me) mistakes the current shape for a real requirement.
  //   v1 (original): reset every ~500ms reporting tick. Coarse (basically 0/50/100%
  //     given ~2 packets/tick) but this was the actual best version - simple, no
  //     surprises, good enough.
  //   v2: separate longer reset window (~5s) decoupled from the rssi/rxCount tick.
  //     Introduced a bug where a freshly-reset window with nothing received *yet*
  //     reported n/a instead of 0%. Still usable overall despite the bug, but the hard
  //     reset itself - not the n/a bug - is the part that made it worse than v1: it
  //     injects visible jumps in the number that are just an artifact of when the reset
  //     happened, unrelated to the actual signal. That's real noise, not a feature.
  //   v3 (wrong "fix"): tried decaying/halving the accumulated counts instead of
  //     resetting. This was worse still - smoothing directly costs responsiveness,
  //     which matters for quickly A/B-ing two hand positions.
  //   v4 (current, also wrong): "fixed" the v2 n/a bug by keeping a hard reset but
  //     computing "expected" from elapsed wall-clock time instead of received-packet
  //     gaps. This does make a fully-empty window read as a real 0% instead of n/a, but
  //     it's still a HARD RESET, which is exactly the part identified above as the
  //     actual problem with v2. Do not read this as "hard reset is desired" - it isn't.
  //   What's actually wanted: a genuinely rolling window (continuously slides forward,
  //   no discrete reset boundary at all) rather than a periodically-resetting one - so
  //   old data ages out smoothly by naturally falling outside the window as time moves
  //   on, without ever producing a visible jump that has nothing to do with the signal.
  //   Not yet implemented - flag before changing this again.
  uint32_t m_expectedIntervalMs;
  bool m_haveSeq = false;
  uint32_t m_pdrWindowStartMs = 0;
  uint32_t m_pdrRxCount = 0;
};

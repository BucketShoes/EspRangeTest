#include "stats.h"

void RollingLink::onRx(int8_t rssi, uint8_t mode) {
  m_rxCount++;
  m_rssiSum += rssi;
  if (rssi < m_rssiMin) m_rssiMin = rssi;
  if (rssi > m_rssiMax) m_rssiMax = rssi;
  m_mode = mode;
}

void RollingLink::onRxSeq(int8_t rssi, uint8_t mode, uint32_t seq) {
  onRx(rssi, mode);
  if (!m_haveSeq) {
    m_haveSeq = true;
    m_pdrExpectedTotal = 1;
    m_pdrReceivedTotal = 1;
  } else {
    // Gap since the immediately previous packet: 1 = none missed, >1 = some missed.
    // A non-positive gap (reorder/duplicate) can't tell us anything about loss, so
    // just count this packet as expected-and-received rather than corrupting the ratio.
    uint32_t gap = (seq > m_pdrLastSeq) ? (seq - m_pdrLastSeq) : 1;
    m_pdrExpectedTotal += gap;
    m_pdrReceivedTotal += 1;
  }
  m_pdrLastSeq = seq;
}

LinkStat RollingLink::snapshot() {
  LinkStat s{};
  s.rxCount = (uint16_t)m_rxCount;

  if (m_rxCount == 0) {
    s.rssiAvg = 0;
    s.rssiMin = 0;
    s.rssiMax = 0;
  } else {
    s.rssiAvg = (int8_t)(m_rssiSum / (int32_t)m_rxCount);
    s.rssiMin = m_rssiMin;
    s.rssiMax = m_rssiMax;
  }
  s.mode = m_mode;

  if (m_haveSeq) {
    uint32_t pct = (m_pdrReceivedTotal * 100UL) / m_pdrExpectedTotal;
    s.pdrPercent = (uint8_t)(pct > 100 ? 100 : pct);
  } else {
    s.pdrPercent = 0xFF;  // nothing has ever arrived from this link
  }

  m_rxCount = 0;
  m_rssiSum = 0;
  m_rssiMin = 127;
  m_rssiMax = -128;

  // Decay (halve) the accumulated totals periodically so old history fades out and the
  // ratio stays responsive to current conditions - without ever fully clearing back to
  // "nothing yet", which is what caused the misleading n/a mid-session.
  m_snapshotsSincePdrDecay++;
  if (m_snapshotsSincePdrDecay >= kPdrDecayEveryNSnapshots) {
    m_snapshotsSincePdrDecay = 0;
    if (m_haveSeq) {
      m_pdrExpectedTotal = (m_pdrExpectedTotal + 1) / 2;
      m_pdrReceivedTotal = (m_pdrReceivedTotal + 1) / 2;
    }
  }

  return s;
}

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
  m_haveSeq = true;
  if (!m_windowSeqStarted) {
    m_seqWindowStart = seq;
    m_windowSeqStarted = true;
  }
  m_seqLast = seq;
}

LinkStat RollingLink::snapshot() {
  LinkStat s{};
  s.rxCount = (uint16_t)m_rxCount;

  if (m_rxCount == 0) {
    s.pdrPercent = 0xFF;
    s.rssiAvg = 0;
    s.rssiMin = 0;
    s.rssiMax = 0;
    s.mode = m_mode;
  } else {
    s.rssiAvg = (int8_t)(m_rssiSum / (int32_t)m_rxCount);
    s.rssiMin = m_rssiMin;
    s.rssiMax = m_rssiMax;
    s.mode = m_mode;

    if (m_haveSeq && m_windowSeqStarted) {
      uint32_t expected = (m_seqLast - m_seqWindowStart) + 1;
      if (expected < m_rxCount) expected = m_rxCount;  // guard against reorder/wrap
      uint32_t pct = (m_rxCount * 100UL) / expected;
      s.pdrPercent = (uint8_t)(pct > 100 ? 100 : pct);
    } else {
      s.pdrPercent = 0xFF;
    }
  }

  m_rxCount = 0;
  m_rssiSum = 0;
  m_rssiMin = 127;
  m_rssiMax = -128;
  m_windowSeqStarted = false;
  return s;
}

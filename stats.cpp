#include "stats.h"
#include "config.h"
#include <Arduino.h>

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
    m_pdrWindowStartMs = millis();
  }
  m_pdrRxCount++;
  (void)seq;  // loss is inferred from elapsed time vs the known send interval, not seq gaps
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

  if (m_expectedIntervalMs > 0 && m_haveSeq) {
    uint32_t elapsed = millis() - m_pdrWindowStartMs;
    uint32_t expected = elapsed / m_expectedIntervalMs;
    if (expected < 1) expected = 1;
    uint32_t received = m_pdrRxCount;
    if (received > expected) expected = received;  // guard against clock jitter
    uint32_t pct = (received * 100UL) / expected;
    s.pdrPercent = (uint8_t)(pct > 100 ? 100 : pct);
  } else {
    s.pdrPercent = 0xFF;  // nothing has ever arrived from this link, ever
  }

  m_rxCount = 0;
  m_rssiSum = 0;
  m_rssiMin = 127;
  m_rssiMax = -128;

  // Periodic reset - see the PDR history note in stats.h. Produces a step change in the
  // reported number at the reset instant, independent of the underlying signal.
  if (m_haveSeq && (millis() - m_pdrWindowStartMs) >= PDR_WINDOW_MS) {
    m_pdrWindowStartMs = millis();
    m_pdrRxCount = 0;
  }

  return s;
}

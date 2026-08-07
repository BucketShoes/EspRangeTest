#include "espnow_link.h"
#include "identity.h"
#include "config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

static RollingLink s_rxFromPeer(ESPNOW_TX_PERIOD_MS);
static uint8_t s_peerMac[6] = {0};
static bool s_havePeer = false;
static uint32_t s_txSeq = 0;
static BoardSnapshot s_outgoing{};
static BoardSnapshot s_lastPeerSnapshot{};
static const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static void addPeerIfNeeded(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;  // use current channel of ifidx below
  p.ifidx = WIFI_IF_STA;
  p.encrypt = false;
  esp_now_add_peer(&p);

  // esp_wifi_set_protocol(..., WIFI_PROTOCOL_LR) only makes the LR PHY available on the
  // interface - it does not select it as the rate actually used for transmission. That
  // has to be set per peer. On WiFi 6/HE chips (the C6 is one), the older interface-wide
  // esp_wifi_config_espnow_rate() doesn't work at all (esp_now.h documents this
  // explicitly), so esp_now_set_peer_rate_config() is the only way to actually get LR
  // packets rather than a standard 802.11 rate. Without this call, every ESP-NOW packet
  // sent so far has been at whatever the default non-LR rate is - "LR mode" in the logs
  // was a label, not a fact.
  esp_now_rate_config_t rate{};
  rate.phymode = WIFI_PHY_MODE_LR;
  rate.rate = WIFI_PHY_RATE_LORA_250K;
  rate.ersu = false;
  rate.dcm = false;
  esp_err_t rc = esp_now_set_peer_rate_config(mac, &rate);
  Serial.printf("esp_now_set_peer_rate_config(%02X:%02X:%02X:%02X:%02X:%02X) = %d\n", mac[0], mac[1], mac[2], mac[3],
                mac[4], mac[5], (int)rc);
}

static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(EspNowPacket)) return;
  EspNowPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != ESPNOW_MAGIC) return;
  if (pkt.boardId == identityBoardId()) return;  // ignore anything that looks like our own packet

  int8_t rssi = (info && info->rx_ctrl) ? info->rx_ctrl->rssi : 0;
  s_rxFromPeer.onRxSeq(rssi, 1 /* LR mode */, pkt.txSeq);
  s_lastPeerSnapshot = pkt.snap;

  if (!s_havePeer) {
    memcpy(s_peerMac, info->src_addr, 6);
    addPeerIfNeeded(s_peerMac);
    s_havePeer = true;
  }
}

static bool s_active = true;

void espNowLinkInit() {
  // STA protocol (LR vs 11B) is set centrally by main.cpp's applyRadioMode() - both
  // because it needs to be switchable at runtime (see BLE_CHAR_MODE_UUID), and because on
  // this hardware, in practice, *any* LR bit in the STA protocol set makes the
  // concurrently-running softAP invisible to a phone - contrary to Espressif's ESP32-C6
  // Wi-Fi guide, which documents a mixed (non-LR-only) STA protocol set as staying
  // compatible with a plain AP. Confirmed the hard way across several combinations; LR
  // mode and phone-visible-AP mode are mutually exclusive on this chip/core version, so
  // they're switched between rather than run mixed.
  esp_now_init();
  esp_now_register_recv_cb(onRecv);
  addPeerIfNeeded(kBroadcast);
}

void espNowLinkSetActive(bool active) {
  s_active = active;
}

void espNowLinkLoop() {
  if (!s_active) return;
  static uint32_t lastTx = 0;
  uint32_t now = millis();
  if (now - lastTx < ESPNOW_TX_PERIOD_MS) return;
  lastTx = now;

  EspNowPacket pkt{};
  pkt.magic = ESPNOW_MAGIC;
  pkt.boardId = identityBoardId();
  pkt.txSeq = s_txSeq++;
  pkt.snap = s_outgoing;

  const uint8_t* dest = s_havePeer ? s_peerMac : kBroadcast;
  esp_now_send(dest, (const uint8_t*)&pkt, sizeof(pkt));
}

LinkStat espNowLinkGetStat() {
  return s_rxFromPeer.snapshot();
}

bool espNowLinkHasPeer() {
  return s_havePeer;
}

BoardSnapshot espNowLinkGetLastPeerSnapshot() {
  return s_lastPeerSnapshot;
}

void espNowLinkSetOutgoingSnapshot(const BoardSnapshot& snap) {
  s_outgoing = snap;
}

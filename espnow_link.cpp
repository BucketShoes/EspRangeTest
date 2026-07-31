#include "espnow_link.h"
#include "identity.h"
#include "config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

static RollingLink s_rxFromPeer;
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

void espNowLinkInit() {
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR | WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N );
  esp_now_init();
  esp_now_register_recv_cb(onRecv);
  addPeerIfNeeded(kBroadcast);
}

void espNowLinkLoop() {
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

// ESP32-C6 range test rig - identical firmware for both boards.
// See C:\Users\Kaldosh\.claude\plans\i-have-devkits-for-robust-diffie.md for the design.
//
// Board identity, BLE name and AP SSID are all derived automatically from this chip's
// own BT MAC (see identity.cpp) - nothing to configure per-board.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "config.h"
#include "identity.h"
#include "stats.h"
#include "espnow_link.h"
#include "ble_link.h"
#include "webserver_link.h"

void setup() {
  Serial.begin(115200);
  delay(200);  // let USB CDC settle; debug-only, never load-bearing for results

  identityInit();
  Serial.printf("\n=== %s (id=%08X) ===\n", identityDeviceName(), identityBoardId());

  WiFi.mode(WIFI_AP_STA);
  // Fixed channel shared by both boards so the LR-mode STA interface and the softAP
  // land on the same channel (required for AP+STA concurrent operation anyway).
  WiFi.softAP(identityDeviceName(), HTTP_AP_PASSWORD, ESPNOW_CHANNEL);
  esp_wifi_set_max_tx_power(WIFI_TX_POWER_DBM_QUARTER);

  espNowLinkInit();
  bleLinkInit();
  webServerLinkInit();

  Serial.println("Init complete. AP SSID = BLE name = " + String(identityDeviceName()));
}

void loop() {
  espNowLinkLoop();
  bleLinkLoop();
  webServerLinkLoop();

  static uint32_t lastPublish = 0;
  uint32_t now = millis();
  if (now - lastPublish < BLE_SNAPSHOT_PERIOD_MS) return;
  lastPublish = now;

  BoardSnapshot own{};
  own.boardId = identityBoardId();
  own.uptimeMs = now;
  own.espnow = espNowLinkGetStat();
  own.blePeer = bleLinkGetPeerStat();
  own.blePhone = bleLinkGetPhoneStat();
  own.phoneConnected = bleLinkPhoneConnected() ? 1 : 0;
  own.txPowerDbm = BLE_TX_POWER_DBM;

  BoardSnapshot peerRelay = espNowLinkGetLastPeerSnapshot();

  espNowLinkSetOutgoingSnapshot(own);
  bleLinkPublish(own, peerRelay);
  webServerLinkPublish(own, peerRelay);

  Serial.printf("[%lus] espnow rx=%u pdr=%u%% rssi=%d | ble-peer rx=%u rssi=%d phy=%u | ble-phone %s rssi=%d phy=%u\n",
                now / 1000, own.espnow.rxCount, own.espnow.pdrPercent == 0xFF ? 0 : own.espnow.pdrPercent, own.espnow.rssiAvg,
                own.blePeer.rxCount, own.blePeer.rssiAvg, own.blePeer.mode, own.phoneConnected ? "connected" : "-",
                own.blePhone.rssiAvg, own.blePhone.mode);
}

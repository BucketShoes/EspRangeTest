// ESP32-C6 range test rig - identical firmware for both boards.
// See C:\Users\Kaldosh\.claude\plans\i-have-devkits-for-robust-diffie.md for the design.
//
// Board identity, BLE name and AP SSID are all derived automatically from this chip's
// own BT MAC (see identity.cpp) - nothing to configure per-board.
//
// Plain .cpp instead of a .ino: PlatformIO's .ino-to-.cpp conversion step was writing a
// transient intermediate into the project root (visible/writable by every environment
// sharing this src_dir) and racing when two environments built concurrently. A plain
// .cpp with setup()/loop() defined works identically under framework=arduino (the
// framework's own cores/esp32/main.cpp calls extern setup()/loop() regardless of
// whether they came from a .ino conversion or a normal translation unit) - no Arduino
// library usage changes, just skips that conversion step entirely.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include "config.h"
#include "identity.h"
#include "stats.h"
#include "espnow_link.h"
#include "ble_link.h"
#include "webserver_link.h"

// Answer FTM (802.11mc) range requests on the AP - lets a phone (Android WiFi RTT) or
// another board running the esp-idf ftm example range against this board directly.
// esp_wifi_get_config() first so this only adds the ftm_responder flag on top of whatever
// WiFi.softAP() already set (SSID/password/channel), rather than clobbering it. Called
// again after every applyRadioMode() switch (not just once at boot) since it's cheap and
// removes any doubt about whether AP config actually survives an esp_wifi_stop()/start().
static void applyApFtmResponder() {
  wifi_config_t apConf{};
  esp_wifi_get_config(WIFI_IF_AP, &apConf);
  apConf.ap.ftm_responder = true;
  esp_err_t ftmRc = esp_wifi_set_config(WIFI_IF_AP, &apConf);
  Serial.printf("FTM responder enable = %d\n", (int)ftmRc);
}

// 0 = LR mode (ESP-NOW LR + BLE Coded test, default at boot): STA protocol restricted to
//     LR only, ESP-NOW TX active. Trades away phone-AP-visibility/FTM - on this hardware,
//     in practice, any LR bit in the STA protocol set makes the concurrently-running
//     softAP invisible to a phone (see espnow_link.cpp), so LR mode and WiFi/FTM mode are
//     mutually exclusive rather than run together.
// 1 = WiFi/FTM mode: STA protocol forced to 11B only (long-range-capable but still a real
//     802.11 PHY, unlike LR), ESP-NOW TX paused to give FTM's time-critical burst exchange
//     a clearer shot at the shared radio, AP answers FTM requests.
// A full esp_wifi_stop()/esp_wifi_start() cycle is used to change protocol rather than
// just calling esp_wifi_set_protocol() live - protocol changes aren't reliably picked up
// without it.
static uint8_t s_appliedRadioMode = 0xFF;  // sentinel - forces the first apply at boot

static void applyRadioMode(uint8_t mode) {
  if (mode == s_appliedRadioMode) return;
  bool lrMode = (mode == 0);
  Serial.printf("Switching radio mode -> %s\n", lrMode ? "LR (espnow+ble)" : "WiFi/FTM (11b)");

  espNowLinkSetActive(lrMode);
  esp_wifi_stop();
  esp_wifi_set_protocol(WIFI_IF_STA, lrMode ? WIFI_PROTOCOL_LR : WIFI_PROTOCOL_11B);
  esp_wifi_start();
  esp_wifi_set_max_tx_power(WIFI_TX_POWER_DBM_QUARTER);
  applyApFtmResponder();

  s_appliedRadioMode = mode;
}

void setup() {
  Serial.begin(115200);
  delay(200);  // let USB CDC settle; debug-only, never load-bearing for results

  identityInit();
  const uint8_t* mac = identityMac();
  Serial.printf("\n=== %s (id=%08X) mac=%02X:%02X:%02X:%02X:%02X:%02X ===\n", identityDeviceName(), identityBoardId(), mac[0],
                mac[1], mac[2], mac[3], mac[4], mac[5]);

  // Wi-Fi config is flash-backed (WIFI_STORAGE_FLASH) by default and persists in the nvs
  // partition across reflashes of ANY firmware that doesn't override it - confirmed the
  // hard way: a totally unrelated sketch flashed afterward inherited a dead softAP from
  // this, and only a full chip erase (not just reflashing something else) cleared it.
  // esp_wifi_set_storage(WIFI_STORAGE_RAM) below stops *new* writes, but WiFi.mode() (the
  // next line) already triggers esp_wifi_init(), which can itself commit an initial NVS
  // write before that call runs - so on its own it wasn't reliably undoing whatever the
  // previous boot left behind. Erase outright instead: this project stores nothing else
  // in NVS, so every boot now starts from the exact same known-clean state, independent
  // of anything left over from a prior flash.
  nvs_flash_erase();
  nvs_flash_init();

  WiFi.mode(WIFI_AP_STA);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  // Fixed channel shared by both boards so the LR-mode STA interface and the softAP
  // land on the same channel (required for AP+STA concurrent operation anyway).
  bool apOk = WiFi.softAP(identityDeviceName(), HTTP_AP_PASSWORD, ESPNOW_CHANNEL);
  Serial.printf("softAP() = %s, IP = %s, mode = %d\n", apOk ? "true" : "false", WiFi.softAPIP().toString().c_str(), WiFi.getMode());

  espNowLinkInit();
  bleLinkInit();
  webServerLinkInit();
  applyRadioMode(0);  // boot into LR mode by default

  Serial.println("Init complete. AP SSID = BLE name = " + String(identityDeviceName()));
}

void loop() {
  applyRadioMode(bleLinkGetRadioMode());
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

  char pdrStr[6];
  if (own.espnow.pdrPercent == 0xFF) {
    snprintf(pdrStr, sizeof(pdrStr), "n/a");
  } else {
    snprintf(pdrStr, sizeof(pdrStr), "%u%%", own.espnow.pdrPercent);
  }
  Serial.printf("[%lus] espnow rx=%u pdr=%s rssi=%d | ble-peer rx=%u rssi=%d phy=%u | ble-phone %s rssi=%d phy=%u\n",
                now / 1000, own.espnow.rxCount, pdrStr, own.espnow.rssiAvg,
                own.blePeer.rxCount, own.blePeer.rssiAvg, own.blePeer.mode, own.phoneConnected ? "connected" : "-",
                own.blePhone.rssiAvg, own.blePhone.mode);
}

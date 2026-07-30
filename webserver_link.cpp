#include "webserver_link.h"
#include "identity.h"
#include "config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

static WebServer s_server(80);
static BoardSnapshot s_own{};
static BoardSnapshot s_peer{};

static String linkRow(const char* label, const LinkStat& s) {
  String row = "<tr><td>" + String(label) + "</td><td>" + String(s.rxCount) + "</td><td>";
  if (s.pdrPercent == 0xFF) {
    row += "n/a";
  } else {
    row += String(s.pdrPercent) + "%";
  }
  row += "</td><td>" + String(s.rssiAvg) + "</td><td>" + String(s.rssiMin) + "</td><td>" + String(s.rssiMax) + "</td><td>" +
         String(s.mode) + "</td></tr>";
  return row;
}

static void handleRoot() {
  String html = "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='2'>";
  html += "<title>" + String(identityDeviceName()) + "</title>";
  html += "<style>body{font-family:sans-serif;margin:1em}table{border-collapse:collapse;width:100%}"
          "td,th{border:1px solid #999;padding:4px 8px;text-align:right}th{text-align:center}"
          "td:first-child{text-align:left}h2{margin-top:1.5em}</style></head><body>";
  html += "<h1>" + String(identityDeviceName()) + "</h1>";
  html += "<p>Fallback dashboard - no Web Bluetooth here, see the main index.html page for the "
          "full map/timeline view. This just shows what this board currently knows.</p>";

  html += "<h2>This board</h2><table><tr><th>link</th><th>rx</th><th>pdr</th><th>rssi avg</th>"
          "<th>min</th><th>max</th><th>mode</th></tr>";
  html += linkRow("ESP-NOW (from peer)", s_own.espnow);
  html += linkRow("BLE (from peer)", s_own.blePeer);
  html += linkRow("BLE (from phone)", s_own.blePhone);
  html += "</table>";
  html += "<p>phone connected: " + String(s_own.phoneConnected ? "yes" : "no") + "</p>";

  html += "<h2>Peer board (relayed via ESP-NOW)</h2><table><tr><th>link</th><th>rx</th><th>pdr</th>"
          "<th>rssi avg</th><th>min</th><th>max</th><th>mode</th></tr>";
  html += linkRow("ESP-NOW (from us)", s_peer.espnow);
  html += linkRow("BLE (from us)", s_peer.blePeer);
  html += linkRow("BLE (from phone)", s_peer.blePhone);
  html += "</table>";
  html += "<p>peer id: " + String(s_peer.boardId, HEX) + ", peer uptime: " + String(s_peer.uptimeMs / 1000) + "s</p>";

  html += "</body></html>";
  s_server.send(200, "text/html", html);
}

static String linkJson(const LinkStat& s) {
  String j = "{\"rxCount\":" + String(s.rxCount) + ",\"pdrPercent\":";
  j += (s.pdrPercent == 0xFF) ? "null" : String(s.pdrPercent);
  j += ",\"rssiAvg\":" + String(s.rssiAvg) + ",\"rssiMin\":" + String(s.rssiMin) + ",\"rssiMax\":" + String(s.rssiMax) +
       ",\"mode\":" + String(s.mode) + "}";
  return j;
}

static String snapJson(const BoardSnapshot& b) {
  String j = "{\"boardId\":" + String(b.boardId) + ",\"uptimeMs\":" + String(b.uptimeMs);
  j += ",\"espnow\":" + linkJson(b.espnow) + ",\"blePeer\":" + linkJson(b.blePeer) + ",\"blePhone\":" + linkJson(b.blePhone);
  j += ",\"phoneConnected\":" + String(b.phoneConnected ? "true" : "false") + ",\"txPowerDbm\":" + String(b.txPowerDbm) + "}";
  return j;
}

static void handleStatus() {
  String j = "{\"name\":\"" + String(identityDeviceName()) + "\",\"own\":" + snapJson(s_own) + ",\"peer\":" + snapJson(s_peer) + "}";
  s_server.send(200, "application/json", j);
}

void webServerLinkInit() {
  s_server.on("/", handleRoot);
  s_server.on("/status.json", handleStatus);
  s_server.begin();
}

void webServerLinkLoop() {
  s_server.handleClient();
}

void webServerLinkPublish(const BoardSnapshot& ownSnap, const BoardSnapshot& peerRelaySnap) {
  s_own = ownSnap;
  s_peer = peerRelaySnap;
}

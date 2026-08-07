#include "ble_link.h"
#include "identity.h"
#include "config.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <esp_random.h>
#include <string.h>

static const uint16_t kNoHandle = 0xFFFF;

static NimBLEServer* g_server = nullptr;
static NimBLECharacteristic* g_charOwn = nullptr;
static NimBLECharacteristic* g_charPeer = nullptr;

static RollingLink s_rxFromPeer;
static RollingLink s_rxFromPhone;

static NimBLEClient* s_client = nullptr;
static NimBLEAddress s_peerAddr;  // computed once at init - see bleLinkInit()

static uint16_t s_peerConnHandle = kNoHandle;
static uint16_t s_phoneConnHandle = kNoHandle;
// Set directly by RangeServerCB::onConnect() by comparing the connecting device's
// address against s_peerAddr - both are known at boot (hardcoded MACs, see config.h),
// so this is instant and unambiguous. No handshake, no scanning, no dependency on
// ESP-NOW having exchanged anything yet.
static uint16_t s_serverPeerHandle = kNoHandle;

static uint8_t s_radioMode = 0;  // 0 = LR, 1 = WiFi/FTM - see BLE_CHAR_MODE_UUID in config.h

class RadioModeCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    if (c->getValue().length() < 1) return;
    uint8_t requested = c->getValue()[0];
    if (requested <= 1) s_radioMode = requested;
  }
};
static RadioModeCB s_radioModeCB;

static void requestCodedPhy(uint16_t connHandle, bool viaClient) {
  if (viaClient && s_client) {
    s_client->updatePhy(BLE_HCI_LE_PHY_CODED_PREF_MASK, BLE_HCI_LE_PHY_CODED_PREF_MASK, BLE_HCI_LE_PHY_CODED_S8_PREF);
  } else if (g_server) {
    g_server->updatePhy(connHandle, BLE_HCI_LE_PHY_CODED_PREF_MASK, BLE_HCI_LE_PHY_CODED_PREF_MASK, BLE_HCI_LE_PHY_CODED_S8_PREF);
  }
}

// ---- Peripheral role: phone connects here, and the peer board connects here too (both
// boards always dial each other if neither has an incoming connection yet - see
// bleLinkLoop()). ----
class RangeServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    uint16_t h = info.getConnHandle();
    requestCodedPhy(h, false);
    server->updateConnParams(h, BLE_CONN_INTERVAL_MIN, BLE_CONN_INTERVAL_MAX, 0, BLE_CONN_SUPERVISION_TIMEOUT);
    if (info.getAddress() == s_peerAddr) {
      s_serverPeerHandle = h;
    }
    // Legacy advertising stops once a connection is accepted - restart it (while under
    // the connection limit) so this board stays reachable for a second connection
    // (peer + phone at once).
    if (server->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
      NimBLEDevice::startAdvertising();
    }
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& info, int reason) override {
    if (info.getConnHandle() == s_serverPeerHandle) s_serverPeerHandle = kNoHandle;
  }
};
static RangeServerCB s_serverCB;

// ---- Central role: whichever board doesn't yet have any incoming connection dials the
// other (see bleLinkLoop()). ----
class RangeClientCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* client) override {
    s_peerConnHandle = client->getConnHandle();
    requestCodedPhy(s_peerConnHandle, true);
  }

  void onDisconnect(NimBLEClient* client, int reason) override {
    if (client->getConnHandle() == s_peerConnHandle) s_peerConnHandle = kNoHandle;
    s_client = nullptr;  // allow a fresh connect attempt later
  }

  void onConnectFail(NimBLEClient* client, int reason) override {
    s_client = nullptr;  // allow bleLinkLoop() to retry on the next cycle
  }
};
static RangeClientCB s_clientCB;

void bleLinkInit() {
  NimBLEDevice::init(identityDeviceName());
  NimBLEDevice::setPower(BLE_TX_POWER_DBM);

  // Hardcoded identity: figure out which of the two known boards we are, and therefore
  // which one the peer is, by comparing our own MAC against both constants. Only two
  // boards exist for this project, so there's no reason to discover this at runtime.
  const uint8_t macA[6] = BOARD_MAC_3EFE;
  const uint8_t macB[6] = BOARD_MAC_63CA;
  NimBLEAddress addrA(macA, BLE_ADDR_PUBLIC);
  NimBLEAddress addrB(macB, BLE_ADDR_PUBLIC);
  NimBLEAddress myAddr(identityMac(), BLE_ADDR_PUBLIC);
  s_peerAddr = (myAddr == addrA) ? addrB : addrA;

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(&s_serverCB);
  g_server->advertiseOnDisconnect(true);

  NimBLEService* svc = g_server->createService(BLE_SVC_UUID);
  g_charOwn = svc->createCharacteristic(BLE_CHAR_OWN_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_charPeer = svc->createCharacteristic(BLE_CHAR_PEER_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* charInfo = svc->createCharacteristic(BLE_CHAR_INFO_UUID, NIMBLE_PROPERTY::READ);
  charInfo->setValue(identityDeviceName());
  NimBLECharacteristic* charMode = svc->createCharacteristic(BLE_CHAR_MODE_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  charMode->setValue(&s_radioMode, 1);
  charMode->setCallbacks(&s_radioModeCB);
  svc->start();

  // Primary advertisement (31-byte budget) carries just the name, which is what
  // Chrome's requestDevice() namePrefix matches on. The service UUID goes in the scan
  // response instead (separate 31-byte budget) - trying to fit both in the primary
  // packet overflows it and silently drops the UUID.
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName(identityDeviceName());
  // Per-boot random jitter so two otherwise-identical boards don't run perfectly
  // synchronized advertising timing against each other.
  adv->setMinInterval(BLE_ADV_INTERVAL_MIN + (esp_random() % BLE_ADV_INTERVAL_JITTER));
  adv->setMaxInterval(BLE_ADV_INTERVAL_MAX + (esp_random() % BLE_ADV_INTERVAL_JITTER));
  NimBLEAdvertisementData scanResponse;
  scanResponse.addServiceUUID(BLE_SVC_UUID);
  adv->setScanResponseData(scanResponse);
  adv->start();
}

// PHY is fetched fresh at poll time (rather than cached from onPhyUpdate events) so it
// stays correct regardless of which connection a handle turns out to be classified as.
static void pollHandle(uint16_t handle, RollingLink& link, bool isClientHandle) {
  if (handle == kNoHandle) return;
  int8_t rssi = 0;
  if (ble_gap_conn_rssi(handle, &rssi) != 0) return;
  uint8_t txPhy = 0, rxPhy = 0;
  if (isClientHandle && s_client) {
    s_client->getPhy(&txPhy, &rxPhy);
  } else if (g_server) {
    g_server->getPhy(handle, &txPhy, &rxPhy);
  }
  link.onRx(rssi, rxPhy);
}

void bleLinkLoop() {
  static uint32_t lastRssiPoll = 0;
  static uint32_t lastAdvCheck = 0;
  static uint32_t nextConnectAttempt = 0;
  uint32_t now = millis();

  // Any currently-connected server-side handle that isn't the peer (by hardcoded
  // address match) is, by elimination, the phone.
  uint16_t serverPhoneHandle = kNoHandle;
  if (g_server) {
    for (uint16_t h : g_server->getPeerDevices()) {
      if (h != s_serverPeerHandle) serverPhoneHandle = h;
    }
  }
  s_phoneConnHandle = serverPhoneHandle;
  // If we're the central for the peer link, that handle (tracked via the client
  // callbacks) is already authoritative - don't let a stale/absent server-side lookup
  // clobber it.
  if (s_client == nullptr) {
    s_peerConnHandle = s_serverPeerHandle;
  }

  if (now - lastRssiPoll >= BLE_RSSI_POLL_MS) {
    lastRssiPoll = now;
    pollHandle(s_peerConnHandle, s_rxFromPeer, s_client != nullptr);
    pollHandle(s_phoneConnHandle, s_rxFromPhone, false);
  }

  // Defensive: keep re-asserting advertising rather than trusting it to still be
  // running. NimBLE-Arduino has changed its behavior across versions for whether
  // legacy advertising needs to be explicitly restarted in various situations - rather
  // than depend on any particular version, just check periodically and restart if not.
  if (now - lastAdvCheck >= 2000) {
    lastAdvCheck = now;
    if (g_server && g_server->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS && !NimBLEDevice::getAdvertising()->isAdvertising()) {
      NimBLEDevice::startAdvertising();
    }
  }

  // Only dial out if we don't already have any incoming connection at all. Whichever
  // board doesn't yet have anything connected to it (peer or phone) does the work of
  // finding the other; a board that already has an incoming connection just waits.
  // Both addresses are hardcoded (see config.h) - no scanning, no runtime discovery, no
  // handshake, so this can happen immediately and doesn't depend on ESP-NOW at all.
  bool haveAnyIncoming = g_server && g_server->getConnectedCount() > 0;
  if (!haveAnyIncoming && s_peerConnHandle == kNoHandle && s_client == nullptr && now >= nextConnectAttempt) {
    s_client = NimBLEDevice::createClient(s_peerAddr);
    // Without this, a client object that disconnects or fails to connect is never
    // actually freed - NimBLEDevice::deleteClient() only runs automatically when the
    // client is configured to self-delete. Every retry was leaking a client into the
    // fixed-size (CONFIG_BT_NIMBLE_MAX_CONNECTIONS) internal client table, which is
    // almost certainly why one board would work once and then go completely dead:
    // after a few retries there were no client slots left at all ("Already connected to
    // device" is NimBLE finding a stale leaked entry for that address).
    s_client->setSelfDelete(true, true);
    s_client->setClientCallbacks(&s_clientCB, false);
    s_client->setConnectionParams(BLE_CONN_INTERVAL_MIN, BLE_CONN_INTERVAL_MAX, 0, BLE_CONN_SUPERVISION_TIMEOUT);
    // Async - a blocking connect() here freezes the shared main loop() (ESP-NOW TX, BLE
    // peripheral servicing, HTTP server, everything) for as long as the connection
    // attempt takes. Completion arrives via onConnect()/onConnectFail() instead.
    s_client->connect(true, true, true);
    // Cooldown (with jitter) before the next attempt, whether this one succeeds or not -
    // keeps a peer that's out of range from being hammered with reconnect attempts, and
    // keeps the two boards' attempts from staying in lockstep with each other.
    nextConnectAttempt = now + BLE_CONNECT_RETRY_MS + (esp_random() % BLE_CONNECT_RETRY_JITTER_MS);
  }
}

LinkStat bleLinkGetPeerStat() {
  return s_rxFromPeer.snapshot();
}

LinkStat bleLinkGetPhoneStat() {
  return s_rxFromPhone.snapshot();
}

bool bleLinkPhoneConnected() {
  return s_phoneConnHandle != kNoHandle;
}

uint8_t bleLinkGetRadioMode() {
  return s_radioMode;
}

void bleLinkPublish(const BoardSnapshot& ownSnap, const BoardSnapshot& peerRelaySnap) {
  if (!g_charOwn || !g_charPeer) return;
  g_charOwn->setValue((const uint8_t*)&ownSnap, sizeof(ownSnap));
  g_charOwn->notify();
  g_charPeer->setValue((const uint8_t*)&peerRelaySnap, sizeof(peerRelaySnap));
  g_charPeer->notify();
}

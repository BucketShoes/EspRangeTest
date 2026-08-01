#include "ble_link.h"
#include "identity.h"
#include "config.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string.h>

static const uint16_t kNoHandle = 0xFFFF;

static NimBLEServer* g_server = nullptr;
static NimBLECharacteristic* g_charOwn = nullptr;
static NimBLECharacteristic* g_charPeer = nullptr;

static RollingLink s_rxFromPeer;
static RollingLink s_rxFromPhone;

static bool s_havePeerAddr = false;
static NimBLEAddress s_peerAddr;
static NimBLEClient* s_client = nullptr;

static uint16_t s_peerConnHandle = kNoHandle;
static uint16_t s_phoneConnHandle = kNoHandle;

static void requestCodedPhy(uint16_t connHandle, bool viaClient) {
  if (viaClient && s_client) {
    s_client->updatePhy(BLE_HCI_LE_PHY_CODED_PREF_MASK, BLE_HCI_LE_PHY_CODED_PREF_MASK, BLE_HCI_LE_PHY_CODED_S8_PREF);
  } else if (g_server) {
    g_server->updatePhy(connHandle, BLE_HCI_LE_PHY_CODED_PREF_MASK, BLE_HCI_LE_PHY_CODED_PREF_MASK, BLE_HCI_LE_PHY_CODED_S8_PREF);
  }
}

// ---- Scan: find the peer board's BLE address (used for both role tie-break and
// classifying incoming server connections as "peer" vs "phone"). ----
class RangeScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (s_havePeerAddr) return;
    if (!dev->haveName()) return;
    std::string name = dev->getName();
    if (name.rfind(DEVICE_NAME_PREFIX, 0) != 0) return;  // not one of ours
    if (dev->getAddress() == NimBLEDevice::getAddress()) return;
    s_peerAddr = dev->getAddress();
    s_havePeerAddr = true;
  }
};
static RangeScanCB s_scanCB;

// ---- Peripheral role: phone connects here, and/or peer board connects here if we lost the tie-break.
// Which incoming connection is "the peer" vs "the phone" is NOT decided here - see the
// reclassification loop in bleLinkLoop(). Deciding it once, at connect time, races against
// our own scan learning the peer's address (whichever side connects first can easily beat
// the other side's scan), which permanently misclassifies the connection as "phone" if it
// loses that race. Recomputing it every cycle self-heals once the address is known. ----
class RangeServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    requestCodedPhy(info.getConnHandle(), false);
    // Legacy advertising stops once a connection is accepted - restart it (while under
    // the connection limit) so this board stays reachable for a second connection
    // (peer + phone at once).
    if (server->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
      NimBLEDevice::startAdvertising();
    }
  }
};
static RangeServerCB s_serverCB;

// ---- Central role: only used if this board wins the address tie-break with the peer. ----
class RangeClientCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* client) override {
    s_peerConnHandle = client->getConnHandle();
    requestCodedPhy(s_peerConnHandle, true);
  }

  void onDisconnect(NimBLEClient* client, int reason) override {
    if (client->getConnHandle() == s_peerConnHandle) s_peerConnHandle = kNoHandle;
    s_client = nullptr;  // allow a fresh connect attempt later
  }
};
static RangeClientCB s_clientCB;

void bleLinkInit() {
  NimBLEDevice::init(identityDeviceName());
  NimBLEDevice::setPower(BLE_TX_POWER_DBM);

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(&s_serverCB);
  g_server->advertiseOnDisconnect(true);

  NimBLEService* svc = g_server->createService(BLE_SVC_UUID);
  g_charOwn = svc->createCharacteristic(BLE_CHAR_OWN_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_charPeer = svc->createCharacteristic(BLE_CHAR_PEER_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* charInfo = svc->createCharacteristic(BLE_CHAR_INFO_UUID, NIMBLE_PROPERTY::READ);
  charInfo->setValue(identityDeviceName());
  svc->start();

  // Primary advertisement (31-byte budget) carries just the name, which is what both
  // the peer-discovery scan filter and Chrome's requestDevice() namePrefix match on.
  // The service UUID goes in the scan response instead (separate 31-byte budget) -
  // trying to fit both in the primary packet overflows it and silently drops the UUID.
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName(identityDeviceName());
  NimBLEAdvertisementData scanResponse;
  scanResponse.addServiceUUID(BLE_SVC_UUID);
  adv->setScanResponseData(scanResponse);
  adv->start();

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&s_scanCB, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(80);
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
  static uint32_t lastScanKick = 0;
  uint32_t now = millis();

  // Reclassify every currently-connected server-side (peripheral) handle fresh each
  // cycle, rather than trusting a one-time decision made at connect time.
  uint16_t serverPeerHandle = kNoHandle;
  uint16_t serverPhoneHandle = kNoHandle;
  if (g_server) {
    for (uint16_t h : g_server->getPeerDevices()) {
      NimBLEConnInfo info = g_server->getPeerInfoByHandle(h);
      if (s_havePeerAddr && info.getAddress() == s_peerAddr) {
        serverPeerHandle = h;
      } else {
        serverPhoneHandle = h;
      }
    }
  }
  s_phoneConnHandle = serverPhoneHandle;
  // If we're the central for the peer link, that handle (tracked via the client
  // callbacks) is already authoritative - don't let a stale/absent server-side lookup
  // clobber it.
  if (s_client == nullptr) {
    s_peerConnHandle = serverPeerHandle;
  }

  if (now - lastRssiPoll >= BLE_RSSI_POLL_MS) {
    lastRssiPoll = now;
    pollHandle(s_peerConnHandle, s_rxFromPeer, s_client != nullptr);
    pollHandle(s_phoneConnHandle, s_rxFromPhone, false);
  }

  // Keep looking for the peer's address if we don't have it yet, and (if we're the
  // tie-break winner) use it to initiate the inter-module connection.
  if (now - lastScanKick >= BLE_SCAN_KICK_MS) {
    lastScanKick = now;
    if (!s_havePeerAddr && !NimBLEDevice::getScan()->isScanning()) {
      NimBLEDevice::getScan()->start(BLE_SCAN_DURATION_S, false);
    }
  }

  bool weAreSenior = s_havePeerAddr && (NimBLEDevice::getAddress().toString() < s_peerAddr.toString());
  if (weAreSenior && s_peerConnHandle == kNoHandle && s_client == nullptr) {
    s_client = NimBLEDevice::createClient(s_peerAddr);
    s_client->setClientCallbacks(&s_clientCB, false);
    s_client->connect();
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

void bleLinkPublish(const BoardSnapshot& ownSnap, const BoardSnapshot& peerRelaySnap) {
  if (!g_charOwn || !g_charPeer) return;
  g_charOwn->setValue((const uint8_t*)&ownSnap, sizeof(ownSnap));
  g_charOwn->notify();
  g_charPeer->setValue((const uint8_t*)&peerRelaySnap, sizeof(peerRelaySnap));
  g_charPeer->notify();
}

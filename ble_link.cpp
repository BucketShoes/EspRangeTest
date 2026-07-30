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
static uint8_t s_peerPhy = 0;
static uint8_t s_phonePhy = 0;

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

// ---- Peripheral role: phone connects here, and/or peer board connects here if we lost the tie-break. ----
class RangeServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    uint16_t h = info.getConnHandle();
    bool isPeer = s_havePeerAddr && (info.getAddress() == s_peerAddr);
    if (isPeer) {
      s_peerConnHandle = h;
    } else {
      s_phoneConnHandle = h;
    }
    requestCodedPhy(h, false);
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& info, int reason) override {
    uint16_t h = info.getConnHandle();
    if (h == s_peerConnHandle) s_peerConnHandle = kNoHandle;
    if (h == s_phoneConnHandle) s_phoneConnHandle = kNoHandle;
  }

  void onPhyUpdate(NimBLEConnInfo& info, uint8_t txPhy, uint8_t rxPhy) override {
    uint16_t h = info.getConnHandle();
    if (h == s_peerConnHandle) s_peerPhy = rxPhy;
    if (h == s_phoneConnHandle) s_phonePhy = rxPhy;
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

  void onPhyUpdate(NimBLEClient* client, uint8_t txPhy, uint8_t rxPhy) override {
    s_peerPhy = rxPhy;
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

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SVC_UUID);
  adv->start();

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&s_scanCB, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(80);
}

static void pollConnection(uint16_t handle, RollingLink& link) {
  if (handle == kNoHandle) return;
  int8_t rssi = 0;
  if (ble_gap_conn_rssi(handle, &rssi) != 0) return;
  uint8_t mode = (handle == s_peerConnHandle) ? s_peerPhy : s_phonePhy;
  link.onRx(rssi, mode);
}

void bleLinkLoop() {
  static uint32_t lastRssiPoll = 0;
  static uint32_t lastScanKick = 0;
  uint32_t now = millis();

  if (now - lastRssiPoll >= BLE_RSSI_POLL_MS) {
    lastRssiPoll = now;
    pollConnection(s_peerConnHandle, s_rxFromPeer);
    pollConnection(s_phoneConnHandle, s_rxFromPhone);
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

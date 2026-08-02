#include "ble_link.h"
#include "identity.h"
#include "espnow_link.h"
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

static uint16_t s_peerConnHandle = kNoHandle;
static uint16_t s_phoneConnHandle = kNoHandle;
// Set only by HelloCharCB below (a direct handshake, not an address guess) - the peer
// board writes its boardId to our hello characteristic right after connecting, so we
// never have to infer identity from a scan result that may not have arrived yet.
static uint16_t s_serverPeerHandle = kNoHandle;

static void requestCodedPhy(uint16_t connHandle, bool viaClient) {
  if (viaClient && s_client) {
    s_client->updatePhy(BLE_HCI_LE_PHY_CODED_PREF_MASK, BLE_HCI_LE_PHY_CODED_PREF_MASK, BLE_HCI_LE_PHY_CODED_S8_PREF);
  } else if (g_server) {
    g_server->updatePhy(connHandle, BLE_HCI_LE_PHY_CODED_PREF_MASK, BLE_HCI_LE_PHY_CODED_PREF_MASK, BLE_HCI_LE_PHY_CODED_S8_PREF);
  }
}

// ---- Peripheral role: phone connects here, and the peer board connects here too (both
// boards always dial each other now - see bleLinkLoop()). Peer-vs-phone identity is
// established by HelloCharCB's write handler, not decided here. ----
class RangeServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    uint16_t h = info.getConnHandle();
    requestCodedPhy(h, false);
    // Request the longest allowed supervision timeout so the link survives a fade at
    // the edge of range instead of tearing down and forcing a full reconnect. The
    // central (phone or peer) can decline; this is a request, not a guarantee.
    server->updateConnParams(h, BLE_CONN_INTERVAL_MIN, BLE_CONN_INTERVAL_MAX, 0, BLE_CONN_SUPERVISION_TIMEOUT);
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

// ---- Central role: only used if this board wins the boardId tie-break with the peer. ----
class RangeClientCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* client) override {
    s_peerConnHandle = client->getConnHandle();
    requestCodedPhy(s_peerConnHandle, true);
    // Identify ourselves to the peer's peripheral side so it can positively recognize
    // this connection as "the peer" rather than guessing from a scan result.
    NimBLERemoteService* svc = client->getService(BLE_SVC_UUID);
    NimBLERemoteCharacteristic* hello = svc ? svc->getCharacteristic(BLE_CHAR_HELLO_UUID) : nullptr;
    if (hello) {
      uint32_t myId = identityBoardId();
      // response=true (Write Request) - the characteristic was declared with the WRITE
      // property (not WRITE_NR), and a mismatched write type here was silently dropped
      // by the peripheral rather than erroring, which is why the peer-vs-phone
      // classification was failing on one side without any visible error.
      hello->writeValue((const uint8_t*)&myId, sizeof(myId), true);
    }
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

// ---- Hello characteristic (peripheral side): the connecting central writes its
// boardId here immediately after connecting; if it matches the peer we already know
// about via ESP-NOW, that connection handle is positively the peer, not the phone. ----
class HelloCharCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    if (c->getLength() < sizeof(uint32_t)) return;
    uint32_t senderId = c->getValue<uint32_t>();
    if (espNowLinkHasPeer() && senderId == espNowLinkGetLastPeerSnapshot().boardId) {
      s_serverPeerHandle = info.getConnHandle();
    }
  }
};
static HelloCharCB s_helloCB;

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
  NimBLECharacteristic* charHello = svc->createCharacteristic(BLE_CHAR_HELLO_UUID, NIMBLE_PROPERTY::WRITE);
  charHello->setCallbacks(&s_helloCB);
  svc->start();

  // Primary advertisement (31-byte budget) carries just the name, which is what both
  // the peer-discovery scan filter and Chrome's requestDevice() namePrefix match on.
  // The service UUID goes in the scan response instead (separate 31-byte budget) -
  // trying to fit both in the primary packet overflows it and silently drops the UUID.
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
  static uint32_t nextConnectAttempt = 0;
  uint32_t now = millis();

  // Any currently-connected server-side handle that isn't the hello-verified peer is,
  // by elimination, the phone.
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

  // Both boards always try to dial the peer - no elected "central" board. Two redundant
  // peer-to-peer links (we-initiated + they-initiated) can coexist fine; whichever one
  // we dialed ourselves takes priority for reporting (see the s_client check above).
  // This used to be gated on a boardId tie-break so only one side ever dialed out, which
  // concentrated all the central-role work on whichever board had the lower boardId -
  // that board then had a harder time also servicing its own peripheral role (accepting
  // the phone), which is why one specific board was consistently the flaky one to
  // connect a phone to. Splitting the work evenly fixes that asymmetry.
  //
  // The peer's identity (boardId) and its exact connectable BLE address (bleMac) both
  // come from ESP-NOW - no BLE scanning at all for inter-board discovery, so we can
  // never end up dialing some other nearby BLE device by mistake.
  if (espNowLinkHasPeer() && s_peerConnHandle == kNoHandle && s_client == nullptr && now >= nextConnectAttempt) {
    NimBLEAddress peerAddr(espNowLinkGetLastPeerSnapshot().bleMac, BLE_ADDR_PUBLIC);
    s_client = NimBLEDevice::createClient(peerAddr);
    s_client->setClientCallbacks(&s_clientCB, false);
    s_client->setConnectionParams(BLE_CONN_INTERVAL_MIN, BLE_CONN_INTERVAL_MAX, 0, BLE_CONN_SUPERVISION_TIMEOUT);
    // Async - a blocking connect() here freezes the shared main loop() (ESP-NOW TX, BLE
    // peripheral servicing, HTTP server, everything) for as long as the connection
    // attempt takes, which can be many seconds or longer if it doesn't succeed cleanly.
    // Completion arrives via onConnect()/onConnectFail() instead.
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

void bleLinkPublish(const BoardSnapshot& ownSnap, const BoardSnapshot& peerRelaySnap) {
  if (!g_charOwn || !g_charPeer) return;
  g_charOwn->setValue((const uint8_t*)&ownSnap, sizeof(ownSnap));
  g_charOwn->notify();
  g_charPeer->setValue((const uint8_t*)&peerRelaySnap, sizeof(peerRelaySnap));
  g_charPeer->notify();
}

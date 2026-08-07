#pragma once
#include "stats.h"

// BLE peripheral (for phone + peer board) and central (for peer board, if this board wins
// the deterministic address tie-break) running concurrently. Both connections get a
// post-connect PHY update request to Coded S=8; RSSI/PHY are polled periodically on
// whichever connection handles are active.
void bleLinkInit();
void bleLinkLoop();

LinkStat bleLinkGetPeerStat();   // this board's RX-from-peer over BLE, last window
LinkStat bleLinkGetPhoneStat();  // this board's RX-from-phone over BLE, last window
bool bleLinkPhoneConnected();

// Last radio mode requested via BLE_CHAR_MODE_UUID (0 = LR, 1 = WiFi/FTM - see config.h).
// Defaults to 0 (LR) until a write arrives. main.cpp polls this each loop() and applies
// it via applyRadioMode() - kept as a poll rather than a callback-triggered push so the
// actual radio reinit always happens from the main loop, not from the NimBLE task.
uint8_t bleLinkGetRadioMode();

// Pushes the two GATT notify characteristics (own snapshot, relayed peer snapshot).
void bleLinkPublish(const BoardSnapshot& ownSnap, const BoardSnapshot& peerRelaySnap);

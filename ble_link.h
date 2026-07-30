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

// Pushes the two GATT notify characteristics (own snapshot, relayed peer snapshot).
void bleLinkPublish(const BoardSnapshot& ownSnap, const BoardSnapshot& peerRelaySnap);

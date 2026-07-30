#pragma once
#include "stats.h"

// ESP-NOW Long-Range link between the two boards. Also doubles as the relay channel:
// each outgoing packet carries this board's full BoardSnapshot, so whichever board the
// phone is connected to (via BLE) can show both boards' numbers.
void espNowLinkInit();  // call after WiFi.mode(WIFI_AP_STA)+softAP is up
void espNowLinkLoop();  // periodic TX; call every loop()

LinkStat espNowLinkGetStat();  // this board's RX-from-peer stats for the last window (resets window)
bool espNowLinkHasPeer();
BoardSnapshot espNowLinkGetLastPeerSnapshot();  // peer's self-reported snapshot, relayed to us

// Set each cycle with this board's current full snapshot, so it goes out on the next TX.
void espNowLinkSetOutgoingSnapshot(const BoardSnapshot& snap);

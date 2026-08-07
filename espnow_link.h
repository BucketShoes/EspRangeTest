#pragma once
#include "stats.h"

// ESP-NOW Long-Range link between the two boards. Also doubles as the relay channel:
// each outgoing packet carries this board's full BoardSnapshot, so whichever board the
// phone is connected to (via BLE) can show both boards' numbers.
void espNowLinkInit();  // call after WiFi.mode(WIFI_AP_STA)+softAP is up
void espNowLinkLoop();  // periodic TX; call every loop()

// Pause/resume the periodic TX (e.g. while in WiFi/FTM radio mode, to give FTM's
// time-critical burst exchange a clearer shot at the shared radio). RX stays live either
// way - it's the 2Hz broadcast that's the real airtime cost.
void espNowLinkSetActive(bool active);

LinkStat espNowLinkGetStat();  // this board's RX-from-peer stats for the last window (resets window)
bool espNowLinkHasPeer();
BoardSnapshot espNowLinkGetLastPeerSnapshot();  // peer's self-reported snapshot, relayed to us

// Set each cycle with this board's current full snapshot, so it goes out on the next TX.
void espNowLinkSetOutgoingSnapshot(const BoardSnapshot& snap);

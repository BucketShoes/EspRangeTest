#pragma once
#include "stats.h"

// Plain HTTP dashboard served from this board's own softAP. No Web Bluetooth (insecure
// context over plain http://), no GPS/map - just a read-only fallback view of the numbers
// this board already knows (own + relayed peer), reachable by joining its WiFi AP.
void webServerLinkInit();
void webServerLinkLoop();
void webServerLinkPublish(const BoardSnapshot& ownSnap, const BoardSnapshot& peerRelaySnap);

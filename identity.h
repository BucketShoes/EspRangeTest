#pragma once
#include <stdint.h>

// Derives a stable per-chip identity from the BT MAC - no manual configuration needed.
void identityInit();

uint32_t identityBoardId();        // 4-byte id derived from BT MAC (last 4 bytes)
const char* identityDeviceName();  // "ESPRT-XXXX", used for BLE name + AP SSID

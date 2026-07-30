#include "identity.h"
#include "config.h"
#include <esp_mac.h>
#include <stdio.h>

static uint8_t s_mac[6];
static uint32_t s_boardId;
static char s_name[24];

void identityInit() {
  esp_read_mac(s_mac, ESP_MAC_BT);
  s_boardId = ((uint32_t)s_mac[2] << 24) | ((uint32_t)s_mac[3] << 16) | ((uint32_t)s_mac[4] << 8) | (uint32_t)s_mac[5];
  snprintf(s_name, sizeof(s_name), "%s%02X%02X", DEVICE_NAME_PREFIX, s_mac[4], s_mac[5]);
}

uint32_t identityBoardId() {
  return s_boardId;
}

const char* identityDeviceName() {
  return s_name;
}

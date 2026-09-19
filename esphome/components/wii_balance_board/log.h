#pragma once

#include <cstdio>

#ifndef NATIVE
// Route through ESPHome's logger so messages reach the dashboard/API log stream
// (Arduino's log_* macros are compiled out and printf only hits the UART).
#include "esphome/core/log.h"
#define log_d(format, ...) ESP_LOGD("wii_bt", format, ##__VA_ARGS__)
#define log_i(format, ...) ESP_LOGI("wii_bt", format, ##__VA_ARGS__)
#define log_w(format, ...) ESP_LOGW("wii_bt", format, ##__VA_ARGS__)
#define log_e(format, ...) ESP_LOGE("wii_bt", format, ##__VA_ARGS__)
#else
#define log_d(format, ...) printf("\033[32m[D]\033[0m " format "\n", ##__VA_ARGS__)
#define log_i(format, ...) printf("\033[1;32m[I]\033[0m " format "\n", ##__VA_ARGS__)
#define log_w(format, ...) printf("\033[1;33m[W]\033[0m " format "\n", ##__VA_ARGS__)
#define log_e(format, ...) printf("\033[1;31m[E]\033[0m " format "\n", ##__VA_ARGS__)

#endif

#define FORMAT_HEX_MAX_BYTES 30
static char formatHexBuffer[FORMAT_HEX_MAX_BYTES * 3 + 3 + 1];
static char *formatHex(uint8_t *data, uint16_t len) {
  size_t rem = FORMAT_HEX_MAX_BYTES * 3 + 3 + 1;
  for (uint16_t i = 0; i < len && i < FORMAT_HEX_MAX_BYTES; i++) {
    snprintf(formatHexBuffer + 3 * i, rem, "%02X ", data[i]);
    rem -= 3;
  }
  if (FORMAT_HEX_MAX_BYTES < len) {
    snprintf(formatHexBuffer + 3 * FORMAT_HEX_MAX_BYTES, 4, "...");
  }
  return formatHexBuffer;
}

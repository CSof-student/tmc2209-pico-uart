#pragma once

#include <Arduino.h>
#include <IPAddress.h>

// Pico W / Pico 2 W only. Plain Pico builds get no-op stubs.
#if defined(PICO_CYW43_SUPPORTED) || defined(ARDUINO_RASPBERRY_PI_PICO_W) || \
    defined(ARDUINO_RASPBERRY_PI_PICO_2W)
#define TMC_HAS_WIFI 1
#else
#define TMC_HAS_WIFI 0
#endif

#ifndef WIFI_CMD_PORT
#define WIFI_CMD_PORT 3333
#endif

// Copy src/secrets.example.h to src/secrets.h and set your 2.4 GHz SSID/pass.

void wifiSetup();
void wifiService();
bool wifiIsConnected();
IPAddress wifiLocalIP();
int wifiReadChar();

// USB serial plus every connected TCP client.
class CmdOut : public Print {
 public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buffer, size_t size) override;
};

extern CmdOut Out;

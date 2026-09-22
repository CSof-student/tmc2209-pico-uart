#include "wifi_control.h"

#include <string.h>

#if TMC_HAS_WIFI
#include <WiFi.h>
#if __has_include("secrets.h")
#include "secrets.h"
#else
#define WIFI_SSID ""
#define WIFI_PASS ""
#endif
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif

static const char *kPlaceholderSsid = "your-2.4ghz-ssid";
static const uint8_t kMaxClients = 2;
static const uint32_t kRejoinMs = 15000;

CmdOut Out;

#if TMC_HAS_WIFI
static WiFiServer server(WIFI_CMD_PORT);
static WiFiClient clients[kMaxClients];
static bool joinStarted = false;
static bool serverUp = false;
static bool announced = false;
static uint32_t lastJoinMs = 0;
static uint32_t lastLedMs = 0;
static bool ledOn = false;

static bool haveCredentials() {
  return WIFI_SSID[0] != '\0' && strcmp(WIFI_SSID, kPlaceholderSsid) != 0;
}

static void startJoin() {
  WiFi.mode(WIFI_STA);
  WiFi.hostname("tmc-pico");
  if (WIFI_PASS[0] == '\0') {
    WiFi.beginNoBlock(WIFI_SSID);
  } else {
    WiFi.beginNoBlock(WIFI_SSID, WIFI_PASS);
  }
  lastJoinMs = millis();
  joinStarted = true;
}

static void stopClients() {
  for (uint8_t i = 0; i < kMaxClients; i++) {
    if (clients[i]) {
      clients[i].stop();
    }
  }
}

static void wifiWrite(const uint8_t *buf, size_t n) {
  if (n == 0) {
    return;
  }
  for (uint8_t i = 0; i < kMaxClients; i++) {
    if (!clients[i] || !clients[i].connected()) {
      continue;
    }
    if (clients[i].write(buf, n) != n) {
      clients[i].stop();
    }
  }
}

static void acceptClients() {
  WiFiClient incoming = server.accept();
  if (!incoming) {
    return;
  }
  incoming.setNoDelay(true);
  incoming.setTimeout(10);
  for (uint8_t i = 0; i < kMaxClients; i++) {
    if (!clients[i] || !clients[i].connected()) {
      clients[i].stop();
      clients[i] = incoming;
      clients[i].print("TMC2209 ready. Same commands as USB. h = help\r\n");
      return;
    }
  }
  incoming.print("busy\r\n");
  incoming.stop();
}

static void setLed(bool on) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
  ledOn = on;
}
#endif

void wifiSetup() {
#if !TMC_HAS_WIFI
  Serial.println("WiFi: this board has no radio (use rpipico2w / rpipicow)");
  return;
#else
  if (!haveCredentials()) {
    Serial.println("WiFi: copy src/secrets.example.h to src/secrets.h and set SSID/pass");
    return;
  }
  Serial.print("WiFi: joining ");
  Serial.println(WIFI_SSID);
  startJoin();
#endif
}

void wifiService() {
#if TMC_HAS_WIFI
  if (!joinStarted) {
    return;
  }

  if (WiFi.connected()) {
    if (!serverUp) {
      server.begin();
      serverUp = true;
      announced = false;
    }
    if (!announced) {
      announced = true;
      setLed(true);
      Serial.print("WiFi ");
      Serial.print(WiFi.localIP());
      Serial.print("  nc ");
      Serial.print(WiFi.localIP());
      Serial.print(' ');
      Serial.println(WIFI_CMD_PORT);
    }
    acceptClients();
    return;
  }

  if (serverUp) {
    stopClients();
    server.end();
    serverUp = false;
    announced = false;
  }

  const uint32_t now = millis();
  if (now - lastLedMs >= 250) {
    lastLedMs = now;
    setLed(!ledOn);
  }
  if (now - lastJoinMs >= kRejoinMs) {
    startJoin();
  }
#endif
}

bool wifiIsConnected() {
#if TMC_HAS_WIFI
  return joinStarted && WiFi.connected();
#else
  return false;
#endif
}

IPAddress wifiLocalIP() {
#if TMC_HAS_WIFI
  if (wifiIsConnected()) {
    return WiFi.localIP();
  }
#endif
  return IPAddress((uint32_t)0);
}

int wifiReadChar() {
#if TMC_HAS_WIFI
  for (uint8_t i = 0; i < kMaxClients; i++) {
    if (clients[i] && clients[i].connected() && clients[i].available()) {
      return clients[i].read();
    }
  }
#endif
  return -1;
}

size_t CmdOut::write(uint8_t c) {
  Serial.write(c);
#if TMC_HAS_WIFI
  wifiWrite(&c, 1);
#endif
  return 1;
}

size_t CmdOut::write(const uint8_t *buffer, size_t size) {
  Serial.write(buffer, size);
#if TMC_HAS_WIFI
  wifiWrite(buffer, size);
#endif
  return size;
}

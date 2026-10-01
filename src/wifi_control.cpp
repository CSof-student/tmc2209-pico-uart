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
UsbOut Plot;

static void usbWrite(const uint8_t *buf, size_t n) {
  if (n == 0 || !Serial) {
    return;
  }
  const int space = Serial.availableForWrite();
  if (space <= 0) {
    return;
  }
  Serial.write(buf, n < (size_t)space ? n : (size_t)space);
}

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
    int space = clients[i].availableForWrite();
    if (space <= 0) {
      continue;
    }
    if ((size_t)space > n) {
      space = (int)n;
    }
    clients[i].write(buf, (size_t)space);
  }
}

static void acceptClients() {
  WiFiClient incoming = server.accept();
  if (!incoming) {
    return;
  }
  incoming.setNoDelay(true);
  incoming.setSync(false);
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

static const char *wifiStatusName(uint8_t st) {
  switch (st) {
    case WL_IDLE_STATUS:
      return "idle";
    case WL_NO_SSID_AVAIL:
      return "no-ssid";
    case WL_SCAN_COMPLETED:
      return "scanned";
    case WL_CONNECTED:
      return "connected";
    case WL_CONNECT_FAILED:
      return "fail";
    case WL_CONNECTION_LOST:
      return "lost";
    case WL_DISCONNECTED:
      return "disconnected";
    default:
      return "other";
  }
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
    Serial.print("WiFi: retry ");
    Serial.print(WIFI_SSID);
    Serial.print("  st=");
    Serial.println(wifiStatusName(WiFi.status()));
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

void wifiPrintStatus() {
#if !TMC_HAS_WIFI
  Out.println("  wifi=down  no-radio (build env must be rpipico2w / rpipicow)");
  return;
#else
  if (!haveCredentials()) {
    Out.println("  wifi=down  no-secrets (src/secrets.h missing or still placeholder)");
    return;
  }
  if (wifiIsConnected()) {
    Out.print("  wifi=");
    Out.print(WiFi.localIP());
    Out.print(':');
    Out.print(WIFI_CMD_PORT);
    Out.print("  ssid=");
    Out.println(WIFI_SSID);
    return;
  }
  Out.print("  wifi=down  ssid=");
  Out.print(WIFI_SSID);
  Out.print("  st=");
  Out.print(wifiStatusName(WiFi.status()));
  Out.print('(');
  Out.print(WiFi.status());
  Out.print(')');
  Out.println(joinStarted ? "  still joining" : "  join not started");
  Out.println("  scanning 2.4 GHz...");
  const int8_t n = WiFi.scanNetworks();
  if (n <= 0) {
    Out.println("  scan: none (out of range, or no 2.4 GHz APs)");
    return;
  }
  bool seen = false;
  for (int8_t i = 0; i < n && i < 16; i++) {
    const bool match = strcmp(WiFi.SSID(i), WIFI_SSID) == 0;
    if (match) {
      seen = true;
    }
    Out.print(match ? "  * " : "    ");
    Out.print(WiFi.SSID(i));
    Out.print("  ch=");
    Out.print(WiFi.channel(i));
    Out.print("  rssi=");
    Out.println(WiFi.RSSI(i));
  }
  if (seen) {
    Out.println("  SSID found — fail is usually password or WPA3-only (Pico needs WPA2)");
  } else {
    Out.println("  SSID not in 2.4 GHz scan — wrong name, or that AP is 5 GHz only");
  }
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
  usbWrite(&c, 1);
#if TMC_HAS_WIFI
  wifiWrite(&c, 1);
#endif
  return 1;
}

size_t CmdOut::write(const uint8_t *buffer, size_t size) {
  usbWrite(buffer, size);
#if TMC_HAS_WIFI
  wifiWrite(buffer, size);
#endif
  return size;
}

size_t UsbOut::write(uint8_t c) {
  usbWrite(&c, 1);
  return 1;
}

size_t UsbOut::write(const uint8_t *buffer, size_t size) {
  usbWrite(buffer, size);
  return size;
}

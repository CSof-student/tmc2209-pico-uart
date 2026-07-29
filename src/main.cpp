/*
  Pico + TMC2209 UART diagnostics + FastAccelStepper

  If `t` used to freeze: often TX (GP4) is shorted to GND from an old
  breadboard short. This build checks pins BEFORE any Serial1 write.

  Wiring (normal):
    GP4 TX -> TMC Rx
    GP5 RX <- TMC Tx
    STEP GP3, DIR GP2, EN->GND
    VIO 3.3V, VM motor PSU, GND common
*/

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

static const uint8_t STEP_PIN = 3;
static const uint8_t DIR_PIN = 2;
static const int8_t ENABLE_PIN = -1;

static const uint8_t PIN_A = 4;  // normally TX
static const uint8_t PIN_B = 5;  // normally RX
static const float R_SENSE = 0.11f;

static const uint16_t DEFAULT_RMS_MA = 300;
static const uint16_t DEFAULT_MICROSTEPS = 16;
static const uint32_t DEFAULT_SPEED_HZ = 200;
static const uint32_t DEFAULT_ACCEL = 400;
static const int32_t DEFAULT_STEP_SIZE = 8;

#define SERIAL_PORT Serial1

uint8_t tmcTxPin = PIN_A;
uint8_t tmcRxPin = PIN_B;
uint8_t driverAddress = 0;
bool uartPinsSwapped = false;
bool uartPortStarted = false;

TMC2209Stepper *driver = nullptr;
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

int32_t stepSize = DEFAULT_STEP_SIZE;
uint16_t rmsMa = DEFAULT_RMS_MA;
bool driverConfigured = false;
String line;

void recreateDriver(uint8_t addr) {
  driverAddress = addr & 0x03;
  delete driver;
  driver = new TMC2209Stepper(&SERIAL_PORT, R_SENSE, driverAddress);
}

void stopUartPort() {
  if (uartPortStarted) {
    SERIAL_PORT.end();
    delay(10);
    uartPortStarted = false;
  }
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
}

// Returns false only if the TX pin looks stuck LOW (unsafe to transmit into).
// RX may read LOW when the TMC is attached (module pulldown) — that is OK.
bool checkUartPinsSafeToStart() {
  stopUartPort();
  delay(5);

  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  delay(5);

  const int a = digitalRead(PIN_A);
  const int b = digitalRead(PIN_B);
  const int txLevel = (tmcTxPin == PIN_A) ? a : b;
  const int rxLevel = (tmcRxPin == PIN_A) ? a : b;

  Serial.print(F("Pin check: GP4="));
  Serial.print(a == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F("  GP5="));
  Serial.print(b == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F("  (TX=GP"));
  Serial.print(tmcTxPin);
  Serial.print(F(" RX=GP"));
  Serial.print(tmcRxPin);
  Serial.println(F(")"));

  if (txLevel == LOW) {
    Serial.println(F("UNSAFE: TX pin is LOW — do not start UART (would drive into a low/short)."));
    Serial.println(F("  Try: w  (swap so the LOW line is RX), then k/t again"));
    Serial.println(F("  Or disconnect TMC; if TX pin still LOW, breadboard/Pico issue"));
    return false;
  }

  if (rxLevel == LOW) {
    Serial.println(F("OK-ish: RX is LOW with TMC attached (common pulldown). Safe to try UART."));
  } else {
    Serial.println(F("Pin check OK for UART start (TX is HIGH)."));
  }
  return true;
}

bool ensureUartPortSafe() {
  if (uartPortStarted) {
    return true;
  }
  if (!checkUartPinsSafeToStart()) {
    return false;
  }

  SERIAL_PORT.setTX(tmcTxPin);
  SERIAL_PORT.setRX(tmcRxPin);
  SERIAL_PORT.begin(115200);
  delay(30);
  while (SERIAL_PORT.available()) {
    SERIAL_PORT.read();
  }
  uartPortStarted = true;
  return true;
}

void printHelp() {
  Serial.println(F("\nCommands (safe ones first):"));
  Serial.println(F("  h   help"));
  Serial.println(F("  k   pin short check (do this first)"));
  Serial.println(F("  l   UART loopback hint / test helpers"));
  Serial.println(F("  t   TMC version scan (timed, no Serial1.flush)"));
  Serial.println(F("  w   swap TX/RX — ONLY if TX pin was LOW"));
  Serial.println(F("      If RX was LOW, do not swap (that caused the freeze)"));
  Serial.println(F("  u   apply TMC defaults"));
  Serial.println(F("  c/m/i/+/- /p/x   current/microsteps/status/move/..."));
  Serial.println(F("\nAfter a shorted driver: rewire on a FRESH breadboard section."));
}

void printMotionStatus() {
  Serial.print(F("pos="));
  Serial.print(stepper ? stepper->getCurrentPosition() : 0);
  Serial.print(F("  stepSize="));
  Serial.print(stepSize);
  Serial.print(F("  addr="));
  Serial.print(driverAddress);
  Serial.print(F("  swapped="));
  Serial.println(uartPinsSwapped ? F("yes") : F("no"));
}

void loopbackHelp() {
  Serial.println(F("\nLoopback test (TMC UART wires DISCONNECTED):"));
  Serial.println(F("  1) Run k  — both pins must be HIGH"));
  Serial.println(F("  2) Jumper GP4 directly to GP5"));
  Serial.println(F("  3) Run b  — should echo a byte OK"));
  Serial.println(F("  4) Remove jumper, reconnect to TMC Rx/Tx, run k then t"));
}

void loopbackByteTest() {
  stopUartPort();
  if (!checkUartPinsSafeToStart()) {
    return;
  }

  // User should have GP4 jumpered to GP5 for this test
  SERIAL_PORT.setTX(tmcTxPin);
  SERIAL_PORT.setRX(tmcRxPin);
  SERIAL_PORT.begin(115200);
  uartPortStarted = true;
  delay(20);
  while (SERIAL_PORT.available()) {
    SERIAL_PORT.read();
  }

  Serial.println(F("Loopback: writing 0xA5 (GP4 must be jumpered to GP5)..."));
  Serial.flush();

  SERIAL_PORT.write((uint8_t)0xA5);
  SERIAL_PORT.flush();

  const uint32_t start = millis();
  int got = -1;
  while (millis() - start < 200) {
    if (SERIAL_PORT.available()) {
      got = SERIAL_PORT.read();
      break;
    }
    delay(1);
  }

  if (got == 0xA5) {
    Serial.println(F("Loopback OK — Pico UART pins work"));
  } else {
    Serial.print(F("Loopback FAIL (got "));
    Serial.print(got);
    Serial.println(F("). Pins/wiring/Serial1 problem — do not run t yet"));
  }

  stopUartPort();
}

// TMC2209 uses an 8-bit CRC (same polynomial as TMCStepper).
uint8_t tmcCrc(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t current = data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (current & 0x01)) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
      current >>= 1;
    }
  }
  return crc;
}

// Timed TX that never calls flush() — flush can hang forever if TX fights the bus.
bool writeBytesNoFlush(const uint8_t *data, uint8_t len, uint32_t timeoutMs) {
  const uint32_t start = millis();
  uint8_t i = 0;
  while (i < len) {
    if (millis() - start > timeoutMs) {
      return false;
    }
    if (SERIAL_PORT.availableForWrite() > 0) {
      SERIAL_PORT.write(data[i++]);
    } else {
      delay(1);
    }
  }
  return true;
}

bool readBytesTimed(uint8_t *data, uint8_t len, uint32_t timeoutMs) {
  const uint32_t start = millis();
  uint8_t i = 0;
  while (i < len) {
    if (millis() - start > timeoutMs) {
      return false;
    }
    if (SERIAL_PORT.available()) {
      data[i++] = (uint8_t)SERIAL_PORT.read();
    } else {
      delay(1);
    }
  }
  return true;
}

// Returns version byte or 0xFF on timeout/fail. Does not use TMCStepper (avoids flush hang).
uint8_t probeVersionRaw(uint8_t addr) {
  while (SERIAL_PORT.available()) {
    SERIAL_PORT.read();
  }

  // Read request for register 0x06 (IOIN) — reply includes version in bits 31..24 of IOIN payload.
  // Simpler: read GCONF 0x00 also works; TMCStepper uses IFCNT/IOIN. Use IOIN 0x06.
  uint8_t req[4];
  req[0] = 0x05;                  // sync
  req[1] = (uint8_t)(addr & 0x03); // slave addr, read bit=0
  req[2] = 0x06;                  // IOIN
  req[3] = tmcCrc(req, 3);

  if (!writeBytesNoFlush(req, 4, 50)) {
    Serial.print(F("tx-timeout "));
    return 0xFF;
  }

  // Give the driver a moment; do NOT flush.
  delay(5);

  uint8_t resp[8];
  if (!readBytesTimed(resp, 8, 100)) {
    return 0x00;  // no reply
  }

  // Reply: sync, addr|0xFF master, reg, data0..data3, crc
  // IOIN version is in data3 (byte index 6) for TMC2209
  if (resp[0] != 0x05) {
    return 0x00;
  }
  return resp[6];
}

void testUart() {
  Serial.println(F("Pre-check before TMC UART..."));
  Serial.flush();  // USB Serial flush is fine
  stopUartPort();

  if (!ensureUartPortSafe()) {
    Serial.println(F("Aborting t."));
    Serial.println(F("If RX was the LOW pin, do NOT swap — stay TX=GP4 RX=GP5 and retry t."));
    return;
  }

  Serial.println(F("Scanning addrs 0..3 with timed UART (no flush, cannot hang on TX)."));
  Serial.print(F("TX=GP"));
  Serial.print(tmcTxPin);
  Serial.print(F(" RX=GP"));
  Serial.println(tmcRxPin);
  Serial.flush();

  int found = -1;
  uint8_t foundVer = 0;
  for (uint8_t addr = 0; addr < 4; addr++) {
    Serial.print(F("  addr "));
    Serial.print(addr);
    Serial.print(F(": "));
    Serial.flush();

    const uint8_t ver = probeVersionRaw(addr);
    Serial.print(F("version=0x"));
    Serial.print(ver, HEX);

    if (ver == 0x21 || ver == 0x20) {
      Serial.println(F(" OK"));
      found = addr;
      foundVer = ver;
      break;
    }
    if (ver == 0xFF) {
      Serial.println(F(" TX stuck/timeout — unswap with w if you swapped"));
      break;
    }
    Serial.println(F(" fail"));
  }

  // Always release UART hardware after probe so a bad bus can't keep wedging later.
  stopUartPort();

  if (found >= 0) {
    recreateDriver((uint8_t)found);
    Serial.print(F("Using addr "));
    Serial.print(found);
    Serial.print(F(" ver=0x"));
    Serial.println(foundVer, HEX);
    Serial.println(F("Next: u"));
  } else {
    Serial.println(F("No TMC response."));
    Serial.println(F("Tip: LOW on GP5 (RX) is normal — use default TX=GP4, do not swap."));
    Serial.println(F("     Swap only if GP4 (TX) was LOW."));
  }
}

void swapUartPins() {
  uartPinsSwapped = !uartPinsSwapped;
  if (uartPinsSwapped) {
    tmcTxPin = PIN_B;
    tmcRxPin = PIN_A;
  } else {
    tmcTxPin = PIN_A;
    tmcRxPin = PIN_B;
  }
  stopUartPort();
  Serial.print(F("Now TX=GP"));
  Serial.print(tmcTxPin);
  Serial.print(F(" RX=GP"));
  Serial.println(tmcRxPin);
  Serial.println(F("Reminder: only swap if the LOW pin was the TX pin."));
}

void applyDriverDefaults() {
  if (!ensureUartPortSafe()) {
    return;
  }
  if (!driver) {
    recreateDriver(driverAddress);
  }
  Serial.println(F("Applying TMC defaults..."));
  Serial.flush();
  driver->begin();
  driver->pdn_disable(true);
  driver->I_scale_analog(false);
  driver->toff(5);
  driver->rms_current(rmsMa);
  driver->microsteps(DEFAULT_MICROSTEPS);
  driver->pwm_autoscale(true);
  driver->en_spreadCycle(false);
  uint8_t hold = (uint8_t)(driver->irun() * 3 / 10);
  if (hold < 1) {
    hold = 1;
  }
  driver->ihold(hold);
  driver->iholddelay(2);
  driverConfigured = true;
  Serial.println(F("Done"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  delay(200);

  Serial.println(F("=== FW tmc-uart-v7 ==="));
  Serial.println(F("USB OK — TMC UART not started"));
  Serial.println(F("If RX pin was LOW: do NOT swap. Freeze was likely TX fighting TMC Tx."));
  Serial.println(F("Safe UART probe: timed writes, never Serial1.flush to TMC."));
  recreateDriver(0);

  if (ENABLE_PIN >= 0) {
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
  }

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN, true, 5);
    stepper->setSpeedInHz(DEFAULT_SPEED_HZ);
    stepper->setAcceleration(DEFAULT_ACCEL);
    stepper->setCurrentPosition(0);
  }

  printHelp();
  Serial.println(F("Start with: k"));
  Serial.print(F("> "));
  Serial.flush();
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    return;
  }
  const char c = cmd.charAt(0);

  if (c == 'h' || c == 'H' || c == '?') {
    printHelp();
  } else if (c == 'k' || c == 'K') {
    checkUartPinsSafeToStart();
  } else if (c == 'l' || c == 'L') {
    loopbackHelp();
  } else if (c == 'b' || c == 'B') {
    loopbackByteTest();
  } else if (c == 't' || c == 'T') {
    testUart();
  } else if (c == 'w' || c == 'W') {
    swapUartPins();
  } else if (c == 'u' || c == 'U') {
    applyDriverDefaults();
  } else if (c == 'p' || c == 'P') {
    printMotionStatus();
  } else if (c == '+' || c == '-') {
    int32_t delta = (c == '+') ? stepSize : -stepSize;
    if (cmd.length() > 1) {
      delta = cmd.substring(1).toInt();
      if (c == '-' && delta > 0) {
        delta = -delta;
      }
    }
    if (stepper) {
      stepper->move(delta);
    }
    Serial.print(F("move "));
    Serial.println(delta);
  } else if (c == 'n' || c == 'N') {
    const int32_t v = cmd.substring(1).toInt();
    if (v > 0) {
      stepSize = v;
    }
  } else if (c == 's' || c == 'S') {
    const uint32_t v = (uint32_t)cmd.substring(1).toInt();
    if (v > 0 && stepper) {
      stepper->setSpeedInHz(v);
    }
  } else if (c == 'a' || c == 'A') {
    const int32_t v = cmd.substring(1).toInt();
    if (v > 0 && stepper) {
      stepper->setAcceleration(v);
    }
  } else if (c == 'c' || c == 'C') {
    const int v = cmd.substring(1).toInt();
    if (v > 0 && v < 2000 && ensureUartPortSafe()) {
      rmsMa = (uint16_t)v;
      if (!driver) {
        recreateDriver(driverAddress);
      }
      driver->rms_current(rmsMa);
      Serial.print(F("rms_mA="));
      Serial.println(rmsMa);
    }
  } else if (c == 'm' || c == 'M') {
    const int v = cmd.substring(1).toInt();
    if ((v == 8 || v == 16 || v == 32 || v == 64 || v == 128 || v == 256) &&
        ensureUartPortSafe()) {
      if (!driver) {
        recreateDriver(driverAddress);
      }
      driver->microsteps(v);
      Serial.println(v);
    }
  } else if (c == 'i' || c == 'I') {
    if (ensureUartPortSafe() && driver) {
      Serial.print(F("ver=0x"));
      Serial.println(driver->version(), HEX);
    }
  } else if (c == 'x' || c == 'X') {
    if (stepper) {
      stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
    }
  } else {
    Serial.println(F("unknown (h)"));
  }
}

void loop() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (line.length() > 0) {
        processCommand(line);
        line = "";
        Serial.print(F("> "));
        Serial.flush();
      }
    } else {
      line += ch;
    }
  }
}

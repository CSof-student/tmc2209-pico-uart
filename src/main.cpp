/*
  Pico + TMC2209 UART diagnostics + FastAccelStepper

  Wiring (v11):
    GP8 TX -> TMC Rx
    GP9 RX <- TMC Tx
    STEP GP3, DIR GP2, EN->GND
    VIO 3.3V, VM motor PSU, GND common

  Avoid GP0/GP1 (default Serial1) and GP5 (stuck LOW on your breadboard).
*/

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

static const char *FW_VERSION = "tmc-uart-v23";

static const uint8_t STEP_PIN = 3;
static const uint8_t DIR_PIN = 2;
static const int8_t ENABLE_PIN = -1;

// Soft/HW UART pins — keep off GP0/GP1 (UART0 defaults) and off damaged GP5 column
static const uint8_t PIN_A = 8;  // TX -> TMC Rx
static const uint8_t PIN_B = 9;  // RX <- TMC Tx
static const float R_SENSE = 0.11f;

static const uint16_t DEFAULT_RMS_MA = 300;
static const uint16_t DEFAULT_MICROSTEPS = 16;
static const uint32_t DEFAULT_SPEED_HZ = 200;
static const uint32_t DEFAULT_ACCEL = 400;
static const int32_t DEFAULT_STEP_SIZE = 200;

#define SERIAL_PORT Serial1

// 1/9600 ≈ 104 µs — used by soft UART loopback + TMC probe
static const uint16_t SOFT_BIT_US = 104;

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
  // Avoid Serial1.end() — can hang forever on single-wire / half-duplex setups.
  uartPortStarted = false;
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  delay(1);
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
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

  Serial.print(F("Pin check: GP"));
  Serial.print(PIN_A);
  Serial.print(F("="));
  Serial.print(a == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F("  GP"));
  Serial.print(PIN_B);
  Serial.print(F("="));
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
    Serial.println(F("RX reads LOW. If meter shows ~1.2V on GP9, add pull-up to 3.3V."));
    Serial.println(F("  Need idle >~2.0V for Pico to see HIGH."));
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
  Serial.println(F("  v   print firmware version (always safe)"));
  Serial.println(F("  h   help"));
  Serial.println(F("  k   pin short check (do this first)"));
  Serial.println(F("  l   UART loopback hint / test helpers"));
  Serial.println(F("  t   TMC probe via software UART @9600 (no Serial1)"));
  Serial.println(F("  w   swap TX/RX — ONLY if TX pin was LOW"));
  Serial.println(F("      If RX was LOW, do not swap"));
  Serial.println(F("  u   apply TMC defaults (uses Serial1 — may hang if bus bad)"));
  Serial.println(F("  c/m/i/+/- /p/x   current/microsteps/status/move/..."));
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

// Soft UART helpers (defined below) — used by b/t so Serial1 is never required for diagnostics.
void softUartIdle();
void softUartWriteByte(uint8_t b);
bool softUartReadByte(uint8_t &out, uint32_t timeoutMs, bool requireIdleHigh = true);

void loopbackHelp() {
  Serial.println(F("\nLoopback test (TMC UART wires DISCONNECTED):"));
  Serial.println(F("  1) Run k  — both pins must be HIGH"));
  Serial.println(F("  2) Jumper GP8 directly to GP9"));
  Serial.println(F("  3) Run b  — soft-UART echo (no Serial1; should not freeze)"));
  Serial.println(F("  4) Remove jumper, reconnect to TMC, run k then t"));
}

void loopbackByteTest() {
  stopUartPort();  // release Serial1 if it was left on
  if (!checkUartPinsSafeToStart()) {
    return;
  }

  Serial.println(F("Loopback: soft UART 0xA5 (GP8 jumpered to GP9; TMC UART wires OFF)..."));
  Serial.flush();

  // Must sample RX *during* TX — after writeByte() returns, the echo is already gone.
  const uint8_t sent = 0xA5;
  uint8_t got = 0;
  pinMode(tmcTxPin, OUTPUT);
  digitalWrite(tmcTxPin, HIGH);
  pinMode(tmcRxPin, INPUT_PULLUP);
  delay(1);

  noInterrupts();
  digitalWrite(tmcTxPin, LOW);  // start bit
  delayMicroseconds(SOFT_BIT_US);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(tmcTxPin, (sent >> i) & 0x01);
    delayMicroseconds(SOFT_BIT_US / 2);
    if (digitalRead(tmcRxPin)) {
      got |= (uint8_t)(1u << i);
    }
    delayMicroseconds(SOFT_BIT_US - SOFT_BIT_US / 2);
  }
  digitalWrite(tmcTxPin, HIGH);  // stop bit
  delayMicroseconds(SOFT_BIT_US);
  interrupts();

  if (got == sent) {
    Serial.println(F("Loopback OK — Pico GP8/GP9 jumper path works"));
  } else {
    Serial.print(F("Loopback FAIL (sent 0xA5 got 0x"));
    Serial.print(got, HEX);
    Serial.println(F(")."));
    Serial.println(F("  - Disconnect TMC RX/TX from GP8/GP9"));
    Serial.println(F("  - Jumper GP8 directly to GP9 (same breadboard row)"));
    Serial.println(F("  - Run k (both HIGH), then b again"));
  }

  pinMode(tmcTxPin, INPUT_PULLUP);
  pinMode(tmcRxPin, INPUT_PULLUP);
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

// Software UART @ 9600 — never touches Serial1, so it cannot hang the HW UART FIFO.
// Datasheet-style: push-pull TX through series 1k while sending; release TX (high-Z)
// after the datagram so the TMC can drive the shared UART pin.
static inline void softWaitUntil(uint32_t deadline_us) {
  while ((int32_t)(time_us_32() - deadline_us) < 0) {
  }
}

void softUartIdle() {
  pinMode(tmcTxPin, INPUT);  // high-Z; external pull-up holds idle HIGH
  pinMode(tmcRxPin, INPUT_PULLUP);
}

// Unlocked write — caller must hold noInterrupts() for the whole datagram.
void softUartWriteByteUnlocked(uint8_t b) {
  pinMode(tmcTxPin, OUTPUT);

  // Start bit — time each bit FROM the edge (avoids short bits if setup is slow)
  digitalWrite(tmcTxPin, LOW);
  softWaitUntil(time_us_32() + SOFT_BIT_US);

  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(tmcTxPin, (b & 0x01) ? HIGH : LOW);
    b >>= 1;
    softWaitUntil(time_us_32() + SOFT_BIT_US);
  }

  digitalWrite(tmcTxPin, HIGH);  // stop bit
  softWaitUntil(time_us_32() + SOFT_BIT_US);
}

void softUartWriteBytes(const uint8_t *data, uint8_t len) {
  noInterrupts();
  for (uint8_t i = 0; i < len; i++) {
    softUartWriteByteUnlocked(data[i]);
  }
  // Release bus for TMC reply (do not leave UART TX driving HIGH into the 1k)
  pinMode(tmcTxPin, INPUT);
  interrupts();
}

void softUartWriteByte(uint8_t b) {
  noInterrupts();
  softUartWriteByteUnlocked(b);
  pinMode(tmcTxPin, INPUT);
  interrupts();
}

// Read one byte; start-bit falling edge already seen unless waitFalling.
// Caller must hold noInterrupts().
bool softUartReadByteUnlocked(uint8_t &out, bool waitFalling) {
  if (waitFalling) {
    const uint32_t giveUp = time_us_32() + SOFT_BIT_US * 30;
    // If line is already LOW, that IS the next start bit — do not wait for HIGH first
    // (waiting HIGH would skip the byte and cause noSync).
    if (digitalRead(tmcRxPin) == HIGH) {
      while (digitalRead(tmcRxPin) == HIGH) {
        if ((int32_t)(time_us_32() - giveUp) >= 0) {
          return false;
        }
      }
    }
  }

  const uint32_t t0 = time_us_32();
  uint8_t b = 0;
  for (uint8_t i = 0; i < 8; i++) {
    softWaitUntil(t0 + SOFT_BIT_US + (SOFT_BIT_US / 2) + (uint32_t)i * SOFT_BIT_US);
    if (digitalRead(tmcRxPin)) {
      b |= (uint8_t)(1u << i);
    }
  }
  softWaitUntil(t0 + SOFT_BIT_US * 10);
  out = b;
  return true;
}

bool softUartReadByte(uint8_t &out, uint32_t timeoutMs, bool requireIdleHigh) {
  const uint32_t start = millis();

  if (requireIdleHigh) {
    while (digitalRead(tmcRxPin) == LOW) {
      if (millis() - start > timeoutMs) {
        return false;
      }
    }
  }

  if (digitalRead(tmcRxPin) == HIGH) {
    while (digitalRead(tmcRxPin) == HIGH) {
      if (millis() - start > timeoutMs) {
        return false;
      }
    }
  }

  noInterrupts();
  const bool ok = softUartReadByteUnlocked(out, false);
  interrupts();
  return ok;
}

bool softUartReadBytes(uint8_t *data, uint8_t len, uint32_t firstTimeoutMs, uint32_t nextTimeoutMs) {
  for (uint8_t i = 0; i < len; i++) {
    if (!softUartReadByte(data[i], i == 0 ? firstTimeoutMs : nextTimeoutMs, i == 0)) {
      return false;
    }
  }
  return true;
}

uint8_t probeVersionSoft(uint8_t addr) {
  softUartIdle();
  delay(2);

  if (digitalRead(tmcRxPin) == LOW) {
    Serial.print(F("busLOW "));
    return 0x00;
  }

  uint8_t req[4];
  req[0] = 0x05;
  req[1] = (uint8_t)(addr & 0x03);
  req[2] = 0x06;
  req[3] = tmcCrc(req, 3);

  softUartWriteBytes(req, 4);

  {
    const uint32_t t0 = millis();
    while (digitalRead(tmcRxPin) == LOW) {
      if (millis() - t0 > 20) {
        Serial.print(F("stuckLOW "));
        return 0x00;
      }
    }
    delayMicroseconds(100);
  }

  // After sync 0x05, grab the rest with IRQs off. Prefer header 05 FF (reply),
  // but if we only see echo (05 <addr>), keep hunting.
  uint8_t resp[8];
  bool got = false;
  const uint32_t huntStart = millis();
  while (millis() - huntStart < 200) {
    uint8_t b0 = 0;
    if (!softUartReadByte(b0, 40, true) || b0 != 0x05) {
      continue;
    }

    noInterrupts();
    resp[0] = 0x05;
    uint8_t n = 1;
    for (; n < 8; n++) {
      if (!softUartReadByteUnlocked(resp[n], true)) {
        break;
      }
    }
    interrupts();

    if (n < 8) {
      Serial.print(F("short"));
      Serial.print(n);
      Serial.print(F(" "));
      continue;
    }
    if (resp[1] != 0xFF) {
      Serial.print(F("echo? "));
      continue;  // likely TX echo 05 00 06 ..., not slave reply
    }
    got = true;
    break;
  }

  if (!got) {
    Serial.print(F("noSync "));
    return 0x00;
  }

  Serial.print(F("rx8b"));
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(F(" "));
    Serial.print(resp[i], HEX);
  }
  Serial.print(F(" "));

  const uint8_t crc = tmcCrc(resp, 7);
  if (crc != resp[7]) {
    Serial.print(F("badCRC(calc="));
    Serial.print(crc, HEX);
    Serial.print(F(") "));
    return 0x00;
  }

  return resp[6];
}

void testUart() {
  Serial.println(F("t: soft UART only @9600 (no HW Serial1 — avoids hang)"));

  stopUartPort();

  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  delay(5);
  const int a = digitalRead(PIN_A);
  const int b = digitalRead(PIN_B);
  const int txLevel = (tmcTxPin == PIN_A) ? a : b;

  Serial.print(F("t: TX=GP"));
  Serial.print(tmcTxPin);
  Serial.print(F(" RX=GP"));
  Serial.print(tmcRxPin);
  Serial.print(F(" "));
  Serial.print(a == HIGH ? F("H") : F("L"));
  Serial.print(F("/"));
  Serial.println(b == HIGH ? F("H") : F("L"));

  if (txLevel == LOW) {
    Serial.println(F("t: ABORT — TX pin LOW."));
    return;
  }

  softUartIdle();
  int found = -1;
  uint8_t foundVer = 0;
  for (uint8_t addr = 0; addr < 4; addr++) {
    Serial.print(F("  addr "));
    Serial.print(addr);
    Serial.print(F(": "));

    const uint8_t ver = probeVersionSoft(addr);
    Serial.print(F("ver=0x"));
    Serial.print(ver, HEX);

    if (ver != 0x00) {
      Serial.println(F(" OK"));
      found = addr;
      foundVer = ver;
      break;
    }
    Serial.println(F(" fail"));
  }

  pinMode(tmcTxPin, INPUT_PULLUP);
  pinMode(tmcRxPin, INPUT_PULLUP);

  if (found >= 0) {
    recreateDriver((uint8_t)found);
    Serial.print(F("t: OK addr="));
    Serial.print(found);
    Serial.print(F(" chip_ver=0x"));
    Serial.println(foundVer, HEX);
  } else {
    Serial.println(F("t: fail — do not run u yet"));
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

  Serial.print(F("=== FW "));
  Serial.print(FW_VERSION);
  Serial.println(F(" ==="));
  Serial.println(F("Type v anytime to print firmware version."));
  Serial.println(F("UART: GP8-[1k]-UART_pin, GP9 same node, pull-up 1k-2.2k to 3.3V"));
  Serial.println(F("t: soft@9600 only (v23)."));
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
  } else if (c == 'v' || c == 'V') {
    Serial.print(F("firmware="));
    Serial.println(FW_VERSION);
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

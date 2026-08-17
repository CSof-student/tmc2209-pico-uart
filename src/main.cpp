/*
  Pico + TMC2209 + FastAccelStepper

  All register access goes through TMCStepper. The Stream under it is a
  half-duplex soft UART that releases TX after each datagram (SoftwareSerial /
  Serial1 keep TX driven and often get no TMC reply on this single-wire bus).

    TmcSoftUart SerialTMC(8, 9);
    TMC2209Stepper driver(&SerialTMC, R_SENSE, ADDR);
    SerialTMC.begin(9600);
    driver.begin();
    driver.version(); / rms_current() / SGTHRS() / SG_RESULT() / ...

  Wiring:
    GP8 --[1k]--+---- TMC UART pin
    GP9 --------+
    STEP GP3, DIR GP2, EN->GND, VIO 3.3V, VM motor PSU

  Physical: + retracts, - extends. After H: 0 = retracted, travelMax = extended.
*/

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include "TmcSoftUart.h"

static const char *FW_VERSION = "tmc-stepper-uart-v5";

static const uint8_t STEP_PIN = 3;
static const uint8_t DIR_PIN = 2;
static const int8_t ENABLE_PIN = -1;

static const uint8_t TMC_TX_PIN = 8;
static const uint8_t TMC_RX_PIN = 9;
static const float R_SENSE = 0.11f;
static const uint8_t DRIVER_ADDRESS = 0b00;
static const uint32_t TMC_BAUD = 9600;

static const uint16_t DEFAULT_RMS_MA = 300;
static const uint16_t DEFAULT_MICROSTEPS = 16;
static const uint32_t DEFAULT_SPEED_HZ = 200;
static const uint32_t DEFAULT_ACCEL = 400;
static const int32_t DEFAULT_STEP_SIZE = 200;

TmcSoftUart SerialTMC(TMC_TX_PIN, TMC_RX_PIN);
TMC2209Stepper driver(&SerialTMC, R_SENSE, DRIVER_ADDRESS);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

int32_t stepSize = DEFAULT_STEP_SIZE;
uint16_t rmsMa = DEFAULT_RMS_MA;
bool uartReady = false;
bool driverConfigured = false;
String line;

uint8_t sgThreshold = 40;
bool travelCalibrated = false;
int32_t travelMin = 0;
int32_t travelMax = 0;

static const int32_t HOME_BACKOFF = 48;
static const int32_t HOME_CHUNK = 40;
static const int32_t HOME_MAX_TRAVEL = 20000;
static const int32_t HOME_MIN_TRAVEL = 200;
static const uint8_t HOME_IGNORE_CHUNKS = 4;
static const uint8_t HOME_STALL_HITS = 2;
static const uint32_t HOME_SPEED_HZ = 500;
static const uint32_t HOME_ACCEL = 800;

uint32_t moveSpeedHz = DEFAULT_SPEED_HZ;
int32_t moveAccel = DEFAULT_ACCEL;

bool beginTmcUart() {
  pinMode(TMC_TX_PIN, INPUT_PULLUP);
  pinMode(TMC_RX_PIN, INPUT_PULLUP);
  delay(5);
  if (digitalRead(TMC_TX_PIN) == LOW) {
    Serial.println(F("TX pin LOW — check wiring"));
    return false;
  }

  SerialTMC.begin(TMC_BAUD);
  uartReady = true;
  return true;
}

void loopbackCommand() {
  Serial.println(F("b: loopback (TMC UART disconnected, jumper GP8-GP9)"));
  if (SerialTMC.loopbackTest(0xA5)) {
    Serial.println(F("b: OK (got 0xA5)"));
  } else {
    Serial.println(F("b: FAIL"));
  }
  // Restore port for TMCStepper
  SerialTMC.begin(TMC_BAUD);
  uartReady = true;
}

bool probeDriver() {
  Serial.println(F("t: raw IOIN then TMCStepper.version()"));
  if (!uartReady && !beginTmcUart()) {
    return false;
  }

  uint8_t raw[8];
  bool crcOk = false;
  if (!SerialTMC.rawIoinProbe(DRIVER_ADDRESS, raw, crcOk)) {
    Serial.println(F("  raw: no 05 FF frame"));
  } else {
    Serial.print(F("  raw:"));
    for (uint8_t i = 0; i < 8; i++) {
      Serial.print(F(" "));
      Serial.print(raw[i], HEX);
    }
    Serial.println(crcOk ? F("  CRC OK") : F("  badCRC"));
    Serial.println(F("  expect: 5 FF 6 … 21 …"));
  }

  // Re-begin so TMCStepper datagram state is clean after raw probe
  SerialTMC.begin(TMC_BAUD);

  uint8_t ver = 0;
  for (uint8_t i = 0; i < 5; i++) {
    ver = driver.version();
    Serial.print(F("  try "));
    Serial.print(i + 1);
    Serial.print(F(": 0x"));
    Serial.println(ver, HEX);
    if (ver == 0x21 || ver == 0x20) {
      return true;
    }
    delay(20);
  }
  return false;
}

void applyDriverDefaults() {
  if (!uartReady && !beginTmcUart()) {
    return;
  }

  Serial.println(F("Applying TMC defaults (TMCStepper)..."));
  driver.begin();
  driver.pdn_disable(true);
  driver.I_scale_analog(false);
  driver.mstep_reg_select(true);
  driver.multistep_filt(true);
  driver.en_spreadCycle(false);  // stealthChop required for StallGuard4
  driver.TPWMTHRS(0);            // stay in stealthChop at all speeds
  driver.rms_current(rmsMa);
  driver.microsteps(DEFAULT_MICROSTEPS);
  driver.toff(5);
  // Same PWMCONF as working main soft-UART path (autoscale + autograd)
  driver.PWMCONF(0xC10D0024UL);
  driver.TCOOLTHRS(0xFFFFF);  // StallGuard enabled for practical TSTEP values
  driver.SGTHRS(sgThreshold);

  driverConfigured = true;
  Serial.print(F("Done  rms="));
  Serial.print(rmsMa);
  Serial.print(F("mA  SGTHRS="));
  Serial.print(sgThreshold);
  Serial.print(F("  spread="));
  Serial.print(driver.en_spreadCycle() ? F("on") : F("off"));
  Serial.print(F("  stealth="));
  Serial.println(driver.stealth() ? F("yes") : F("no"));
}

void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  h/? help   v version   t UART probe   b loopback   u defaults"));
  Serial.println(F("  c <mA>  m <usteps>  i status"));
  Serial.println(F("  +/- nudge (+retract -extend)  n size  s Hz  a accel  g <pos>  x stop"));
  Serial.println(F("  z SG read   y <0-255> SG thresh   d SG diag move   H home ends"));
}

void printMotionStatus() {
  Serial.print(F("pos="));
  Serial.print(stepper ? stepper->getCurrentPosition() : 0);
  Serial.print(F("  stepSize="));
  Serial.print(stepSize);
  Serial.print(F("  rms="));
  Serial.print(rmsMa);
  Serial.print(F("  cfg="));
  Serial.println(driverConfigured ? F("yes") : F("no"));
  if (travelCalibrated) {
    Serial.print(F("  travel=0.."));
    Serial.println(travelMax);
  }
}

void waitStepperIdle() {
  if (!stepper) {
    return;
  }
  while (stepper->isRunning()) {
    delay(1);
  }
}

uint16_t readStallGuard() {
  if (!uartReady) {
    return 0xFFFF;
  }
  driver.CRCerror = false;
  const uint16_t v = (uint16_t)(driver.SG_RESULT() & 0x3FF);
  if (driver.CRCerror) {
    return 0xFFFF;
  }
  return v;
}

// Live StallGuard tune: move while printing SG / TSTEP / mode.
// Free motion should show SG well above SGTHRS (often 50–400+), not 0–2.
void diagStallGuard() {
  if (!stepper) {
    Serial.println(F("no stepper"));
    return;
  }
  if (!driverConfigured) {
    Serial.println(F("run u first"));
    return;
  }

  driver.TCOOLTHRS(0xFFFFF);
  driver.TPWMTHRS(0);
  driver.en_spreadCycle(false);
  driver.SGTHRS(sgThreshold);

  Serial.println(F("d: move + poll SG (should rise while free-running)"));
  Serial.print(F("  SGTHRS="));
  Serial.print(sgThreshold);
  Serial.print(F("  stealth="));
  Serial.print(driver.stealth() ? F("1") : F("0"));
  Serial.print(F("  spreadIO="));
  Serial.println(driver.spread_en() ? F("1") : F("0"));

  stepper->setSpeedInHz(HOME_SPEED_HZ);
  stepper->setAcceleration(HOME_ACCEL);
  stepper->move(-800);  // extend a bit

  uint8_t n = 0;
  while (stepper->isRunning() && n < 40) {
    driver.CRCerror = false;
    const uint16_t sg = (uint16_t)(driver.SG_RESULT() & 0x3FF);
    const bool crcFail = driver.CRCerror;
    const uint32_t tstep = driver.TSTEP();
    Serial.print(F("  sg="));
    Serial.print(sg);
    Serial.print(F(" tstep="));
    Serial.print(tstep);
    Serial.print(F(" stealth="));
    Serial.print(driver.stealth() ? F("1") : F("0"));
    if (crcFail) {
      Serial.print(F(" CRC_FAIL"));
    }
    Serial.println();
    n++;
    delay(30);
  }
  waitStepperIdle();
  stepper->setSpeedInHz(moveSpeedHz);
  stepper->setAcceleration(moveAccel);
  Serial.println(F("d: done — if sg stays 0..2 and tstep=1048575, steps aren't seen / not stealthChop"));
}

// Poll SG_RESULT while stepping. StallGuard is invalid when standing still
// (idle reads often 0..2) — that was why mid-test looked stuck low.
bool moveUntilStall(int dirSign, int32_t maxSteps) {
  if (!stepper || maxSteps <= 0) {
    return false;
  }

  stepper->setSpeedInHz(HOME_SPEED_HZ);
  stepper->setAcceleration(HOME_ACCEL);

  int32_t moved = 0;
  uint8_t chunkIdx = 0;
  uint8_t lowHits = 0;
  bool stalled = false;
  delay(150);

  while (moved < maxSteps) {
    const int32_t chunk =
        (maxSteps - moved > HOME_CHUNK) ? HOME_CHUNK : (maxSteps - moved);
    const int32_t pos0 = stepper->getCurrentPosition();
    stepper->move((int32_t)dirSign * chunk);

    uint16_t sgMin = 0xFFFF;
    while (stepper->isRunning()) {
      const uint16_t sg = readStallGuard();
      if (sg != 0xFFFF && sg < sgMin) {
        sgMin = sg;
      }
      delay(4);
    }

    const int32_t stepped = abs(stepper->getCurrentPosition() - pos0);
    moved += (stepped > 0) ? stepped : chunk;

    Serial.print(F("  sgMin="));
    if (sgMin == 0xFFFF) {
      Serial.print(F("crcFail"));
    } else {
      Serial.print(sgMin);
    }
    Serial.print(F(" pos="));
    Serial.print(stepper->getCurrentPosition());

    if (chunkIdx < HOME_IGNORE_CHUNKS) {
      Serial.println(F(" (ignore)"));
      lowHits = 0;
    } else if (sgMin != 0xFFFF && sgMin <= (uint16_t)sgThreshold) {
      lowHits++;
      Serial.print(F(" low "));
      Serial.print(lowHits);
      Serial.print(F("/"));
      Serial.println(HOME_STALL_HITS);
      if (lowHits >= HOME_STALL_HITS) {
        stalled = true;
        break;
      }
    } else {
      lowHits = 0;
      Serial.println();
    }

    chunkIdx++;
  }

  if (stalled) {
    stepper->move((int32_t)(-dirSign) * HOME_BACKOFF);
    waitStepperIdle();
    delay(100);
  }

  stepper->setSpeedInHz(moveSpeedHz);
  stepper->setAcceleration(moveAccel);
  return stalled;
}

void homeBothEnds() {
  if (!stepper) {
    Serial.println(F("no stepper"));
    return;
  }
  if (!driverConfigured) {
    Serial.println(F("run u first"));
    return;
  }

  Serial.println(F("StallGuard home (+retract / -extend)..."));
  driver.TCOOLTHRS(0xFFFFF);
  driver.SGTHRS(sgThreshold);
  Serial.print(F("SGTHRS="));
  Serial.println(sgThreshold);

  Serial.println(F("Seeking RETRACTED (+) ..."));
  if (!moveUntilStall(+1, HOME_MAX_TRAVEL)) {
    Serial.println(F("RETRACT: no stall — try higher y"));
    travelCalibrated = false;
    return;
  }
  stepper->setCurrentPosition(0);
  travelMin = 0;
  Serial.println(F("Retracted end = 0"));
  delay(100);

  Serial.println(F("Seeking EXTENDED (-) ..."));
  if (!moveUntilStall(-1, HOME_MAX_TRAVEL)) {
    Serial.println(F("EXTEND: no stall — try higher y"));
    travelCalibrated = false;
    return;
  }

  const int32_t raw = stepper->getCurrentPosition();
  travelMax = -raw;
  Serial.print(F("  rawPos="));
  Serial.print(raw);
  Serial.print(F(" → travelMax="));
  Serial.println(travelMax);

  if (travelMax < HOME_MIN_TRAVEL) {
    Serial.println(F("Travel too short — lower y, then H"));
    travelCalibrated = false;
    return;
  }
  stepper->setCurrentPosition(travelMax);
  travelCalibrated = true;
  Serial.print(F("Travel 0 (retract) .. "));
  Serial.print(travelMax);
  Serial.println(F(" (extend)"));
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    return;
  }
  const char c = cmd.charAt(0);

  if (c == 'h' || c == '?') {
    printHelp();
  } else if (c == 'H') {
    homeBothEnds();
  } else if (c == 'v' || c == 'V') {
    Serial.print(F("firmware="));
    Serial.println(FW_VERSION);
  } else if (c == 'b' || c == 'B') {
    loopbackCommand();
  } else if (c == 'd' || c == 'D') {
    diagStallGuard();
  } else if (c == 't' || c == 'T') {
    if (probeDriver()) {
      Serial.println(F("t: OK"));
    } else {
      Serial.println(F("t: FAIL — see raw rx line (want 05 FF 06 …)"));
    }
  } else if (c == 'u' || c == 'U') {
    applyDriverDefaults();
  } else if (c == 'p' || c == 'P') {
    printMotionStatus();
  } else if (c == 'z' || c == 'Z') {
    const bool moving = stepper && stepper->isRunning();
    Serial.print(F("SG_RESULT="));
    Serial.print(readStallGuard());
    Serial.print(F("  SGTHRS="));
    Serial.print(sgThreshold);
    Serial.println(moving ? F("  (moving)") : F("  (idle — SG often ~0; use H for live sgMin)"));
  } else if (c == 'y' || c == 'Y') {
    const int v = cmd.substring(1).toInt();
    if (v >= 0 && v <= 255) {
      sgThreshold = (uint8_t)v;
      if (uartReady) {
        driver.SGTHRS(sgThreshold);
      }
      Serial.print(F("SGTHRS="));
      Serial.println(sgThreshold);
    }
  } else if (c == 'g' || c == 'G') {
    int32_t dest = cmd.substring(1).toInt();
    if (!stepper) {
      return;
    }
    if (travelCalibrated) {
      if (dest < travelMin) dest = travelMin;
      if (dest > travelMax) dest = travelMax;
    }
    stepper->moveTo(dest);
    Serial.print(F("goto "));
    Serial.println(dest);
  } else if (c == '+' || c == '-') {
    int32_t delta = (c == '+') ? -stepSize : stepSize;
    if (cmd.length() > 1) {
      int32_t mag = cmd.substring(1).toInt();
      if (mag < 0) mag = -mag;
      delta = (c == '+') ? -mag : mag;
    }
    if (stepper) {
      int32_t dest = stepper->getCurrentPosition() + delta;
      if (travelCalibrated) {
        if (dest < travelMin) dest = travelMin;
        if (dest > travelMax) dest = travelMax;
        delta = dest - stepper->getCurrentPosition();
      }
      if (delta != 0) {
        stepper->move(delta);
      }
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
      moveSpeedHz = v;
      stepper->setSpeedInHz(v);
    }
  } else if (c == 'a' || c == 'A') {
    const int32_t v = cmd.substring(1).toInt();
    if (v > 0 && stepper) {
      moveAccel = v;
      stepper->setAcceleration(v);
    }
  } else if (c == 'c' || c == 'C') {
    const int v = cmd.substring(1).toInt();
    if (v > 0 && v < 2000 && uartReady) {
      rmsMa = (uint16_t)v;
      driver.rms_current(rmsMa);
      Serial.print(F("rms_mA="));
      Serial.println(rmsMa);
    }
  } else if (c == 'm' || c == 'M') {
    const int v = cmd.substring(1).toInt();
    if ((v == 8 || v == 16 || v == 32 || v == 64 || v == 128 || v == 256) &&
        uartReady) {
      driver.microsteps(v);
      Serial.println(v);
    }
  } else if (c == 'i' || c == 'I') {
    printMotionStatus();
    if (uartReady) {
      Serial.print(F("ver=0x"));
      Serial.println(driver.version(), HEX);
    }
  } else if (c == 'x' || c == 'X') {
    if (stepper) {
      stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
    }
  } else {
    Serial.println(F("unknown (h)"));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  delay(200);

  Serial.print(F("=== FW "));
  Serial.print(FW_VERSION);
  Serial.println(F(" ==="));
  Serial.println(F("TMCStepper registers + half-duplex Stream @9600"));

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

  beginTmcUart();
  printHelp();
  Serial.println(F("Start with: t   then u"));
  Serial.print(F("> "));
}

void loop() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (line.length() > 0) {
        processCommand(line);
        line = "";
        Serial.print(F("> "));
      }
    } else {
      line += ch;
    }
  }
}

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

static const char *FW_VERSION = "tmc-stepper-uart-v23";

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

// Classic StallGuard polarity (datasheet):
//   free / mid-travel → high SG
//   blocked / end      → low SG
// Homing trips in software when SG <= y (same sense as SGTHRS).
uint8_t sgThreshold = 6;  // stall when SG drops to this (below freeMin, above blockMax)
bool travelCalibrated = false;
int32_t travelMin = 0;
int32_t travelMax = 0;

static const int32_t HOME_BACKOFF = 80;
static const int32_t HOME_CHUNK = 60;
static const int32_t HOME_MAX_TRAVEL = 20000;
static const int32_t HOME_MIN_TRAVEL = 200;
static const uint8_t HOME_IGNORE_CHUNKS = 8;
static const uint8_t HOME_STALL_HITS = 3;
static const uint16_t HOME_ARM_SG = 15;  // arm after seeing free (high) SG >= this
static const uint32_t HOME_SPEED_HZ = 400;  // slower = cleaner soft-UART SG reads
static const uint32_t HOME_ACCEL = 600;

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
  driver.internal_Rsense(false);
  driver.mstep_reg_select(true);
  driver.multistep_filt(true);
  driver.VACTUAL(0);
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(rmsMa);
  driver.microsteps(DEFAULT_MICROSTEPS);
  driver.PWMCONF(0xC10D0024UL);
  driver.pwm_autoscale(true);
  // Datasheet: TPWMTHRS=0 → StealthChop only (do NOT use 1 here)
  driver.TPWMTHRS(0);
  driver.TCOOLTHRS(0xFFFFF);
  driver.semin(0);
  driver.SGTHRS(sgThreshold);
  // SPREAD pin HIGH inverts en_spreadCycle. reg==pin → effective stealthChop
  const bool spreadPin = driver.spread_en();
  driver.en_spreadCycle(spreadPin);
  delay(150);  // stealthChop must engage at standstill first

  driverConfigured = true;
  Serial.print(F("Done  rms="));
  Serial.print(rmsMa);
  Serial.print(F("mA  SPREAD_pin="));
  Serial.print(spreadPin ? 1 : 0);
  Serial.print(F("  en_spread_reg="));
  Serial.print(spreadPin ? 1 : 0);
  Serial.print(F("  DRV.stealth="));
  Serial.println(driver.stealth() ? 1 : 0);
}

void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  h/? help   v version   t UART probe   b loopback   u defaults"));
  Serial.println(F("  c <mA>  m <usteps>  i status"));
  Serial.println(F("  +/- nudge (+retract -extend)  n size  s Hz  a accel  g <pos>  x stop"));
  Serial.println(F("  z SG read   y <N> stall if SG<=N (classic)   d FREE/BLOCK   H home"));
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
  // Soft UART drops CRCs while STEP is noisy — retry before treating as fail.
  for (uint8_t attempt = 0; attempt < 8; attempt++) {
    driver.CRCerror = false;
    const uint16_t v = (uint16_t)(driver.SG_RESULT() & 0x3FF);
    if (!driver.CRCerror) {
      return v;
    }
    delay(3);
  }
  return 0xFFFF;
}

void applyStallGuardMode() {
  driver.VACTUAL(0);
  driver.TPWMTHRS(0);  // stealthChop only (datasheet)
  driver.PWMCONF(0xC10D0024UL);
  driver.pwm_autoscale(true);
  driver.TCOOLTHRS(0xFFFFF);
  driver.semin(0);
  driver.SGTHRS(sgThreshold);
  const bool spreadPin = driver.spread_en();
  driver.en_spreadCycle(spreadPin);  // compensate SPREAD pin invert
  delay(100);
}

// One SG sample with CRC labeling + motion context.
struct SgSample {
  uint16_t sg;  // 0xFFFF = CRC fail
  uint32_t tstep;
  uint32_t sgRaw;
  uint8_t pwmScale;
  bool stealth;
  bool spread;
  bool ola;
  bool olb;
};

SgSample sampleSgDetailed() {
  SgSample s{};
  s.sg = 0xFFFF;
  s.tstep = 0xFFFFFFFFUL;
  s.sgRaw = 0;
  s.pwmScale = 0;
  s.stealth = false;
  s.spread = false;
  s.ola = false;
  s.olb = false;

  for (uint8_t attempt = 0; attempt < 6; attempt++) {
    driver.CRCerror = false;
    s.sgRaw = driver.SG_RESULT();
    if (!driver.CRCerror) {
      s.sg = (uint16_t)(s.sgRaw & 0x3FF);
      break;
    }
    delay(3);
  }
  if (s.sg == 0xFFFF) {
    return s;
  }

  driver.CRCerror = false;
  s.tstep = driver.TSTEP();
  if (driver.CRCerror) {
    s.tstep = 0xFFFFFFFFUL;
  }
  s.stealth = driver.stealth();
  s.spread = driver.spread_en();
  s.ola = driver.ola();
  s.olb = driver.olb();
  s.pwmScale = driver.pwm_scale_sum();
  return s;
}

void printSgSample(const SgSample &s) {
  Serial.print(F("  "));
  if (s.sg == 0xFFFF) {
    Serial.println(F("SG=CRC_FAIL"));
    return;
  }
  Serial.print(F("SG="));
  Serial.print(s.sg);
  Serial.print(F(" raw=0x"));
  Serial.print(s.sgRaw, HEX);
  Serial.print(F(" tstep="));
  if (s.tstep == 0xFFFFFFFFUL) {
    Serial.print(F("?"));
  } else {
    Serial.print(s.tstep);
  }
  Serial.print(F(" stealth="));
  Serial.print(s.stealth ? 1 : 0);
  Serial.print(F(" pwmScale="));
  Serial.print(s.pwmScale);
  Serial.print(F(" ol="));
  Serial.print(s.ola ? 1 : 0);
  Serial.print(s.olb ? 1 : 0);
  Serial.println();
}

// Poll SG while moving `delta` steps.
void runSgPhase(const __FlashStringHelper *name, int32_t delta, uint8_t maxSamples,
                uint16_t &outMax, uint16_t &outMin, uint8_t &outOk, uint8_t &outFail) {
  outMax = 0;
  outMin = 0xFFFF;
  outOk = 0;
  outFail = 0;
  Serial.println(name);

  stepper->move(delta);
  delay(40);

  uint8_t n = 0;
  while (n < maxSamples && stepper->isRunning()) {
    const uint16_t sg = readStallGuard();
    Serial.print(F("  SG="));
    if (sg == 0xFFFF) {
      Serial.println(F("CRC_FAIL"));
      outFail++;
    } else {
      Serial.println(sg);
      outOk++;
      if (sg > outMax) {
        outMax = sg;
      }
      if (sg < outMin) {
        outMin = sg;
      }
    }
    n++;
    delay(8);
  }
  waitStepperIdle();
  Serial.print(F("  → sg="));
  Serial.print(outMin == 0xFFFF ? 0 : outMin);
  Serial.print(F(".."));
  Serial.println(outMax);
}

// Classic StallGuard: free = high SG, block = low SG.
// Probe both directions for free, then block the better free direction.
void diagStallGuard() {
  if (!stepper) {
    Serial.println(F("no stepper"));
    return;
  }
  if (!driverConfigured) {
    Serial.println(F("run u first"));
    return;
  }

  applyStallGuardMode();
  const uint16_t oldMa = rmsMa;
  if (rmsMa < 400) {
    driver.rms_current(400);
  }
  driver.microsteps(8);
  applyStallGuardMode();

  // Absolute FAS: same as H. + = tip IN, - = tip OUT.
  const int32_t TIP_OUT = -1200;
  const int32_t TIP_IN = 600;

  Serial.println(F("d: classic SG — FREE should be HIGH, BLOCK low"));
  Serial.println(F("  Start mid-travel. Tip must be free both ways."));

  stepper->setSpeedInHz(400);
  stepper->setAcceleration(600);

  Serial.println(F("  (prep) tip IN..."));
  stepper->move(TIP_IN);
  waitStepperIdle();
  delay(150);

  uint16_t freeOutMax = 0, freeOutMin = 0xFFFF;
  uint8_t freeOutOk = 0, freeOutFail = 0;
  Serial.println(F(">>> FREE tip OUT (do not touch)"));
  runSgPhase(F("--- FREE tip OUT ---"), TIP_OUT, 40, freeOutMax, freeOutMin, freeOutOk,
             freeOutFail);

  uint16_t freeInMax = 0, freeInMin = 0xFFFF;
  uint8_t freeInOk = 0, freeInFail = 0;
  Serial.println(F(">>> FREE tip IN (do not touch)"));
  runSgPhase(F("--- FREE tip IN ---"), TIP_IN, 40, freeInMax, freeInMin, freeInOk,
             freeInFail);

  const bool preferOut = (freeOutMax >= freeInMax);
  const uint16_t freeMax = preferOut ? freeOutMax : freeInMax;
  const uint16_t freeMin = preferOut
                               ? (freeOutMin == 0xFFFF ? 0 : freeOutMin)
                               : (freeInMin == 0xFFFF ? 0 : freeInMin);
  const int32_t blockDir = preferOut ? TIP_OUT : TIP_IN;
  const uint8_t freeOk = preferOut ? freeOutOk : freeInOk;

  Serial.print(F("  best free dir="));
  Serial.print(preferOut ? F("OUT") : F("IN"));
  Serial.print(F(" sg="));
  Serial.print(freeMin);
  Serial.print(F(".."));
  Serial.println(freeMax);

  if (freeMax < 15) {
    Serial.println(F("  Free SG still low both ways — not mid-travel / jammed."));
    Serial.println(F("  Jog +/- until tip moves freely, then d again."));
    driver.microsteps(DEFAULT_MICROSTEPS);
    driver.rms_current(oldMa);
    stepper->setSpeedInHz(moveSpeedHz);
    stepper->setAcceleration(moveAccel);
    applyStallGuardMode();
    return;
  }

  Serial.println(F("  (prep) backoff..."));
  stepper->move(-blockDir / 2);
  waitStepperIdle();

  Serial.println(F("--- Finger on TIP. 2s — resist the next move ---"));
  delay(2000);

  Serial.println(F(">>> BLOCK (hold tip)"));
  uint16_t blkMax = 0, blkMin = 0xFFFF;
  uint8_t blkOk = 0, blkFail = 0;
  runSgPhase(F("--- BLOCK ---"), blockDir, 40, blkMax, blkMin, blkOk, blkFail);

  driver.microsteps(DEFAULT_MICROSTEPS);
  driver.rms_current(oldMa);
  stepper->setSpeedInHz(moveSpeedHz);
  stepper->setAcceleration(moveAccel);
  applyStallGuardMode();

  Serial.println(F("=== SUMMARY (classic: free HIGH, block LOW) ==="));
  Serial.print(F("  FREE  sg="));
  Serial.print(freeMin);
  Serial.print(F(".."));
  Serial.println(freeMax);
  Serial.print(F("  BLOCK sg="));
  Serial.print(blkMin == 0xFFFF ? 0 : blkMin);
  Serial.print(F(".."));
  Serial.println(blkMax);

  if (freeOk < 5 || blkOk < 5) {
    Serial.println(F("  Verdict: too few samples"));
  } else if (freeMax > blkMax + 8) {
    int y = (int)((blkMax + freeMin) / 2);
    if (y < 2) {
      y = 2;
    }
    if (y >= (int)freeMin) {
      y = (int)freeMin - 2;
    }
    if (y < 1) {
      y = 1;
    }
    Serial.print(F("  Verdict: OK — stall when SG drops. try y "));
    Serial.println(y);
    Serial.println(F("  Then H from mid-travel."));
  } else if (blkMax >= freeMax) {
    Serial.println(F("  Verdict: BLOCK not lower — hold tip firmer, or free was still loaded"));
  } else {
    Serial.println(F("  Verdict: weak contrast — rerun from true mid-travel"));
  }
}

// Classic StallGuard: arm on free (high SG), stall when SG <= y.
bool moveUntilStall(int dirSign, int32_t maxSteps) {
  if (!stepper || maxSteps <= 0) {
    return false;
  }

  applyStallGuardMode();
  stepper->setSpeedInHz(HOME_SPEED_HZ);
  stepper->setAcceleration(HOME_ACCEL);

  int32_t moved = 0;
  uint8_t chunkIdx = 0;
  uint8_t lowHits = 0;
  bool armed = false;
  bool stalled = false;
  uint16_t seenMax = 0;
  uint16_t goodReads = 0;
  uint16_t badReads = 0;
  delay(150);

  while (moved < maxSteps) {
    const int32_t chunk =
        (maxSteps - moved > HOME_CHUNK) ? HOME_CHUNK : (maxSteps - moved);
    const int32_t pos0 = stepper->getCurrentPosition();
    stepper->move((int32_t)dirSign * chunk);

    uint16_t sgMin = 0xFFFF;
    uint16_t sgMaxChunk = 0;
    uint8_t chunkGood = 0;
    uint8_t chunkBad = 0;
    while (stepper->isRunning()) {
      const uint16_t sg = readStallGuard();
      if (sg == 0xFFFF) {
        chunkBad++;
        badReads++;
      } else {
        chunkGood++;
        goodReads++;
        if (sg < sgMin) {
          sgMin = sg;
        }
        if (sg > sgMaxChunk) {
          sgMaxChunk = sg;
        }
        if (sg > seenMax) {
          seenMax = sg;
        }
        if (!armed && sg >= HOME_ARM_SG) {
          armed = true;
        }
      }
      delay(20);
    }

    const int32_t stepped = abs(stepper->getCurrentPosition() - pos0);
    moved += (stepped > 0) ? stepped : chunk;

    Serial.print(F("  sg="));
    if (chunkGood == 0) {
      Serial.print(F("noRead"));
    } else {
      Serial.print(sgMin);
      Serial.print(F(".."));
      Serial.print(sgMaxChunk);
    }
    Serial.print(F(" ok/fail="));
    Serial.print(chunkGood);
    Serial.print(F("/"));
    Serial.print(chunkBad);
    Serial.print(F(" maxSeen="));
    Serial.print(seenMax);
    Serial.print(F(" pos="));
    Serial.print(stepper->getCurrentPosition());

    if (badReads > 30 && goodReads * 2 < badReads) {
      Serial.println(F("  ABORT: UART SG reads failing — not safe to home"));
      stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
      stepper->setSpeedInHz(moveSpeedHz);
      stepper->setAcceleration(moveAccel);
      return false;
    }

    if (chunkIdx < HOME_IGNORE_CHUNKS || !armed) {
      Serial.println(armed ? F(" (ignore)") : F(" (arming)"));
      lowHits = 0;
    } else if (chunkGood > 0 && sgMin <= (uint16_t)sgThreshold) {
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

  if (!stalled && !armed) {
    Serial.print(F("  never armed (sg max seen "));
    Serial.print(seenMax);
    Serial.println(F(") — need free (high) SG first; start mid-travel / check stealth"));
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
  Serial.println(F("  Classic SG: free=high, end=low. Stall when SG <= y."));
  Serial.println(F("  Start mid-travel. Run d first to pick y."));
  applyStallGuardMode();
  Serial.print(F("SGTHRS/y="));
  Serial.println(sgThreshold);

  Serial.println(F("Seeking RETRACTED (+) ..."));
  if (!moveUntilStall(+1, HOME_MAX_TRAVEL)) {
    Serial.println(F("RETRACT: no stall — try higher y / start mid-travel"));
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
    Serial.println(F("Travel too short — lower y (less sensitive), then H"));
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
    Serial.println(moving ? F("  (moving)") : F("  (idle — SG often ~0; sample while moving)"));
  } else if (c == 'y' || c == 'Y') {
    const int v = cmd.substring(1).toInt();
    if (v >= 0 && v <= 255) {
      sgThreshold = (uint8_t)v;
      if (uartReady) {
        driver.SGTHRS(sgThreshold);
      }
      Serial.print(F("SGTHRS="));
      Serial.print(sgThreshold);
      Serial.println(F("  (classic; stall when SG drops <= y)"));
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

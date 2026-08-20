/*
  TMC2209 Serial2 UART + FastAccelStepper + StallGuard end-finding.

  GP8/GP9 are UART1 → Serial2 (Serial1 is UART0 and will hang on these pins).

  Wiring:
    GP8 --[1k]--+---- TMC UART
    GP9 --------+
    STEP GP3, DIR GP2, EN -> GND
    VIO 3.3V, VM motor PSU, GND common

  Linear actuator: + retracts (toward 0), - extends (toward travelMax).
  USB 115200.
*/

#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

#define DIR_PIN    2
#define STEP_PIN   3
#define ENABLE_PIN -1
#define TX_PIN     8
#define RX_PIN     9

#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b00
#define TMC_SERIAL Serial2

static const uint16_t DEFAULT_RMS_MA = 300;
static const uint16_t DEFAULT_MICROSTEPS = 16;
static const uint32_t DEFAULT_SPEED_HZ = 200;
static const uint32_t DEFAULT_ACCEL = 400;
static const int32_t DEFAULT_STEP_SIZE = 200;

static const int32_t HOME_BACKOFF = 80;
static const int32_t HOME_MAX_TRAVEL = 20000;
static const uint32_t HOME_SPEED_HZ = 2500;
static const uint32_t HOME_ACCEL = 20000;
static const uint8_t STALL_CONFIRM = 3;
static const uint32_t TCOOLTHRS_SETTING = 400;  // tune after measuring TSTEP at steady homing speed

TMC2209Stepper driver(&TMC_SERIAL, R_SENSE, DRIVER_ADDRESS);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

int32_t stepSize = DEFAULT_STEP_SIZE;
uint16_t rmsMa = DEFAULT_RMS_MA;
uint16_t motor_microsteps = DEFAULT_MICROSTEPS;
uint32_t moveSpeedHz = DEFAULT_SPEED_HZ;
int32_t moveAccel = DEFAULT_ACCEL;
uint8_t sgThreshold = 50;  // stall when SG_RESULT < 2 * SGTHRS; higher y = more sensitive
bool travelCalibrated = false;
int32_t travelMin = 0;
int32_t travelMax = 0;
bool sweepEnabled = false;
int32_t sweepLo = 0;
int32_t sweepHi = 0;
int32_t sweepTarget = 0;
String line;

void say(const char *msg) {
  Serial.println(msg);
  Serial.flush();
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  h/? help     t UART test     i status     x stop");
  Serial.println("  +/- nudge (+retract -extend)  n <steps>  s <Hz>  a <accel>  g <pos>");
  Serial.println("  c <mA>  m <usteps>");
  Serial.println("  w [amp]  back-and-forth on/off (z while running; x also stops)");
  Serial.println("  z SG read    v TSTEP read    f [steps] finger stall (two short extends)    y <0-255>    H home");
}

void printStatus() {
  Serial.print("pos=");
  Serial.print(stepper ? stepper->getCurrentPosition() : 0);
  Serial.print("  stepSize=");
  Serial.print(stepSize);
  Serial.print("  speedHz=");
  Serial.print(moveSpeedHz);
  Serial.print("  rms_mA=");
  Serial.print(rmsMa);
  Serial.print("  SGTHRS=");
  Serial.print(sgThreshold);
  Serial.print("  TCOOLTHRS=");
  Serial.print(TCOOLTHRS_SETTING);
  Serial.print("  sweep=");
  Serial.println(sweepEnabled ? "on" : "off");
  if (travelCalibrated) {
    Serial.print("  travel=0 (retract) .. ");
    Serial.print(travelMax);
    Serial.println(" (extend)");
  } else {
    Serial.println("  travel not calibrated (run H)");
  }
}

void uartTest() {
  const uint8_t conn = driver.test_connection();
  Serial.print("test_connection = ");
  Serial.print(conn);
  Serial.println(conn == 0 ? "  OK" : "  FAIL");
  const uint8_t ver = driver.version();
  Serial.print("TMC version = 0x");
  Serial.println(ver, HEX);
  if (ver == 0x21) {
    say("UART OK (TMC2209)");
  } else {
    say("no valid version");
  }
}

void setupStallGuard(uint8_t threshold) {
  sgThreshold = threshold;
  driver.TCOOLTHRS(TCOOLTHRS_SETTING);  // StallGuard valid once TSTEP <= this threshold
  driver.SGTHRS(sgThreshold);
}

uint16_t readStallGuard() {
  const uint16_t sg = driver.SG_RESULT() & 0x3FF;
  if (driver.CRCerror) {
    return 0xFFFF;
  }
  return sg;
}

uint32_t readTstep() {
  return driver.TSTEP() & 0xFFFFF;
}

bool stallGuardVelocityValid(uint32_t tstep) {
  // Smaller TSTEP = faster motor. Only trust StallGuard once the driver is
  // at or above the configured minimum velocity.
  return tstep > 0 && tstep <= TCOOLTHRS_SETTING;
}

// TMC2209: DIAG / stall when SG_RESULT < 2 * SGTHRS
bool isStalled(uint16_t sg) {
  if (sg == 0xFFFF) {
    return false;
  }
  return sg < (uint16_t)(2 * sgThreshold);
}

void waitStepperIdle(uint32_t timeoutMs = 2000) {
  if (!stepper) {
    return;
  }
  const uint32_t t0 = millis();
  while (stepper->isRunning()) {
    if (millis() - t0 > timeoutMs) {
      stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
      say("motion idle timeout — force stop");
      return;
    }
    delay(1);
  }
}

void applyHomeMotion() {
  if (!stepper) {
    return;
  }
  stepper->setSpeedInHz(HOME_SPEED_HZ);
  stepper->setAcceleration(HOME_ACCEL);
}

void stopMotion() {
  sweepEnabled = false;
  if (stepper) {
    stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
  }
}

void serviceSweep() {
  if (!sweepEnabled || !stepper || stepper->isRunning()) {
    return;
  }
  sweepTarget = (sweepTarget == sweepHi) ? sweepLo : sweepHi;
  stepper->moveTo(sweepTarget);
}

void startSweep(int32_t amplitude) {
  if (!stepper) {
    say("no stepper");
    return;
  }
  if (amplitude < 40) {
    amplitude = 40;
  }

  const int32_t pos = stepper->getCurrentPosition();
  if (travelCalibrated) {
    const int32_t inset = (travelMax > HOME_BACKOFF * 8) ? (travelMax / 10) : HOME_BACKOFF;
    sweepLo = travelMin + inset;
    sweepHi = travelMax - inset;
    if (sweepHi - sweepLo < 40) {
      say("travel too short to sweep — skip H ends, or don't run H and use w <amp>");
      return;
    }
  } else {
    sweepLo = pos - amplitude / 2;
    sweepHi = pos + amplitude / 2;
  }

  if (moveAccel < (int32_t)(moveSpeedHz * 2)) {
    Serial.print("accel ");
    Serial.print(moveAccel);
    Serial.print(" is low for speed ");
    Serial.print(moveSpeedHz);
    Serial.println(" — raise a so it reaches cruise (try a = 10x speed)");
  }

  sweepEnabled = true;
  sweepTarget = (pos < (sweepLo + sweepHi) / 2) ? sweepHi : sweepLo;
  stepper->moveTo(sweepTarget);
  Serial.print("sweep ON  ");
  Serial.print(sweepLo);
  Serial.print(" .. ");
  Serial.print(sweepHi);
  Serial.print("  speedHz=");
  Serial.print(moveSpeedHz);
  Serial.println("  (z to read SG, f = free vs finger stall, w or x to stop)");
}

static const uint8_t SG_TEST_N = 20;
static const uint16_t SG_TEST_PERIOD_MS = 50;
static const int32_t FINGER_TEST_STEPS = 1800;

void delayMs(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    delay(5);
  }
}

uint8_t collectSgWhileMoving(uint16_t *buf, uint8_t maxN) {
  uint8_t n = 0;
  const uint32_t giveUp = millis() + 2500;
  while (n < maxN && millis() < giveUp) {
    if (stepper && stepper->isRunning()) {
      buf[n++] = readStallGuard();
    } else if (n > 0) {
      break;
    }
    delayMs(SG_TEST_PERIOD_MS);
  }
  return n;
}

void sortU16(uint16_t *v, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    const uint16_t key = v[i];
    int j = (int)i - 1;
    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = key;
  }
}

bool summarizeSg(const char *label, const uint16_t *raw, uint8_t n,
                 uint16_t &outMed, uint16_t &outMin, uint16_t &outMax) {
  uint16_t tmp[SG_TEST_N];
  uint8_t m = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (raw[i] != 0xFFFF) {
      tmp[m++] = raw[i];
    }
  }
  if (m == 0) {
    Serial.print(label);
    Serial.println(": no valid SG samples (UART?)");
    outMed = outMin = outMax = 0;
    return false;
  }
  sortU16(tmp, m);
  outMin = tmp[0];
  outMax = tmp[m - 1];
  outMed = tmp[m / 2];
  uint32_t sum = 0;
  for (uint8_t i = 0; i < m; i++) {
    sum += tmp[i];
  }
  Serial.print(label);
  Serial.print(": n=");
  Serial.print(m);
  Serial.print("  min=");
  Serial.print(outMin);
  Serial.print("  med=");
  Serial.print(outMed);
  Serial.print("  avg=");
  Serial.print(sum / m);
  Serial.print("  max=");
  Serial.println(outMax);
  return true;
}

bool startExtendBurst(int32_t steps) {
  stopMotion();
  if (steps < 200) {
    steps = 200;
  }

  const int32_t pos = stepper->getCurrentPosition();
  int32_t dest = pos - steps;  // extend on this wiring
  if (travelCalibrated) {
    const int32_t limit = travelMin + HOME_BACKOFF;
    if (pos - limit < steps / 2) {
      say("Not enough room to extend. Jog toward IN, then f again.");
      return false;
    }
    if (dest < limit) {
      dest = limit;
    }
  }

  stepper->moveTo(dest);
  Serial.print("extending ");
  Serial.print(steps);
  Serial.println(" steps (short burst)");
  return true;
}

void fingerStallTest(int32_t steps) {
  if (!stepper) {
    say("no stepper");
    return;
  }

  say("");
  say("FINGER STALL TEST — two short one-way extends");
  say("Start from IN. Hands OFF for the first move.");
  Serial.flush();

  if (!startExtendBurst(steps)) {
    return;
  }
  delayMs(120);

  uint16_t freeBuf[SG_TEST_N];
  uint16_t stallBuf[SG_TEST_N];
  const uint8_t nFree = collectSgWhileMoving(freeBuf, SG_TEST_N);
  stopMotion();

  say("");
  say("Motor stopped. Get ready to HOLD the shaft.");
  say("Next extend in:");
  for (int i = 5; i >= 1; i--) {
    Serial.print("  ");
    Serial.println(i);
    Serial.flush();
    delayMs(1000);
  }
  say("HOLD NOW — extending into your finger...");
  Serial.flush();

  if (!startExtendBurst(steps)) {
    return;
  }
  delayMs(120);
  const uint8_t nStall = collectSgWhileMoving(stallBuf, SG_TEST_N);
  stopMotion();
  say("Done — you can let go.");
  say("");

  uint16_t freeMed = 0, freeMin = 0, freeMax = 0;
  uint16_t stallMed = 0, stallMin = 0, stallMax = 0;
  const bool okFree = summarizeSg("FREE ", freeBuf, nFree, freeMed, freeMin, freeMax);
  const bool okStall = summarizeSg("STALL", stallBuf, nStall, stallMed, stallMin, stallMax);
  if (!okFree || !okStall) {
    return;
  }

  Serial.print("drop (med) = ");
  Serial.println((int)freeMed - (int)stallMed);

  if (stallMed >= freeMed) {
    say("No drop — StallGuard did not see the load. Try higher s/a, more current, or a harder hold.");
    return;
  }
  if (freeMed < stallMed + 8) {
    say("Drop is small — usable but noisy. Try higher speed/current.");
  } else {
    say("Clear drop — StallGuard sees the finger stall.");
  }

  const uint16_t trip = (uint16_t)((stallMed + freeMed) / 2);
  uint16_t y = trip / 2;
  if (y < 1) {
    y = 1;
  }
  Serial.print("suggested y ");
  Serial.print(y);
  Serial.print("  (trip_below=");
  Serial.print(2 * y);
  Serial.println("). Set with y <n> if this looks right.");
}

int32_t clampToTravel(int32_t dest) {
  if (!travelCalibrated) {
    return dest;
  }
  if (dest < travelMin) {
    return travelMin;
  }
  if (dest > travelMax) {
    return travelMax;
  }
  return dest;
}

// dirSign: -1 retract (toward 0 after home), +1 extend (toward travelMax)
bool moveUntilStall(int dirSign, int32_t maxSteps, const char *label) {
  if (!stepper || maxSteps <= 0) {
    return false;
  }

  applyHomeMotion();

  const uint16_t trip = (uint16_t)(2 * sgThreshold);
  const int32_t startPos = stepper->getCurrentPosition();

  Serial.print("--- ");
  Serial.print(label);
  Serial.print("  dir=");
  Serial.print(dirSign > 0 ? "+" : "-");
  Serial.print("  SGTHRS=");
  Serial.print(sgThreshold);
  Serial.print("  trip_below=");
  Serial.print(trip);
  Serial.print("  need ");
  Serial.print(STALL_CONFIRM);
  Serial.print(" in a row  speed=");
  Serial.print(HOME_SPEED_HZ);
  Serial.print(" accel=");
  Serial.println(HOME_ACCEL);

  stepper->move((int32_t)dirSign * maxSteps);

  uint16_t n = 0;
  uint8_t stallHits = 0;
  uint16_t lastSg = 0xFFFF;
  bool stalled = false;
  while (stepper->isRunning()) {
    const uint32_t tstep = readTstep();
    const uint16_t sg = readStallGuard();
    lastSg = sg;
    n++;
    const int32_t pos = stepper->getCurrentPosition();

    const bool velocityValid = stallGuardVelocityValid(tstep);
    const bool hit = velocityValid && isStalled(sg);
    if (hit) {
      stallHits++;
    } else {
      // This also clears any false low-speed hits while accelerating.
      stallHits = 0;
    }

    Serial.print("  n=");
    Serial.print(n);
    Serial.print("  TSTEP=");
    Serial.print(tstep);
    Serial.print(velocityValid ? " SG_ON" : " SG_OFF");
    Serial.print("  sg=");
    if (sg == 0xFFFF) {
      Serial.print("UART?");
    } else {
      Serial.print(sg);
    }
    Serial.print("  trip=");
    Serial.print(trip);
    Serial.print("  hits=");
    Serial.print(stallHits);
    Serial.print("/");
    Serial.print(STALL_CONFIRM);
    Serial.print("  pos=");
    Serial.println(pos);

    if (stallHits >= STALL_CONFIRM) {
      stalled = true;
      stepper->forceStopAndNewPosition(pos);
      break;
    }
    delay(40);
  }

  const int32_t endPos = stepper->getCurrentPosition();
  Serial.print("  samples=");
  Serial.print(n);
  Serial.print("  last_sg=");
  Serial.print(lastSg);
  Serial.print("  moved=");
  Serial.print(endPos - startPos);
  Serial.print("  -> ");
  if (stalled) {
    Serial.println("STALL");
    applyHomeMotion();
    stepper->move((int32_t)(-dirSign) * HOME_BACKOFF);
    waitStepperIdle(1500);
    Serial.print("  backoff to pos=");
    Serial.println(stepper->getCurrentPosition());
  } else {
    Serial.println("NO STALL (hit max travel or stopped)");
  }

  return stalled;
}

void homeBothEnds() {
  if (!stepper) {
    say("no stepper");
    return;
  }
  stopMotion();

  say("StallGuard home (+retract / -extend)...");
  setupStallGuard(sgThreshold);
  Serial.print("SGTHRS=");
  Serial.print(sgThreshold);
  Serial.print("  home speed=");
  Serial.print(HOME_SPEED_HZ);
  Serial.print(" Hz  accel=");
  Serial.println(HOME_ACCEL);

  say("Seeking RETRACTED (+) ...");
  if (!moveUntilStall(-1, HOME_MAX_TRAVEL, "RETRACT")) {
    say("RETRACT: no stall — raise y (more sensitive) or check mechanics");
    travelCalibrated = false;
    stepper->setSpeedInHz(moveSpeedHz);
    stepper->setAcceleration(moveAccel);
    return;
  }
  stepper->setCurrentPosition(0);
  travelMin = 0;
  say("Retracted end = 0");
  delay(100);

  say("Seeking EXTENDED (-) ...");
  if (!moveUntilStall(+1, HOME_MAX_TRAVEL, "EXTEND")) {
    say("EXTEND: no stall — raise y or check mechanics");
    travelCalibrated = false;
    stepper->setSpeedInHz(moveSpeedHz);
    stepper->setAcceleration(moveAccel);
    return;
  }
  travelMax = stepper->getCurrentPosition();
  if (travelMax < HOME_BACKOFF * 2) {
    say("Travel too short — tune y / speed / current");
    travelCalibrated = false;
    stepper->setSpeedInHz(moveSpeedHz);
    stepper->setAcceleration(moveAccel);
    return;
  }
  travelCalibrated = true;
  Serial.print("Travel 0 (retract) .. ");
  Serial.print(travelMax);
  Serial.println(" (extend)");
  stepper->setSpeedInHz(moveSpeedHz);
  stepper->setAcceleration(moveAccel);
  say("Home done. +retracts -extends; g 0 / g <max>");
}

void setupStepper() {
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    say("FastAccelStepper failed to claim STEP pin");
    return;
  }
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(true);
  stepper->setSpeedInHz(moveSpeedHz);
  stepper->setAcceleration(moveAccel);
  stepper->setCurrentPosition(0);
  say("Stepper ready");
}

void setupDriver() {
  TMC_SERIAL.setPollingMode(true);
  if (!TMC_SERIAL.setTX(TX_PIN) || !TMC_SERIAL.setRX(RX_PIN)) {
    say("Serial2 setTX/setRX failed");
    return;
  }
  TMC_SERIAL.begin(115200);
  delay(50);
  say("Serial2.begin returned");

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.I_scale_analog(false);
  driver.internal_Rsense(false);
  driver.mstep_reg_select(true);
  driver.microsteps(motor_microsteps);
  driver.pwm_autoscale(true);
  driver.TPWMTHRS(0);
  driver.semin(0);
  driver.en_spreadCycle(false);
  driver.pdn_disable(true);
  driver.VACTUAL(0);
  driver.rms_current(rmsMa);
  setupStallGuard(sgThreshold);

  uartTest();
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
  } else if (c == 't' || c == 'T') {
    uartTest();
  } else if (c == 'i' || c == 'I' || c == 'p' || c == 'P') {
    printStatus();
  } else if (c == 'z' || c == 'Z') {
    const uint32_t tstep = readTstep();
    const uint16_t sg = readStallGuard();
    const uint16_t trip = (uint16_t)(2 * sgThreshold);
    const bool velocityValid = stallGuardVelocityValid(tstep);
    Serial.print("TSTEP=");
    Serial.print(tstep);
    Serial.print(velocityValid ? "  SG_ON" : "  SG_OFF");
    Serial.print("  SG_RESULT=");
    Serial.print(sg);
    Serial.print("  SGTHRS=");
    Serial.print(sgThreshold);
    Serial.print("  trip_below=");
    Serial.print(trip);
    Serial.print("  stalled=");
    Serial.println(velocityValid && isStalled(sg) ? "yes" : "no");
  } else if (c == 'v' || c == 'V') {
    const uint32_t tstep = readTstep();
    Serial.print("TSTEP=");
    Serial.print(tstep);
    Serial.print("  TCOOLTHRS=");
    Serial.print(TCOOLTHRS_SETTING);
    Serial.print("  StallGuard velocity gate=");
    Serial.println(stallGuardVelocityValid(tstep) ? "ON" : "OFF");
  } else if (c == 'f' || c == 'F') {
    int32_t steps = FINGER_TEST_STEPS;
    if (cmd.length() > 1) {
      const int32_t v = cmd.substring(1).toInt();
      if (v > 0) {
        steps = v;
      }
    }
    fingerStallTest(steps);
  } else if (c == 'y' || c == 'Y') {
    const int v = cmd.substring(1).toInt();
    if (v >= 0 && v <= 255) {
      setupStallGuard((uint8_t)v);
      Serial.print("SGTHRS=");
      Serial.print(sgThreshold);
      Serial.print("  trip_below=");
      Serial.println(2 * sgThreshold);
    }
  } else if (c == 'w' || c == 'W') {
    if (sweepEnabled) {
      stopMotion();
      say("sweep OFF");
    } else {
      int32_t amp = stepSize * 4;
      if (cmd.length() > 1) {
        const int32_t v = cmd.substring(1).toInt();
        if (v > 0) {
          amp = v;
        }
      }
      startSweep(amp);
    }
  } else if (c == 'g' || c == 'G') {
    if (!stepper) {
      return;
    }
    stopMotion();
    const int32_t dest = clampToTravel(cmd.substring(1).toInt());
    stepper->move/*
  TMC2209 Serial2 UART + FastAccelStepper + StallGuard end-finding.

  GP8/GP9 are UART1 → Serial2 (Serial1 is UART0 and will hang on these pins).

  Wiring:
    GP8 --[1k]--+---- TMC UART
    GP9 --------+
    STEP GP3, DIR GP2, EN -> GND
    VIO 3.3V, VM motor PSU, GND common

  Linear actuator: + retracts (toward 0), - extends (toward travelMax).
  USB 115200.
*/

#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

#define DIR_PIN    2
#define STEP_PIN   3
#define ENABLE_PIN -1
#define TX_PIN     8
#define RX_PIN     9

#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b00
#define TMC_SERIAL Serial2

static const uint16_t DEFAULT_RMS_MA = 300;
static const uint16_t DEFAULT_MICROSTEPS = 16;
static const uint32_t DEFAULT_SPEED_HZ = 200;
static const uint32_t DEFAULT_ACCEL = 400;
static const int32_t DEFAULT_STEP_SIZE = 200;

static const int32_t HOME_BACKOFF = 80;
static const int32_t HOME_MAX_TRAVEL = 35000;
static const uint32_t HOME_SPEED_HZ = 2500;
static const uint32_t HOME_ACCEL = 20000;
static const uint8_t STALL_CONFIRM = 5 ;
static const uint32_t TCOOLTHRS_SETTING = 400;  // tune after measuring TSTEP at steady homing speed

TMC2209Stepper driver(&TMC_SERIAL, R_SENSE, DRIVER_ADDRESS);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

int32_t stepSize = DEFAULT_STEP_SIZE;
uint16_t rmsMa = DEFAULT_RMS_MA;
uint16_t motor_microsteps = DEFAULT_MICROSTEPS;
uint32_t moveSpeedHz = DEFAULT_SPEED_HZ;
int32_t moveAccel = DEFAULT_ACCEL;
uint8_t sgThreshold = 50;  // stall when SG_RESULT < 2 * SGTHRS; higher y = more sensitive
bool travelCalibrated = false;
int32_t travelMin = 0;
int32_t travelMax = 0;
bool sweepEnabled = false;
int32_t sweepLo = 0;
int32_t sweepHi = 0;
int32_t sweepTarget = 0;
String line;

void say(const char *msg) {
  Serial.println(msg);
  Serial.flush();
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  h/? help     t UART test     i status     x stop");
  Serial.println("  +/- nudge (+retract -extend)  n <steps>  s <Hz>  a <accel>  g <pos>");
  Serial.println("  c <mA>  m <usteps>");
  Serial.println("  w [amp]  back-and-forth on/off (z while running; x also stops)");
  Serial.println("  z SG read    v TSTEP read    f [steps] finger stall (two short extends)    y <0-255>    H home");
}

void printStatus() {
  Serial.print("pos=");
  Serial.print(stepper ? stepper->getCurrentPosition() : 0);
  Serial.print("  stepSize=");
  Serial.print(stepSize);
  Serial.print("  speedHz=");
  Serial.print(moveSpeedHz);
  Serial.print("  rms_mA=");
  Serial.print(rmsMa);
  Serial.print("  SGTHRS=");
  Serial.print(sgThreshold);
  Serial.print("  TCOOLTHRS=");
  Serial.print(TCOOLTHRS_SETTING);
  Serial.print("  sweep=");
  Serial.println(sweepEnabled ? "on" : "off");
  if (travelCalibrated) {
    Serial.print("  travel=0 (retract) .. ");
    Serial.print(travelMax);
    Serial.println(" (extend)");
  } else {
    Serial.println("  travel not calibrated (run H)");
  }
}

void uartTest() {
  const uint8_t conn = driver.test_connection();
  Serial.print("test_connection = ");
  Serial.print(conn);
  Serial.println(conn == 0 ? "  OK" : "  FAIL");
  const uint8_t ver = driver.version();
  Serial.print("TMC version = 0x");
  Serial.println(ver, HEX);
  if (ver == 0x21) {
    say("UART OK (TMC2209)");
  } else {
    say("no valid version");
  }
}

void setupStallGuard(uint8_t threshold) {
  sgThreshold = threshold;
  driver.TCOOLTHRS(TCOOLTHRS_SETTING);  // StallGuard valid once TSTEP <= this threshold
  driver.SGTHRS(sgThreshold);
}

uint16_t readStallGuard() {
  const uint16_t sg = driver.SG_RESULT() & 0x3FF;
  if (driver.CRCerror) {
    return 0xFFFF;
  }
  return sg;
}

uint32_t readTstep() {
  return driver.TSTEP() & 0xFFFFF;
}

bool stallGuardVelocityValid(uint32_t tstep) {
  // Smaller TSTEP = faster motor. Only trust StallGuard once the driver is
  // at or above the configured minimum velocity.
  return tstep > 0 && tstep <= TCOOLTHRS_SETTING;
}

// TMC2209: DIAG / stall when SG_RESULT < 2 * SGTHRS
bool isStalled(uint16_t sg) {
  if (sg == 0xFFFF) {
    return false;
  }
  return sg < (uint16_t)(2 * sgThreshold);
}

void waitStepperIdle(uint32_t timeoutMs = 2000) {
  if (!stepper) {
    return;
  }
  const uint32_t t0 = millis();
  while (stepper->isRunning()) {
    if (millis() - t0 > timeoutMs) {
      stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
      say("motion idle timeout — force stop");
      return;
    }
    delay(1);
  }
}

void applyHomeMotion() {
  if (!stepper) {
    return;
  }
  stepper->setSpeedInHz(HOME_SPEED_HZ);
  stepper->setAcceleration(HOME_ACCEL);
}

void stopMotion() {
  sweepEnabled = false;
  if (stepper) {
    stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
  }
}

void serviceSweep() {
  if (!sweepEnabled || !stepper || stepper->isRunning()) {
    return;
  }
  sweepTarget = (sweepTarget == sweepHi) ? sweepLo : sweepHi;
  stepper->moveTo(sweepTarget);
}

void startSweep(int32_t amplitude) {
  if (!stepper) {
    say("no stepper");
    return;
  }
  if (amplitude < 40) {
    amplitude = 40;
  }

  const int32_t pos = stepper->getCurrentPosition();
  if (travelCalibrated) {
    const int32_t inset = (travelMax > HOME_BACKOFF * 8) ? (travelMax / 10) : HOME_BACKOFF;
    sweepLo = travelMin + inset;
    sweepHi = travelMax - inset;
    if (sweepHi - sweepLo < 40) {
      say("travel too short to sweep — skip H ends, or don't run H and use w <amp>");
      return;
    }
  } else {
    sweepLo = pos - amplitude / 2;
    sweepHi = pos + amplitude / 2;
  }

  if (moveAccel < (int32_t)(moveSpeedHz * 2)) {
    Serial.print("accel ");
    Serial.print(moveAccel);
    Serial.print(" is low for speed ");
    Serial.print(moveSpeedHz);
    Serial.println(" — raise a so it reaches cruise (try a = 10x speed)");
  }

  sweepEnabled = true;
  sweepTarget = (pos < (sweepLo + sweepHi) / 2) ? sweepHi : sweepLo;
  stepper->moveTo(sweepTarget);
  Serial.print("sweep ON  ");
  Serial.print(sweepLo);
  Serial.print(" .. ");
  Serial.print(sweepHi);
  Serial.print("  speedHz=");
  Serial.print(moveSpeedHz);
  Serial.println("  (z to read SG, f = free vs finger stall, w or x to stop)");
}

static const uint8_t SG_TEST_N = 20;
static const uint16_t SG_TEST_PERIOD_MS = 50;
static const int32_t FINGER_TEST_STEPS = 1800;

void delayMs(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    delay(5);
  }
}

uint8_t collectSgWhileMoving(uint16_t *buf, uint8_t maxN) {
  uint8_t n = 0;
  const uint32_t giveUp = millis() + 2500;
  while (n < maxN && millis() < giveUp) {
    if (stepper && stepper->isRunning()) {
      buf[n++] = readStallGuard();
    } else if (n > 0) {
      break;
    }
    delayMs(SG_TEST_PERIOD_MS);
  }
  return n;
}

void sortU16(uint16_t *v, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    const uint16_t key = v[i];
    int j = (int)i - 1;
    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = key;
  }
}

bool summarizeSg(const char *label, const uint16_t *raw, uint8_t n,
                 uint16_t &outMed, uint16_t &outMin, uint16_t &outMax) {
  uint16_t tmp[SG_TEST_N];
  uint8_t m = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (raw[i] != 0xFFFF) {
      tmp[m++] = raw[i];
    }
  }
  if (m == 0) {
    Serial.print(label);
    Serial.println(": no valid SG samples (UART?)");
    outMed = outMin = outMax = 0;
    return false;
  }
  sortU16(tmp, m);
  outMin = tmp[0];
  outMax = tmp[m - 1];
  outMed = tmp[m / 2];
  uint32_t sum = 0;
  for (uint8_t i = 0; i < m; i++) {
    sum += tmp[i];
  }
  Serial.print(label);
  Serial.print(": n=");
  Serial.print(m);
  Serial.print("  min=");
  Serial.print(outMin);
  Serial.print("  med=");
  Serial.print(outMed);
  Serial.print("  avg=");
  Serial.print(sum / m);
  Serial.print("  max=");
  Serial.println(outMax);
  return true;
}

bool startExtendBurst(int32_t steps) {
  stopMotion();
  if (steps < 200) {
    steps = 200;
  }

  const int32_t pos = stepper->getCurrentPosition();
  int32_t dest = pos - steps;  // extend on this wiring
  if (travelCalibrated) {
    const int32_t limit = travelMin + HOME_BACKOFF;
    if (pos - limit < steps / 2) {
      say("Not enough room to extend. Jog toward IN, then f again.");
      return false;
    }
    if (dest < limit) {
      dest = limit;
    }
  }

  stepper->moveTo(dest);
  Serial.print("extending ");
  Serial.print(steps);
  Serial.println(" steps (short burst)");
  return true;
}

void fingerStallTest(int32_t steps) {
  if (!stepper) {
    say("no stepper");
    return;
  }

  say("");
  say("FINGER STALL TEST — two short one-way extends");
  say("Start from IN. Hands OFF for the first move.");
  Serial.flush();

  if (!startExtendBurst(steps)) {
    return;
  }
  delayMs(120);

  uint16_t freeBuf[SG_TEST_N];
  uint16_t stallBuf[SG_TEST_N];
  const uint8_t nFree = collectSgWhileMoving(freeBuf, SG_TEST_N);
  stopMotion();

  say("");
  say("Motor stopped. Get ready to HOLD the shaft.");
  say("Next extend in:");
  for (int i = 5; i >= 1; i--) {
    Serial.print("  ");
    Serial.println(i);
    Serial.flush();
    delayMs(1000);
  }
  say("HOLD NOW — extending into your finger...");
  Serial.flush();

  if (!startExtendBurst(steps)) {
    return;
  }
  delayMs(120);
  const uint8_t nStall = collectSgWhileMoving(stallBuf, SG_TEST_N);
  stopMotion();
  say("Done — you can let go.");
  say("");

  uint16_t freeMed = 0, freeMin = 0, freeMax = 0;
  uint16_t stallMed = 0, stallMin = 0, stallMax = 0;
  const bool okFree = summarizeSg("FREE ", freeBuf, nFree, freeMed, freeMin, freeMax);
  const bool okStall = summarizeSg("STALL", stallBuf, nStall, stallMed, stallMin, stallMax);
  if (!okFree || !okStall) {
    return;
  }

  Serial.print("drop (med) = ");
  Serial.println((int)freeMed - (int)stallMed);

  if (stallMed >= freeMed) {
    say("No drop — StallGuard did not see the load. Try higher s/a, more current, or a harder hold.");
    return;
  }
  if (freeMed < stallMed + 8) {
    say("Drop is small — usable but noisy. Try higher speed/current.");
  } else {
    say("Clear drop — StallGuard sees the finger stall.");
  }

  const uint16_t trip = (uint16_t)((stallMed + freeMed) / 2);
  uint16_t y = trip / 2;
  if (y < 1) {
    y = 1;
  }
  Serial.print("suggested y ");
  Serial.print(y);
  Serial.print("  (trip_below=");
  Serial.print(2 * y);
  Serial.println("). Set with y <n> if this looks right.");
}

int32_t clampToTravel(int32_t dest) {
  if (!travelCalibrated) {
    return dest;
  }
  if (dest < travelMin) {
    return travelMin;
  }
  if (dest > travelMax) {
    return travelMax;
  }
  return dest;
}

// dirSign: -1 retract (toward 0 after home), +1 extend (toward travelMax)
bool moveUntilStall(int dirSign, int32_t maxSteps, const char *label) {
  if (!stepper || maxSteps <= 0) {
    return false;
  }

  applyHomeMotion();

  const uint16_t trip = (uint16_t)(2 * sgThreshold);
  const int32_t startPos = stepper->getCurrentPosition();

  Serial.print("--- ");
  Serial.print(label);
  Serial.print("  dir=");
  Serial.print(dirSign > 0 ? "+" : "-");
  Serial.print("  SGTHRS=");
  Serial.print(sgThreshold);
  Serial.print("  trip_below=");
  Serial.print(trip);
  Serial.print("  need ");
  Serial.print(STALL_CONFIRM);
  Serial.print(" in a row  speed=");
  Serial.print(HOME_SPEED_HZ);
  Serial.print(" accel=");
  Serial.println(HOME_ACCEL);

  stepper->move((int32_t)dirSign * maxSteps);

  uint16_t n = 0;
  uint8_t stallHits = 0;
  uint16_t lastSg = 0xFFFF;
  bool stalled = false;
  while (stepper->isRunning()) {
    const uint32_t tstep = readTstep();
    const uint16_t sg = readStallGuard();
    lastSg = sg;
    n++;
    const int32_t pos = stepper->getCurrentPosition();

    const bool velocityValid = stallGuardVelocityValid(tstep);
    const bool hit = velocityValid && isStalled(sg);
    if (hit) {
      stallHits++;
    } else {
      // This also clears any false low-speed hits while accelerating.
      stallHits = 0;
    }

    Serial.print("  n=");
    Serial.print(n);
    Serial.print("  TSTEP=");
    Serial.print(tstep);
    Serial.print(velocityValid ? " SG_ON" : " SG_OFF");
    Serial.print("  sg=");
    if (sg == 0xFFFF) {
      Serial.print("UART?");
    } else {
      Serial.print(sg);
    }
    Serial.print("  trip=");
    Serial.print(trip);
    Serial.print("  hits=");
    Serial.print(stallHits);
    Serial.print("/");
    Serial.print(STALL_CONFIRM);
    Serial.print("  pos=");
    Serial.println(pos);

    if (stallHits >= STALL_CONFIRM) {
      stalled = true;
      stepper->forceStopAndNewPosition(pos);
      break;
    }
    delay(40);
  }

  const int32_t endPos = stepper->getCurrentPosition();
  Serial.print("  samples=");
  Serial.print(n);
  Serial.print("  last_sg=");
  Serial.print(lastSg);
  Serial.print("  moved=");
  Serial.print(endPos - startPos);
  Serial.print("  -> ");
  if (stalled) {
    Serial.println("STALL");
    applyHomeMotion();
    stepper->move((int32_t)(-dirSign) * HOME_BACKOFF);
    waitStepperIdle(1500);
    Serial.print("  backoff to pos=");
    Serial.println(stepper->getCurrentPosition());
  } else {
    Serial.println("NO STALL (hit max travel or stopped)");
  }

  return stalled;
}

void homeBothEnds() {
  if (!stepper) {
    say("no stepper");
    return;
  }
  stopMotion();

  say("StallGuard home (+retract / -extend)...");
  setupStallGuard(sgThreshold);
  Serial.print("SGTHRS=");
  Serial.print(sgThreshold);
  Serial.print("  home speed=");
  Serial.print(HOME_SPEED_HZ);
  Serial.print(" Hz  accel=");
  Serial.println(HOME_ACCEL);

  say("Seeking RETRACTED (+) ...");
  if (!moveUntilStall(-1, HOME_MAX_TRAVEL, "RETRACT")) {
    say("RETRACT: no stall — raise y (more sensitive) or check mechanics");
    travelCalibrated = false;
    stepper->setSpeedInHz(moveSpeedHz);
    stepper->setAcceleration(moveAccel);
    return;
  }
  stepper->setCurrentPosition(0);
  travelMin = 0;
  say("Retracted end = 0");
  delay(100);

  say("Seeking EXTENDED (-) ...");
  if (!moveUntilStall(+1, HOME_MAX_TRAVEL, "EXTEND")) {
    say("EXTEND: no stall — raise y or check mechanics");
    travelCalibrated = false;
    stepper->setSpeedInHz(moveSpeedHz);
    stepper->setAcceleration(moveAccel);
    return;
  }
  travelMax = stepper->getCurrentPosition();
  if (travelMax < HOME_BACKOFF * 2) {
    say("Travel too short — tune y / speed / current");
    travelCalibrated = false;
    stepper->setSpeedInHz(moveSpeedHz);
    stepper->setAcceleration(moveAccel);
    return;
  }
  travelCalibrated = true;
  Serial.print("Travel 0 (retract) .. ");
  Serial.print(travelMax);
  Serial.println(" (extend)");
  stepper->setSpeedInHz(moveSpeedHz);
  stepper->setAcceleration(moveAccel);
  say("Home done. +retracts -extends; g 0 / g <max>");
}

void setupStepper() {
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    say("FastAccelStepper failed to claim STEP pin");
    return;
  }
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(true);
  stepper->setSpeedInHz(moveSpeedHz);
  stepper->setAcceleration(moveAccel);
  stepper->setCurrentPosition(0);
  say("Stepper ready");
}

void setupDriver() {
  TMC_SERIAL.setPollingMode(true);
  if (!TMC_SERIAL.setTX(TX_PIN) || !TMC_SERIAL.setRX(RX_PIN)) {
    say("Serial2 setTX/setRX failed");
    return;
  }
  TMC_SERIAL.begin(115200);
  delay(50);
  say("Serial2.begin returned");

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.I_scale_analog(false);
  driver.internal_Rsense(false);
  driver.mstep_reg_select(true);
  driver.microsteps(motor_microsteps);
  driver.pwm_autoscale(true);
  driver.TPWMTHRS(0);
  driver.semin(0);
  driver.en_spreadCycle(false);
  driver.pdn_disable(true);
  driver.VACTUAL(0);
  driver.rms_current(rmsMa);
  setupStallGuard(sgThreshold);

  uartTest();
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
  } else if (c == 't' || c == 'T') {
    uartTest();
  } else if (c == 'i' || c == 'I' || c == 'p' || c == 'P') {
    printStatus();
  } else if (c == 'z' || c == 'Z') {
    const uint32_t tstep = readTstep();
    const uint16_t sg = readStallGuard();
    const uint16_t trip = (uint16_t)(2 * sgThreshold);
    const bool velocityValid = stallGuardVelocityValid(tstep);
    Serial.print("TSTEP=");
    Serial.print(tstep);
    Serial.print(velocityValid ? "  SG_ON" : "  SG_OFF");
    Serial.print("  SG_RESULT=");
    Serial.print(sg);
    Serial.print("  SGTHRS=");
    Serial.print(sgThreshold);
    Serial.print("  trip_below=");
    Serial.print(trip);
    Serial.print("  stalled=");
    Serial.println(velocityValid && isStalled(sg) ? "yes" : "no");
  } else if (c == 'v' || c == 'V') {
    const uint32_t tstep = readTstep();
    Serial.print("TSTEP=");
    Serial.print(tstep);
    Serial.print("  TCOOLTHRS=");
    Serial.print(TCOOLTHRS_SETTING);
    Serial.print("  StallGuard velocity gate=");
    Serial.println(stallGuardVelocityValid(tstep) ? "ON" : "OFF");
  } else if (c == 'f' || c == 'F') {
    int32_t steps = FINGER_TEST_STEPS;
    if (cmd.length() > 1) {
      const int32_t v = cmd.substring(1).toInt();
      if (v > 0) {
        steps = v;
      }
    }
    fingerStallTest(steps);
  } else if (c == 'y' || c == 'Y') {
    const int v = cmd.substring(1).toInt();
    if (v >= 0 && v <= 255) {
      setupStallGuard((uint8_t)v);
      Serial.print("SGTHRS=");
      Serial.print(sgThreshold);
      Serial.print("  trip_below=");
      Serial.println(2 * sgThreshold);
    }
  } else if (c == 'w' || c == 'W') {
    if (sweepEnabled) {
      stopMotion();
      say("sweep OFF");
    } else {
      int32_t amp = stepSize * 4;
      if (cmd.length() > 1) {
        const int32_t v = cmd.substring(1).toInt();
        if (v > 0) {
          amp = v;
        }
      }
      startSweep(amp);
    }
  } else if (c == 'g' || c == 'G') {
    if (!stepper) {
      return;
    }
    stopMotion();
    const int32_t dest = clampToTravel(cmd.substring(1).toInt());
    stepper->moveTo(dest);
    Serial.print("goto ");
    Serial.println(dest);
  } else if (c == '+' || c == '-') {
    int32_t delta = (c == '+') ? -stepSize : stepSize;
    if (cmd.length() > 1) {
      int32_t mag = cmd.substring(1).toInt();
      if (mag < 0) {
        mag = -mag;
      }
      delta = (c == '+') ? -mag : mag;
    }
    stopMotion();
    if (stepper) {
      const int32_t dest = clampToTravel(stepper->getCurrentPosition() + delta);
      delta = dest - stepper->getCurrentPosition();
      if (delta != 0) {
        stepper->move(delta);
      }
    }
    Serial.print("move ");
    Serial.println(delta);
  } else if (c == 'n' || c == 'N') {
    const int32_t v = cmd.substring(1).toInt();
    if (v > 0) {
      stepSize = v;
      Serial.print("stepSize=");
      Serial.println(stepSize);
    }
  } else if (c == 's' || c == 'S') {
    const uint32_t v = (uint32_t)cmd.substring(1).toInt();
    if (v > 0 && stepper) {
      moveSpeedHz = v;
      stepper->setSpeedInHz(v);
      Serial.print("speedHz=");
      Serial.println(moveSpeedHz);
    }
  } else if (c == 'a' || c == 'A') {
    const int32_t v = cmd.substring(1).toInt();
    if (v > 0 && stepper) {
      moveAccel = v;
      stepper->setAcceleration(v);
      Serial.print("accel=");
      Serial.println(moveAccel);
    }
  } else if (c == 'c' || c == 'C') {
    const int v = cmd.substring(1).toInt();
    if (v > 0 && v < 2000) {
      rmsMa = (uint16_t)v;
      driver.rms_current(rmsMa);
      Serial.print("rms_mA=");
      Serial.println(rmsMa);
    }
  } else if (c == 'm' || c == 'M') {
    const int v = cmd.substring(1).toInt();
    if (v == 8 || v == 16 || v == 32 || v == 64 || v == 128 || v == 256) {
      motor_microsteps = (uint16_t)v;
      driver.microsteps(motor_microsteps);
      Serial.print("microsteps=");
      Serial.println(motor_microsteps);
    }
  } else if (c == 'x' || c == 'X') {
    stopMotion();
    say("stop");
  } else {
    say("unknown (h)");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  say("TMC2209 Serial2 UART + StallGuard (no auto-cycle)");

  setupStepper();
  setupDriver();
  printHelp();
  say("Setup complete. Tune z/y, then H.");
}

void loop() {
  serviceSweep();

  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (line.length() > 0) {
        processCommand(line);
        line = "";
      }
    } else {
      line += ch;
    }
  }
}To(dest);
    Serial.print("goto ");
    Serial.println(dest);
  } else if (c == '+' || c == '-') {
    int32_t delta = (c == '+') ? -stepSize : stepSize;
    if (cmd.length() > 1) {
      int32_t mag = cmd.substring(1).toInt();
      if (mag < 0) {
        mag = -mag;
      }
      delta = (c == '+') ? -mag : mag;
    }
    stopMotion();
    if (stepper) {
      const int32_t dest = clampToTravel(stepper->getCurrentPosition() + delta);
      delta = dest - stepper->getCurrentPosition();
      if (delta != 0) {
        stepper->move(delta);
      }
    }
    Serial.print("move ");
    Serial.println(delta);
  } else if (c == 'n' || c == 'N') {
    const int32_t v = cmd.substring(1).toInt();
    if (v > 0) {
      stepSize = v;
      Serial.print("stepSize=");
      Serial.println(stepSize);
    }
  } else if (c == 's' || c == 'S') {
    const uint32_t v = (uint32_t)cmd.substring(1).toInt();
    if (v > 0 && stepper) {
      moveSpeedHz = v;
      stepper->setSpeedInHz(v);
      Serial.print("speedHz=");
      Serial.println(moveSpeedHz);
    }
  } else if (c == 'a' || c == 'A') {
    const int32_t v = cmd.substring(1).toInt();
    if (v > 0 && stepper) {
      moveAccel = v;
      stepper->setAcceleration(v);
      Serial.print("accel=");
      Serial.println(moveAccel);
    }
  } else if (c == 'c' || c == 'C') {
    const int v = cmd.substring(1).toInt();
    if (v > 0 && v < 2000) {
      rmsMa = (uint16_t)v;
      driver.rms_current(rmsMa);
      Serial.print("rms_mA=");
      Serial.println(rmsMa);
    }
  } else if (c == 'm' || c == 'M') {
    const int v = cmd.substring(1).toInt();
    if (v == 8 || v == 16 || v == 32 || v == 64 || v == 128 || v == 256) {
      motor_microsteps = (uint16_t)v;
      driver.microsteps(motor_microsteps);
      Serial.print("microsteps=");
      Serial.println(motor_microsteps);
    }
  } else if (c == 'x' || c == 'X') {
    stopMotion();
    say("stop");
  } else {
    say("unknown (h)");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  say("TMC2209 Serial2 UART + StallGuard (no auto-cycle)");

  setupStepper();
  setupDriver();
  printHelp();
  say("Setup complete. Tune z/y, then H.");
}

void loop() {
  serviceSweep();

  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (line.length() > 0) {
        processCommand(line);
        line = "";
      }
    } else {
      line += ch;
    }
  }
}
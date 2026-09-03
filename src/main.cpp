/*
  TMC2209 Serial2 UART + FastAccelStepper + StallGuard end-finding.

  GP8/GP9 are UART1 → Serial2 (Serial1 is UART0 and will hang on these pins).

  Wiring:
    GP8 --[1k]--+---- TMC UART
    GP9 --------+
    STEP GP3, DIR GP2, EN -> GND
    DIAG GP6 (stall pulse; do not use INDEX)
    VIO 3.3V, VM motor PSU, GND common

  Linear actuator: + extends (out, toward travelMax), - retracts (in, toward 0).
  USB 115200.
*/

#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

#define DIR_PIN    2
#define STEP_PIN   3
#define ENABLE_PIN -1
#define DIAG_PIN   6
#define TX_PIN     8
#define RX_PIN     9

#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b00
#define TMC_SERIAL Serial2

static const uint16_t DEFAULT_RMS_MA = 400;
static const float IHOLD_FRACTION = 1.0f;  // hold = run (400 mA both)
static const uint16_t DEFAULT_MICROSTEPS = 16;
static const uint32_t DEFAULT_SPEED_HZ = 2500;
static const uint32_t DEFAULT_ACCEL = 40000;
static const int32_t DEFAULT_STEP_SIZE = 2000;

static const int32_t HOME_BACKOFF = 80;
static const int32_t HOME_MAX_TRAVEL = 35000;
static const uint32_t HOME_SPEED_HZ = 2500;
static const uint32_t HOME_ACCEL = 20000;
static const uint8_t STALL_CONFIRM = 3;         // UART fallback only; DIAG trips on first pulse
static const uint32_t TCOOLTHRS_SETTING = 400;  // chip pulses DIAG once TSTEP <= this
static const uint32_t HOME_SG_PLOT_MS = 20;     // UART SG is for Teleplot only, not the trip
// Accel to 2500 Hz at 20000 is ~156 steps; SG is still low for a bit after that.
static const int32_t HOME_STALL_ARM_STEPS = 400;
static const uint8_t SG_CRUISE_WIN = 16;
static const uint16_t SG_READY_MIN = 24;  // mean must be up; oscillation around it is fine
static const uint8_t SG_MED_SETTLE = 5;   // consecutive similar medians (~100 ms)


// for speed sweep routine
static const int32_t SG_SWEEP_STEPS = 8000;
static const uint32_t SG_SWEEP_ACCEL = 20000;
static const uint32_t SG_SWEEP_HZ_LO = 1400;
static const uint32_t SG_SWEEP_HZ_HI = 3000;
static const uint32_t SG_SWEEP_HZ_STEP = 200;
static const uint16_t SG_SWEEP_PERIOD_MS = 20;
static const uint8_t SG_SWEEP_MAX_N = 200;
static const uint8_t SG_SWEEP_MAX_SPEEDS = 16;

// After you read AUTO from `k`, paste here so the same PWM survives uploads. 0 = boot AUTO.
static const uint8_t STEALTH_MANUAL_OFS = 80;
static const uint8_t STEALTH_MANUAL_GRAD = 9;
static const uint8_t PWM_OFS_MAX = 80;  // clamp; too high with autoscale off can over-current

TMC2209Stepper driver(&TMC_SERIAL, R_SENSE, DRIVER_ADDRESS);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

int32_t stepSize = DEFAULT_STEP_SIZE;
uint16_t rmsMa = DEFAULT_RMS_MA;
uint16_t motor_microsteps = DEFAULT_MICROSTEPS;
uint32_t moveSpeedHz = DEFAULT_SPEED_HZ;
int32_t moveAccel = DEFAULT_ACCEL;
uint8_t sgThreshold = 20;  // stall when SG_RESULT < 2 * SGTHRS; higher y = more sensitive
bool travelCalibrated = false;
int32_t travelMin = 0;
int32_t travelMax = 0;
bool originSet = false;
bool sweepEnabled = false;
int32_t sweepLo = 0;
int32_t sweepHi = 0;
int32_t sweepTarget = 0;
String line;
bool stealthFrozen = false;
uint8_t frozenOfs = 0;
uint8_t frozenGrad = 0;

// TMC2209 StallGuard4 compares SG_RESULT to 2*SGTHRS on-chip and pulses DIAG.
// INDEX is a different output (electrical position) and cannot carry stall.
volatile bool diagStallPulse = false;

void diagIsr() {
  diagStallPulse = true;
}

void armDiag() {
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN), diagIsr, RISING);
  diagStallPulse = false;  // drop any edge caused by attach
}

void disarmDiag() {
  detachInterrupt(digitalPinToInterrupt(DIAG_PIN));
}

bool diagPinHigh() {
  return digitalRead(DIAG_PIN) == HIGH;
}

void say(const char *msg) {
  Serial.println(msg);
  Serial.flush();
}

void plotPos(int32_t pos) {
  Serial.print(">pos:");
  Serial.println(pos);
}

// Teleplot serial format is one ">name:value" line per variable.
void plotHomeSample(uint16_t sg, uint16_t trip, uint8_t diag, int32_t pos) {
  if (sg != 0xFFFF) {
    Serial.print(">sg:");
    Serial.println(sg);
  }
  Serial.print(">diag:");
  Serial.println(diag);
  Serial.print(">trip:");
  Serial.println(trip);
  if (originSet) {
    plotPos(pos);
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  h/? help     t UART test     i status     x stop     0 set here as pos 0");
  Serial.println("  +/- nudge (+extend -retract)  n <steps>  s <Hz>  a <accel>  g <pos>");
  Serial.println("  c <mA>  m <usteps>");
  Serial.println("  w [amp]  back-and-forth on/off (z while running; x also stops)");
  Serial.println("  z SG/DIAG    v TSTEP     f [steps] finger stall    y <0-255>");
  Serial.println("  H home to 0  (DIAG GP6). Teleplot: >sg:  >diag:  >trip:  >pos:");
  Serial.println("  e [steps]  SG vs speed (AUTO, 1400-3000 Hz). Teleplot >sg_avg >sg_amp");
  Serial.println("  k          print AUTO pwm_ofs / pwm_grad");
  Serial.println("  k copy     lock those AUTO values (manual)");
  Serial.println("  k <ofs> <grad>  lock those numbers (same each upload)");
  Serial.println("  K          StealthChop AUTO again");
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
  Serial.print("  DIAG=");
  Serial.print(diagPinHigh() ? "HIGH" : "LOW");
  Serial.print("  stealth=");
  if (stealthFrozen) {
    Serial.print("MANUAL ofs=");
    Serial.print(frozenOfs);
    Serial.print(" grad=");
    Serial.print(frozenGrad);
  } else {
    Serial.print("AUTO ofs=");
    Serial.print(driver.pwm_ofs_auto());
    Serial.print(" grad=");
    Serial.print(driver.pwm_grad_auto());
  }
  Serial.print("  sweep=");
  Serial.println(sweepEnabled ? "on" : "off");
  if (travelCalibrated) {
    Serial.print("  travel=0 (in) .. ");
    Serial.print(travelMax);
    Serial.println(" (out)");
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

void unfreezeStealthChop();
void stopMotion();

bool applyManualStealth(uint8_t ofs, uint8_t grad) {
  if (ofs > PWM_OFS_MAX) {
    Serial.print("PWM_OFS ");
    Serial.print(ofs);
    Serial.print(" clamped to ");
    Serial.println(PWM_OFS_MAX);
    ofs = PWM_OFS_MAX;
  }
  if (ofs < 1) {
    say("manual skip: ofs must be >= 1");
    return false;
  }
  driver.pwm_ofs(ofs);
  driver.pwm_grad(grad);
  driver.pwm_autoscale(false);
  driver.pwm_autograd(false);
  stealthFrozen = true;
  frozenOfs = ofs;
  frozenGrad = grad;
  Serial.print("StealthChop MANUAL  ofs=");
  Serial.print(ofs);
  Serial.print("  grad=");
  Serial.println(grad);
  say("IRUN does not limit PWM; OT/short watchdog is on. K = AUTO");
  return true;
}

void printAutoPwm() {
  Serial.print("AUTO pwm_ofs=");
  Serial.print(driver.pwm_ofs_auto());
  Serial.print("  pwm_grad=");
  Serial.println(driver.pwm_grad_auto());
  Serial.println("  k copy            lock these");
  Serial.println("  k <ofs> <grad>    lock numbers (survives if you paste into STEALTH_MANUAL_OFS/GRAD)");
}

bool freezeStealthChop() {
  const uint8_t ofs = driver.pwm_ofs_auto();
  const uint8_t grad = driver.pwm_grad_auto();
  if (ofs < 8) {
    say("copy skip: pwm_ofs_auto still low — move at speed first");
    printAutoPwm();
    return false;
  }
  return applyManualStealth(ofs, grad);
}

void unfreezeStealthChop() {
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);
  stealthFrozen = false;
  say("StealthChop AUTO");
}

void serviceManualCurrentGuard() {
  if (!stealthFrozen) {
    return;
  }
  static uint32_t lastMs = 0;
  const uint32_t now = millis();
  if (now - lastMs < 200) {
    return;
  }
  lastMs = now;

  const bool hot = driver.otpw() || driver.ot();
  const bool shorted =
      driver.s2ga() || driver.s2gb() || driver.s2vsa() || driver.s2vsb();
  if (hot || shorted) {
    unfreezeStealthChop();
    stopMotion();
    say(hot ? "OT — MANUAL off, AUTO + stop" : "short — MANUAL off, AUTO + stop");
    return;
  }

  const uint16_t cs = driver.cs_actual();
  if (cs >= 31) {
    if (frozenOfs > 12) {
      applyManualStealth((uint8_t)(frozenOfs - 4), frozenGrad);
      say("cs_actual pegged — reduced PWM_OFS");
    } else {
      unfreezeStealthChop();
      stopMotion();
      say("cs_actual pegged — AUTO + stop");
    }
  }
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

static bool pollAbortX() {
  bool abort = false;
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == 'x' || ch == 'X') {
      abort = true;
    }
  }
  return abort;
}

static uint32_t accelDistanceSteps(uint32_t speedHz, uint32_t accel) {
  if (accel == 0) {
    return 0;
  }
  return (speedHz * speedHz) / (2 * accel);
}

// Sample SG only on the constant-speed middle of a +move. Returns n.
static uint8_t collectCruiseSg(int32_t moveSteps, uint32_t speedHz, uint32_t accel,
                               uint16_t *buf, uint8_t maxN, bool &aborted) {
  aborted = false;
  uint8_t n = 0;
  if (!stepper || moveSteps <= 0) {
    return 0;
  }

  const int32_t start = stepper->getCurrentPosition();
  const uint32_t ramp = accelDistanceSteps(speedHz, accel);
  uint32_t startSkip = ramp + ramp / 4 + 80;
  if (startSkip < (uint32_t)HOME_STALL_ARM_STEPS) {
    startSkip = (uint32_t)HOME_STALL_ARM_STEPS;
  }
  const uint32_t endSkip = ramp + ramp / 4 + 80;
  const int32_t cruise0 = start + (int32_t)startSkip;
  const int32_t cruise1 = start + moveSteps - (int32_t)endSkip;
  if (cruise1 <= cruise0) {
    say("move too short for cruise SG at this speed");
    return 0;
  }

  stepper->setSpeedInHz(speedHz);
  stepper->setAcceleration((int32_t)accel);
  stepper->move(moveSteps);

  uint32_t lastMs = 0;
  uint8_t settle = 0;
  const uint32_t giveUp = millis() + 20000;
  while (stepper->isRunning()) {
    if (pollAbortX() || millis() > giveUp) {
      aborted = true;
      stopMotion();
      break;
    }
    const uint32_t now = millis();
    if (now - lastMs < SG_SWEEP_PERIOD_MS) {
      delay(1);
      continue;
    }
    lastMs = now;
    const int32_t pos = stepper->getCurrentPosition();
    if (pos < cruise0 || pos > cruise1 || n >= maxN) {
      continue;
    }
    const uint16_t sg = readStallGuard();
    if (sg == 0xFFFF) {
      continue;
    }
    // Homing ignores the first samples after reaching speed.
    if (settle < SG_MED_SETTLE) {
      settle++;
      continue;
    }
    buf[n++] = sg;
  }
  return n;
}

static void summarizeCruiseSg(const uint16_t *buf, uint8_t n, float &avg, float &amp,
                              uint16_t &outMin, uint16_t &outMax) {
  avg = 0;
  amp = 0;
  outMin = 0;
  outMax = 0;
  if (n == 0) {
    return;
  }
  uint32_t sum = 0;
  outMin = buf[0];
  outMax = buf[0];
  for (uint8_t i = 0; i < n; i++) {
    sum += buf[i];
    if (buf[i] < outMin) {
      outMin = buf[i];
    }
    if (buf[i] > outMax) {
      outMax = buf[i];
    }
  }
  avg = (float)sum / (float)n;
  amp = (float)(outMax - outMin) * 0.5f;
}

// Re-send the whole curve so Teleplot's time window does not drop earlier speeds.
static void plotSgSweepXy(const uint32_t *hz, const float *avg, const float *amp, uint8_t n) {
  if (n == 0) {
    return;
  }
  Serial.print(">sg_avg:");
  for (uint8_t i = 0; i < n; i++) {
    if (i) {
      Serial.print(';');
    }
    Serial.print(hz[i]);
    Serial.print(':');
    Serial.print(avg[i], 1);
  }
  Serial.println("|xy,clr");

  Serial.print(">sg_amp:");
  for (uint8_t i = 0; i < n; i++) {
    if (i) {
      Serial.print(';');
    }
    Serial.print(hz[i]);
    Serial.print(':');
    Serial.print(amp[i], 1);
  }
  Serial.println("|xy,clr");
}

void sgSpeedSweep(int32_t moveSteps) {
  if (!stepper) {
    say("no stepper");
    return;
  }
  if (moveSteps < 1500) {
    moveSteps = 1500;
  }
  stopMotion();

  const bool wasManual = stealthFrozen;
  const uint8_t saveOfs = frozenOfs;
  const uint8_t saveGrad = frozenGrad;
  if (wasManual) {
    unfreezeStealthChop();
  }

  const uint32_t oldSpeed = moveSpeedHz;
  const int32_t oldAccel = moveAccel;
  const int32_t home = stepper->getCurrentPosition();

  say("");
  say("SG SPEED SWEEP  StealthChop AUTO");
  Serial.print("extend ");
  Serial.print(moveSteps);
  Serial.print("  accel=");
  Serial.print(SG_SWEEP_ACCEL);
  Serial.print("  Hz ");
  Serial.print(SG_SWEEP_HZ_LO);
  Serial.print("..");
  Serial.print(SG_SWEEP_HZ_HI);
  Serial.print(" step ");
  Serial.println(SG_SWEEP_HZ_STEP);
  say("Teleplot: >sg_avg:Hz:mean|xy  and  >sg_amp:Hz:amp|xy   (x to abort)");
  Serial.println("speed_hz,sg_avg,sg_amp,n,min,max");

  uint32_t hzOut[SG_SWEEP_MAX_SPEEDS];
  float avgOut[SG_SWEEP_MAX_SPEEDS];
  float ampOut[SG_SWEEP_MAX_SPEEDS];
  uint8_t nOut = 0;

  bool aborted = false;
  for (uint32_t hz = SG_SWEEP_HZ_LO; hz <= SG_SWEEP_HZ_HI && !aborted;
       hz += SG_SWEEP_HZ_STEP) {
    uint16_t buf[SG_SWEEP_MAX_N];
    uint8_t n = collectCruiseSg(moveSteps, hz, SG_SWEEP_ACCEL, buf, SG_SWEEP_MAX_N, aborted);
    if (aborted) {
      say("aborted");
      break;
    }

    float avg = 0, amp = 0;
    uint16_t mn = 0, mx = 0;
    summarizeCruiseSg(buf, n, avg, amp, mn, mx);

    Serial.print(hz);
    Serial.print(",");
    Serial.print(avg, 1);
    Serial.print(",");
    Serial.print(amp, 1);
    Serial.print(",");
    Serial.print(n);
    Serial.print(",");
    Serial.print(mn);
    Serial.print(",");
    Serial.println(mx);

    if (n > 0 && nOut < SG_SWEEP_MAX_SPEEDS) {
      hzOut[nOut] = hz;
      avgOut[nOut] = avg;
      ampOut[nOut] = amp;
      nOut++;
      plotSgSweepXy(hzOut, avgOut, ampOut, nOut);
    }

    stepper->setSpeedInHz(hz);
    stepper->setAcceleration((int32_t)SG_SWEEP_ACCEL);
    stepper->moveTo(home);
    waitStepperIdle(20000);
    if (pollAbortX()) {
      aborted = true;
      say("aborted");
      break;
    }
  }

  plotSgSweepXy(hzOut, avgOut, ampOut, nOut);

  stepper->moveTo(home);
  waitStepperIdle(20000);
  stepper->setSpeedInHz(oldSpeed);
  stepper->setAcceleration(oldAccel);
  if (wasManual) {
    applyManualStealth(saveOfs, saveGrad);
  }
  say(aborted ? "SG sweep stopped" : "SG sweep done");
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

uint16_t medianU16(const uint16_t *v, uint8_t n) {
  if (n == 0) {
    return 0;
  }
  uint16_t tmp[SG_CRUISE_WIN];
  for (uint8_t i = 0; i < n; i++) {
    tmp[i] = v[i];
  }
  sortU16(tmp, n);
  return tmp[n / 2];
}

void minMaxU16(const uint16_t *v, uint8_t n, uint16_t &outMin, uint16_t &outMax) {
  outMin = v[0];
  outMax = v[0];
  for (uint8_t i = 1; i < n; i++) {
    if (v[i] < outMin) {
      outMin = v[i];
    }
    if (v[i] > outMax) {
      outMax = v[i];
    }
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
  int32_t dest = pos + steps;  // + / extend toward travelMax (out)
  if (travelCalibrated) {
    const int32_t limit = travelMax - HOME_BACKOFF;
    if (limit - pos < steps / 2) {
      say("Not enough room to extend. Jog toward IN, then f again.");
      return false;
    }
    if (dest > limit) {
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

static void stopHere(int32_t *posOut) {
  const int32_t pos = stepper->getCurrentPosition();
  stepper->forceStopAndNewPosition(pos);
  if (posOut) {
    *posOut = pos;
  }
}

// dirSign is stepper counts. User '+'/out = +counts; user '-' /in = -counts.
// Trip is DIAG (on-chip SG compare). UART SG is plotted only.
// *firstStallPos is the stall stop position.
bool moveUntilStall(int dirSign, int32_t maxSteps, const char *label,
                    int32_t *firstStallPos) {
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
  Serial.print("  DIAG=GP");
  Serial.print(DIAG_PIN);
  Serial.print("  speed=");
  Serial.print(HOME_SPEED_HZ);
  Serial.print(" accel=");
  Serial.print(HOME_ACCEL);
  Serial.print("  arm_after=");
  Serial.println(HOME_STALL_ARM_STEPS);

  stepper->move((int32_t)dirSign * maxSteps);

  uint16_t n = 0;
  uint8_t uartHits = 0;
  uint16_t lastSg = 0xFFFF;
  bool stalled = false;
  bool diagArmed = false;
  bool sgSeenHealthy = false;
  bool loggedWaiting = false;
  const char *source = nullptr;
  int32_t firstHitPos = 0;
  uint32_t lastPlotMs = 0;
  uint16_t sgWin[SG_CRUISE_WIN];
  uint8_t sgWinN = 0;
  uint8_t sgWinI = 0;
  uint16_t freeSgMed = 0;
  uint16_t tripHigh = 0;
  uint16_t prevMed = 0;
  uint8_t medSettleHits = 0;

  while (stepper->isRunning()) {
    const int32_t posNow = stepper->getCurrentPosition();
    const int32_t moved = (posNow > startPos) ? (posNow - startPos) : (startPos - posNow);

    if (diagArmed && diagStallPulse) {
      stopHere(&firstHitPos);
      stalled = true;
      source = "DIAG";
      break;
    }

    const uint32_t now = millis();
    if (now - lastPlotMs < HOME_SG_PLOT_MS) {
      continue;
    }
    lastPlotMs = now;

    const uint32_t tstep = readTstep();
    const uint16_t sg = readStallGuard();
    lastSg = sg;
    n++;
    const int32_t pos = stepper->getCurrentPosition();
    const bool velocityValid = stallGuardVelocityValid(tstep);

    if (velocityValid && sg != 0xFFFF) {
      sgWin[sgWinI] = sg;
      sgWinI = (uint8_t)((sgWinI + 1) % SG_CRUISE_WIN);
      if (sgWinN < SG_CRUISE_WIN) {
        sgWinN++;
      }
    }

    const uint16_t sgMed = medianU16(sgWin, sgWinN);
    if (!diagArmed && moved >= HOME_STALL_ARM_STEPS && sgWinN >= SG_CRUISE_WIN &&
        sgMed >= SG_READY_MIN) {
      const uint16_t medTol = (uint16_t)(sgMed / 8 + 6);
      const uint16_t medDiff = (prevMed > sgMed) ? (uint16_t)(prevMed - sgMed)
                                                 : (uint16_t)(sgMed - prevMed);
      if (prevMed != 0 && medDiff <= medTol) {
        medSettleHits++;
      } else {
        medSettleHits = 0;
      }
      prevMed = sgMed;

      uint16_t lo = 0, hi = 0;
      minMaxU16(sgWin, sgWinN, lo, hi);

      if (medSettleHits >= SG_MED_SETTLE) {
        // Oscillation is normal. Trip outside the trough/peak envelope.
        uint16_t tripLow = lo;
        const uint16_t lowMargin = (uint16_t)((lo / 6) + 6);
        if (tripLow > lowMargin) {
          tripLow = (uint16_t)(tripLow - lowMargin);
        } else {
          tripLow = 1;
        }
        uint16_t autoY = tripLow / 2;
        if (autoY < 1) {
          autoY = 1;
        }
        if (autoY > 255) {
          autoY = 255;
        }
        freeSgMed = sgMed;
        tripHigh = (uint16_t)(hi + (hi / 6) + 10);
        setupStallGuard((uint8_t)autoY);
        armDiag();
        diagArmed = true;
        Serial.print("  stall detect ON  sg med=");
        Serial.print(sgMed);
        Serial.print("  osc=");
        Serial.print(lo);
        Serial.print("..");
        Serial.print(hi);
        Serial.print("  trip_below=");
        Serial.print(2 * sgThreshold);
        Serial.print("  trip_above=");
        Serial.print(tripHigh);
        Serial.print("  pwm_ofs=");
        Serial.print(driver.pwm_ofs_auto());
        Serial.print("  pwm_grad=");
        Serial.println(driver.pwm_grad_auto());
      } else if (!loggedWaiting) {
        loggedWaiting = true;
        Serial.print("  waiting for SG mean to settle (med=");
        Serial.print(sgMed);
        Serial.print(" osc=");
        Serial.print(lo);
        Serial.print("..");
        Serial.print(hi);
        Serial.println(") — oscillation is OK");
      }
    }

    const uint16_t tripNow = (uint16_t)(2 * sgThreshold);
    if (velocityValid && sg != 0xFFFF && sg >= tripNow) {
      sgSeenHealthy = true;
    }
    const bool dropHit = diagArmed && sgSeenHealthy && velocityValid && isStalled(sg);
    const bool riseHit = diagArmed && velocityValid && sg != 0xFFFF && tripHigh > 0 &&
                         sg > tripHigh;
    const bool uartHit = dropHit || riseHit;
    if (uartHit) {
      uartHits++;
    } else {
      uartHits = 0;
    }

    plotHomeSample(sg, tripNow, (diagArmed && (diagStallPulse || diagPinHigh())) ? 1 : 0, pos);

    if (diagArmed && diagStallPulse) {
      stopHere(&firstHitPos);
      stalled = true;
      source = "DIAG";
      break;
    }
    if (uartHits >= STALL_CONFIRM) {
      stopHere(&firstHitPos);
      stalled = true;
      source = riseHit ? "UART_UP" : "UART";
      break;
    }
  }
  if (diagArmed) {
    disarmDiag();
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
    Serial.print("STALL ");
    Serial.print(source);
    Serial.print("  pos=");
    Serial.println(firstHitPos);
    if (firstStallPos) {
      *firstStallPos = firstHitPos;
    }
  } else {
    Serial.println("NO STALL (hit max travel or stopped)");
    say("Hard end often raises SG (skipped steps), not a dip. Check trip_above vs the graph.");
  }

  return stalled;
}

void homeBothEnds() {
  if (!stepper) {
    say("no stepper");
    return;
  }
  stopMotion();
  originSet = false;

  say("StallGuard home to 0 (in / retract) — stop is DIAG on GP6");
  say("Plot: Teleplot >sg: (UART, slow) and >diag: (hardware pulse)");
  setupStallGuard(sgThreshold);
  Serial.print("SGTHRS=");
  Serial.print(sgThreshold);
  Serial.print("  home speed=");
  Serial.print(HOME_SPEED_HZ);
  Serial.print(" Hz  accel=");
  Serial.println(HOME_ACCEL);

  say("Seeking RETRACTED / IN (-) ...");
  if (!moveUntilStall(-1, HOME_MAX_TRAVEL, "RETRACT", nullptr)) {
    say("RETRACT: no stall — raise y (more sensitive) or check mechanics");
    travelCalibrated = false;
    originSet = false;
    stepper->setSpeedInHz(moveSpeedHz);
    stepper->setAcceleration(moveAccel);
    return;
  }
  stepper->setCurrentPosition(0);
  travelMin = 0;
  travelMax = 0;
  travelCalibrated = false;
  originSet = true;
  plotPos(0);
  say("Retracted (in) = 0");
  stepper->setSpeedInHz(moveSpeedHz);
  stepper->setAcceleration(moveAccel);
  say("Home done. +extends -retracts; g 0 = stall end");
}

void setupStepper() {
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    say("FastAccelStepper failed to claim STEP pin");
    return;
  }
  // Flip DIR so +counts = out (extend) and -counts = in (retract).
  stepper->setDirectionPin(DIR_PIN, false);
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
  driver.pwm_autograd(true);
  driver.pwm_reg(2);  // softer auto current loop — less SG wander
  stealthFrozen = false;
  driver.TPWMTHRS(0);
  driver.semin(0);
  driver.en_spreadCycle(false);
  driver.pdn_disable(true);
  driver.VACTUAL(0);
  driver.rms_current(rmsMa, IHOLD_FRACTION);
  setupStallGuard(sgThreshold);

  uartTest();
  if (STEALTH_MANUAL_OFS != 0) {
    applyManualStealth(STEALTH_MANUAL_OFS, STEALTH_MANUAL_GRAD);
  }
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
    Serial.print("  DIAG=");
    Serial.print(diagPinHigh() ? "HIGH" : "LOW");
    Serial.print("  pwm_ofs=");
    Serial.print(driver.pwm_ofs_auto());
    Serial.print("  pwm_grad=");
    Serial.print(driver.pwm_grad_auto());
    Serial.print("  stealth=");
    Serial.print(stealthFrozen ? "MANUAL" : "AUTO");
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
  } else if (c == 'k') {
    String rest = cmd.substring(1);
    rest.trim();
    if (rest.length() == 0) {
      printAutoPwm();
    } else if (rest == "copy" || rest == "COPY") {
      freezeStealthChop();
    } else {
      int ofs = 0;
      int grad = 0;
      if (sscanf(rest.c_str(), "%d %d", &ofs, &grad) == 2 && ofs >= 0 && grad >= 0 &&
          ofs <= 255 && grad <= 255) {
        applyManualStealth((uint8_t)ofs, (uint8_t)grad);
      } else {
        say("usage: k | k copy | k <ofs> <grad>");
      }
    }
  } else if (c == 'K') {
    unfreezeStealthChop();
  } else if (c == 'f' || c == 'F') {
    int32_t steps = FINGER_TEST_STEPS;
    if (cmd.length() > 1) {
      const int32_t v = cmd.substring(1).toInt();
      if (v > 0) {
        steps = v;
      }
    }
    fingerStallTest(steps);
  } else if (c == 'e' || c == 'E') {
    int32_t steps = SG_SWEEP_STEPS;
    if (cmd.length() > 1) {
      const int32_t v = cmd.substring(1).toInt();
      if (v > 0) {
        steps = v;
      }
    }
    sgSpeedSweep(steps);
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
  } else if (c == '0') {
    if (!stepper) {
      return;
    }
    stopMotion();
    const int32_t here = stepper->getCurrentPosition();
    if (travelCalibrated) {
      travelMax -= here;
      if (travelMax <= 0) {
        travelCalibrated = false;
        travelMax = 0;
        say("travel limits cleared (new 0 is past old out)");
      }
    }
    travelMin = 0;
    stepper->setCurrentPosition(0);
    originSet = true;
    plotPos(0);
    Serial.print("zeroed here (was pos ");
    Serial.print(here);
    Serial.println(")");
    printStatus();
  } else if (c == '+' || c == '-') {
    int32_t delta = (c == '+') ? stepSize : -stepSize;
    if (cmd.length() > 1) {
      int32_t mag = cmd.substring(1).toInt();
      if (mag < 0) {
        mag = -mag;
      }
      if (mag > 0) {
        delta = (c == '+') ? mag : -mag;
      }
    }
    stopMotion();
    if (stepper) {
      stepper->move(delta);
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
      driver.rms_current(rmsMa, IHOLD_FRACTION);
      Serial.print("rms_mA=");
      Serial.println(rmsMa);
      if (stealthFrozen) {
        unfreezeStealthChop();
        say("current changed — StealthChop back to AUTO (re-learn, then k or H to lock)");
      }
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
  pinMode(DIAG_PIN, INPUT_PULLDOWN);
  delay(1500);
  say("TMC2209 Serial2 UART + StallGuard (DIAG on GP6)");

  setupStepper();
  setupDriver();
  printHelp();
  say("Setup complete. Wire DIAG to GP6, tune z/y, then H.");
}

void loop() {
  serviceSweep();
  serviceManualCurrentGuard();

  if (originSet && stepper && stepper->isRunning()) {
    static uint32_t lastPosPlotMs = 0;
    const uint32_t now = millis();
    if (now - lastPosPlotMs >= 40) {
      lastPosPlotMs = now;
      plotPos(stepper->getCurrentPosition());
    }
  }

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
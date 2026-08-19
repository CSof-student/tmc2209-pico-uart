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

static const int32_t HOME_BACKOFF = 32;
static const int32_t HOME_CHUNK = 40;
static const int32_t HOME_MAX_TRAVEL = 20000;
static const uint32_t HOME_SPEED_HZ = 150;
static const uint32_t HOME_ACCEL = 300;

TMC2209Stepper driver(&TMC_SERIAL, R_SENSE, DRIVER_ADDRESS);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

int32_t stepSize = DEFAULT_STEP_SIZE;
uint16_t rmsMa = DEFAULT_RMS_MA;
uint16_t motor_microsteps = DEFAULT_MICROSTEPS;
uint32_t moveSpeedHz = DEFAULT_SPEED_HZ;
int32_t moveAccel = DEFAULT_ACCEL;
uint8_t sgThreshold = 50;  // higher = more sensitive
bool travelCalibrated = false;
int32_t travelMin = 0;
int32_t travelMax = 0;
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
  Serial.println("  z SG read    y <0-255> SG thresh    H home both ends");
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
  Serial.println(sgThreshold);
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
  driver.TCOOLTHRS(0xFFFFF);  // StallGuard active at homing/move speeds
  driver.SGTHRS(sgThreshold);
}

uint16_t readStallGuard() {
  const uint16_t sg = driver.SG_RESULT() & 0x3FF;
  if (driver.CRCerror) {
    return 0xFFFF;
  }
  return sg;
}

void waitStepperIdle() {
  if (!stepper) {
    return;
  }
  while (stepper->isRunning()) {
    delay(1);
  }
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
bool moveUntilStall(int dirSign, int32_t maxSteps) {
  if (!stepper || maxSteps <= 0) {
    return false;
  }

  stepper->setSpeedInHz(HOME_SPEED_HZ);
  stepper->setAcceleration(HOME_ACCEL);

  int32_t moved = 0;
  bool stalled = false;
  while (moved < maxSteps) {
    const int32_t chunk = (maxSteps - moved > HOME_CHUNK) ? HOME_CHUNK : (maxSteps - moved);
    stepper->move((int32_t)dirSign * chunk);
    waitStepperIdle();
    delay(8);

    const uint16_t sg = readStallGuard();
    Serial.print("  sg=");
    Serial.print(sg);
    Serial.print(" pos=");
    Serial.println(stepper->getCurrentPosition());

    if (sg != 0xFFFF && sg <= (uint16_t)sgThreshold) {
      stalled = true;
      break;
    }
    moved += chunk;
  }

  if (stalled) {
    stepper->move((int32_t)(-dirSign) * HOME_BACKOFF);
    waitStepperIdle();
  }

  stepper->setSpeedInHz(moveSpeedHz);
  stepper->setAcceleration(moveAccel);
  return stalled;
}

void homeBothEnds() {
  if (!stepper) {
    say("no stepper");
    return;
  }

  say("StallGuard home (+retract / -extend)...");
  setupStallGuard(sgThreshold);
  Serial.print("SGTHRS=");
  Serial.println(sgThreshold);

  say("Seeking RETRACTED (+) ...");
  if (!moveUntilStall(-1, HOME_MAX_TRAVEL)) {
    say("RETRACT: no stall — raise y (more sensitive) or check mechanics");
    travelCalibrated = false;
    return;
  }
  stepper->setCurrentPosition(0);
  travelMin = 0;
  say("Retracted end = 0");
  delay(100);

  say("Seeking EXTENDED (-) ...");
  if (!moveUntilStall(+1, HOME_MAX_TRAVEL)) {
    say("EXTEND: no stall — raise y or check mechanics");
    travelCalibrated = false;
    return;
  }
  travelMax = stepper->getCurrentPosition();
  if (travelMax < HOME_BACKOFF * 2) {
    say("Travel too short — tune y / speed / current");
    travelCalibrated = false;
    return;
  }
  travelCalibrated = true;
  Serial.print("Travel 0 (retract) .. ");
  Serial.print(travelMax);
  Serial.println(" (extend)");
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
    const uint16_t sg = readStallGuard();
    Serial.print("SG_RESULT=");
    Serial.print(sg);
    Serial.print("  SGTHRS=");
    Serial.println(sgThreshold);
  } else if (c == 'y' || c == 'Y') {
    const int v = cmd.substring(1).toInt();
    if (v >= 0 && v <= 255) {
      setupStallGuard((uint8_t)v);
      Serial.print("SGTHRS=");
      Serial.println(sgThreshold);
    }
  } else if (c == 'g' || c == 'G') {
    if (!stepper) {
      return;
    }
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
    if (stepper) {
      stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
    }
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

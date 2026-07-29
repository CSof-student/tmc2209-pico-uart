/*
  Pico + TMC2209 (UART config) + FastAccelStepper (STEP/DIR motion)

  Wiring:
    STEP -> GP3
    DIR  -> GP2
    EN   -> GND (or set ENABLE_PIN)
    UART -> GP4 (TX) -> TMC Rx ,  GP5 (RX) <- TMC Tx
    VIO  -> 3.3V
    VM   -> motor supply
    GND  -> common
    CLK  -> leave unconnected

  Serial Monitor 115200, Newline. Type h for help.
  IMPORTANT: do not run t/c/m/i until USB serial responds to h.
*/

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

// --- Motion pins ---
static const uint8_t STEP_PIN = 3;
static const uint8_t DIR_PIN = 2;
static const int8_t ENABLE_PIN = -1;  // -1 = EN hard-wired to GND

// --- TMC2209 UART (Serial1) ---
static const uint8_t TMC_TX_PIN = 4;  // Pico TX -> TMC Rx
static const uint8_t TMC_RX_PIN = 5;  // Pico RX <- TMC Tx
static const uint8_t DRIVER_ADDRESS = 0b00;
static const float R_SENSE = 0.11f;

static const uint16_t DEFAULT_RMS_MA = 300;
static const uint16_t DEFAULT_MICROSTEPS = 16;
static const uint32_t DEFAULT_SPEED_HZ = 200;
static const uint32_t DEFAULT_ACCEL = 400;
static const int32_t DEFAULT_STEP_SIZE = 8;

#define SERIAL_PORT Serial1

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

int32_t stepSize = DEFAULT_STEP_SIZE;
uint16_t rmsMa = DEFAULT_RMS_MA;
bool driverConfigured = false;
String line;

void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  h           help (USB only, no TMC UART)"));
  Serial.println(F("  p           motion status (USB only, no TMC UART)"));
  Serial.println(F("  + / -       nudge +/- stepSize"));
  Serial.println(F("  n <val>     set stepSize"));
  Serial.println(F("  s <hz>      set speed (steps/s)"));
  Serial.println(F("  a <val>     set acceleration"));
  Serial.println(F("  g <pos>     goto absolute position"));
  Serial.println(F("  z           zero position"));
  Serial.println(F("  x           emergency stop"));
  Serial.println(F("  t           UART test (may pause if wiring bad)"));
  Serial.println(F("  c <mA>      set RMS current via UART"));
  Serial.println(F("  m <usteps>  set microsteps via UART"));
  Serial.println(F("  i           driver status via UART"));
  Serial.println(F("  u           apply default TMC UART settings"));
}

void printMotionStatus() {
  Serial.print(F("pos="));
  Serial.print(stepper ? stepper->getCurrentPosition() : 0);
  if (stepper) {
    Serial.print(F("  target="));
    Serial.print(stepper->targetPos());
    Serial.print(F("  running="));
    Serial.print(stepper->isRunning() ? F("yes") : F("no"));
    Serial.print(F("  speedHz="));
    Serial.print(stepper->getSpeedInMilliHz() / 1000UL);
    Serial.print(F("  accel="));
    Serial.print(stepper->getAcceleration());
  }
  Serial.print(F("  stepSize="));
  Serial.print(stepSize);
  Serial.print(F("  rms_mA(setpoint)="));
  Serial.print(rmsMa);
  Serial.print(F("  driverConfigured="));
  Serial.println(driverConfigured ? F("yes") : F("no"));
}

void ensureUartPort() {
  static bool started = false;
  if (started) {
    return;
  }
  SERIAL_PORT.setTX(TMC_TX_PIN);
  SERIAL_PORT.setRX(TMC_RX_PIN);
  SERIAL_PORT.begin(115200);
  delay(20);
  started = true;
}

void testUart() {
  ensureUartPort();
  Serial.println(F("UART: reading version (if this hangs, fix Rx/Tx wiring / power)"));
  Serial.flush();
  delay(10);

  const uint8_t ver = driver.version();

  Serial.print(F("UART version=0x"));
  Serial.println(ver, HEX);
  Serial.flush();

  if (ver == 0x21 || ver == 0x20) {
    Serial.println(F("UART -> OK"));
  } else {
    Serial.println(F("UART -> FAIL (expected 0x21). Check:"));
    Serial.println(F("  Pico GP4 TX -> TMC Rx"));
    Serial.println(F("  Pico GP5 RX <- TMC Tx"));
    Serial.println(F("  VIO=3.3V, common GND, EN=GND, VM on"));
  }
}

void applyDriverDefaults() {
  ensureUartPort();
  Serial.println(F("UART: applying driver defaults..."));
  Serial.flush();

  driver.begin();
  driver.toff(5);
  driver.rms_current(rmsMa);
  driver.microsteps(DEFAULT_MICROSTEPS);
  driver.pwm_autoscale(true);
  driver.en_spreadCycle(false);

  uint8_t hold = (uint8_t)(driver.irun() * 3 / 10);
  if (hold < 1) {
    hold = 1;
  }
  driver.ihold(hold);
  driver.iholddelay(2);
  driverConfigured = true;

  Serial.println(F("UART: defaults applied"));
}

void printDriverStatus() {
  ensureUartPort();
  Serial.println(F("UART: reading driver status..."));
  Serial.flush();

  Serial.print(F("version=0x"));
  Serial.print(driver.version(), HEX);
  Serial.print(F("  microsteps="));
  Serial.print(driver.microsteps());
  Serial.print(F("  SG_RESULT="));
  Serial.println(driver.SG_RESULT());
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  delay(200);

  // USB first — never touch TMC UART during boot so `h` always works.
  Serial.println(F("\nUSB serial OK"));
  Serial.println(F("TMC UART is NOT initialized yet. Try: h"));
  Serial.flush();

  if (ENABLE_PIN >= 0) {
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
  }

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    Serial.println(F("ERROR: FastAccelStepper could not claim STEP pin"));
  } else {
    stepper->setDirectionPin(DIR_PIN, true, 5);
    if (ENABLE_PIN >= 0) {
      stepper->setEnablePin(ENABLE_PIN, true);
      stepper->setAutoEnable(false);
      stepper->enableOutputs();
    }
    stepper->setSpeedInHz(DEFAULT_SPEED_HZ);
    stepper->setAcceleration(DEFAULT_ACCEL);
    stepper->setCurrentPosition(0);
  }

  Serial.println(F("STEP=GP3 DIR=GP2  UART TX=GP4 RX=GP5"));
  printHelp();
  printMotionStatus();
  Serial.println(F("Next: h  then  t  (only after h works)"));
  Serial.print(F("> "));
  Serial.flush();
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    return;
  }

  const char c = cmd.charAt(0);

  if (c == '+' || c == '-') {
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
    printMotionStatus();
  } else if (c == 'n' || c == 'N') {
    const int32_t v = cmd.substring(1).toInt();
    if (v > 0) {
      stepSize = v;
      Serial.print(F("stepSize="));
      Serial.println(stepSize);
    }
  } else if (c == 's' || c == 'S') {
    const uint32_t v = (uint32_t)cmd.substring(1).toInt();
    if (v > 0 && stepper) {
      stepper->setSpeedInHz(v);
      Serial.print(F("speedHz="));
      Serial.println(v);
    }
  } else if (c == 'a' || c == 'A') {
    const int32_t v = cmd.substring(1).toInt();
    if (v > 0 && stepper) {
      stepper->setAcceleration(v);
      Serial.print(F("accel="));
      Serial.println(v);
    }
  } else if (c == 'g' || c == 'G') {
    const int32_t pos = cmd.substring(1).toInt();
    if (stepper) {
      stepper->moveTo(pos);
    }
    Serial.print(F("goto "));
    Serial.println(pos);
    printMotionStatus();
  } else if (c == 'z' || c == 'Z') {
    if (stepper) {
      stepper->setCurrentPosition(0);
    }
    Serial.println(F("position zeroed"));
  } else if (c == 'p' || c == 'P') {
    printMotionStatus();
  } else if (c == 'u' || c == 'U') {
    applyDriverDefaults();
  } else if (c == 'c' || c == 'C') {
    const int v = cmd.substring(1).toInt();
    if (v > 0 && v < 2000) {
      rmsMa = (uint16_t)v;
      ensureUartPort();
      Serial.println(F("UART: setting current..."));
      Serial.flush();
      driver.rms_current(rmsMa);
      uint8_t hold = (uint8_t)(driver.irun() * 3 / 10);
      if (hold < 1) {
        hold = 1;
      }
      driver.ihold(hold);
      driverConfigured = true;
      Serial.print(F("rms_current_mA="));
      Serial.println(rmsMa);
    } else {
      Serial.println(F("usage: c <mA>   e.g. c 300"));
    }
  } else if (c == 'm' || c == 'M') {
    const int v = cmd.substring(1).toInt();
    if (v == 8 || v == 16 || v == 32 || v == 64 || v == 128 || v == 256) {
      ensureUartPort();
      Serial.println(F("UART: setting microsteps..."));
      Serial.flush();
      driver.microsteps(v);
      Serial.print(F("microsteps command sent: "));
      Serial.println(v);
    } else {
      Serial.println(F("usage: m <8|16|32|64|128|256>"));
    }
  } else if (c == 't' || c == 'T') {
    testUart();
  } else if (c == 'i' || c == 'I') {
    printDriverStatus();
    printMotionStatus();
  } else if (c == 'x' || c == 'X') {
    if (stepper) {
      stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
    }
    Serial.println(F("STOPPED"));
  } else if (c == 'h' || c == 'H' || c == '?') {
    printHelp();
  } else {
    Serial.println(F("unknown command (h for help)"));
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

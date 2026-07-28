/*
  Pico + TMC2209 (UART config) + FastAccelStepper (STEP/DIR motion)

  Wiring (see README):
    STEP -> GP3
    DIR  -> GP2
    EN   -> GND (or GP6 if ENABLE_PIN >= 0)
    UART -> GP4 (TX) + GP5 (RX) with 1k half-duplex network to PDN_UART
    VIO  -> 3.3V
    VM   -> motor supply
    GND  -> common

  Serial Monitor 115200, Newline. Type h for help.
*/

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

// --- Motion pins ---
static const uint8_t STEP_PIN = 3;
static const uint8_t DIR_PIN = 2;
// Set to -1 if EN is hard-wired to GND on the driver
static const int8_t ENABLE_PIN = -1;

// --- TMC2209 UART pins (Serial1) ---
static const uint8_t TMC_TX_PIN = 4;  // Pico TX -> 1k -> PDN_UART
static const uint8_t TMC_RX_PIN = 5;  // Pico RX -> PDN_UART directly

// TMC2209 UART address from MS1/MS2 jumpers (both open/low => 0b00)
static const uint8_t DRIVER_ADDRESS = 0b00;

// Sense resistor on most SilentStepStick / BTT TMC2209 modules
static const float R_SENSE = 0.11f;

// Conservative NEMA-8 starting current (mA RMS). Raise carefully.
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
String line;

void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  + / -       nudge +/- stepSize"));
  Serial.println(F("  n <val>     set stepSize"));
  Serial.println(F("  s <hz>      set speed (steps/s)"));
  Serial.println(F("  a <val>     set acceleration"));
  Serial.println(F("  g <pos>     goto absolute position"));
  Serial.println(F("  z           zero position"));
  Serial.println(F("  c <mA>      set RMS current via UART (e.g. c 350)"));
  Serial.println(F("  m <usteps>  set microsteps (8,16,32,64,128,256)"));
  Serial.println(F("  t           UART communication test"));
  Serial.println(F("  i           print driver + motion status"));
  Serial.println(F("  x           emergency stop"));
  Serial.println(F("  h           help"));
}

void printStatus() {
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
  Serial.print(F("  rms_mA="));
  Serial.print(rmsMa);
  Serial.print(F("  microsteps="));
  Serial.print(driver.microsteps());
  Serial.print(F("  SG_RESULT="));
  Serial.println(driver.SG_RESULT());
}

void testUart() {
  const uint8_t ver = driver.version();
  const uint32_t ifcnt = driver.IFCNT();
  Serial.print(F("UART version=0x"));
  Serial.print(ver, HEX);
  Serial.print(F("  IFCNT="));
  Serial.print(ifcnt);
  Serial.print(F("  -> "));
  if (ver == 0x21 || ver == 0x20) {
    Serial.println(F("OK (TMC2209/2208 family responded)"));
  } else {
    Serial.println(F("FAIL (check 1k network, address jumpers, VIO, PDN_UART)"));
  }
}

void applyDriverDefaults() {
  driver.begin();
  driver.toff(5);                 // enable driver output stage via chopper
  driver.rms_current(rmsMa);      // digital current (pot not required)
  driver.microsteps(DEFAULT_MICROSTEPS);
  driver.pwm_autoscale(true);     // StealthChop
  driver.en_spreadCycle(false);   // quiet mode
  // Hold current ~30% of run for less heat when idle
  uint8_t hold = (uint8_t)(driver.irun() * 3 / 10);
  if (hold < 1) {
    hold = 1;
  }
  driver.ihold(hold);
  driver.iholddelay(2);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  delay(200);

  if (ENABLE_PIN >= 0) {
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);  // TMC EN is active low
  }

  // Hardware UART to TMC2209 single-wire interface
  SERIAL_PORT.setTX(TMC_TX_PIN);
  SERIAL_PORT.setRX(TMC_RX_PIN);
  SERIAL_PORT.begin(115200);
  delay(50);

  applyDriverDefaults();

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    Serial.println(F("ERROR: FastAccelStepper could not claim STEP pin"));
    while (true) {
      delay(1000);
    }
  }

  stepper->setDirectionPin(DIR_PIN, /*dirHighCountsUp=*/true, /*dir_change_delay_us=*/5);
  if (ENABLE_PIN >= 0) {
    stepper->setEnablePin(ENABLE_PIN, /*low_active_enables=*/true);
    stepper->setAutoEnable(false);
    stepper->enableOutputs();
  }

  stepper->setSpeedInHz(DEFAULT_SPEED_HZ);
  stepper->setAcceleration(DEFAULT_ACCEL);
  stepper->setCurrentPosition(0);

  Serial.println(F("\nTMC2209 UART + FastAccelStepper ready."));
  Serial.println(F("STEP=GP3 DIR=GP2  UART TX=GP4 RX=GP5"));
  testUart();
  printHelp();
  printStatus();
  Serial.print(F("> "));
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
  } else if (c == 'z' || c == 'Z') {
    if (stepper) {
      stepper->setCurrentPosition(0);
    }
    Serial.println(F("position zeroed"));
  } else if (c == 'c' || c == 'C') {
    const int v = cmd.substring(1).toInt();
    if (v > 0 && v < 2000) {
      rmsMa = (uint16_t)v;
      driver.rms_current(rmsMa);
      uint8_t hold = (uint8_t)(driver.irun() * 3 / 10);
      if (hold < 1) {
        hold = 1;
      }
      driver.ihold(hold);
      Serial.print(F("rms_current_mA="));
      Serial.println(rmsMa);
    } else {
      Serial.println(F("usage: c <mA>   e.g. c 300"));
    }
  } else if (c == 'm' || c == 'M') {
    const int v = cmd.substring(1).toInt();
    if (v == 8 || v == 16 || v == 32 || v == 64 || v == 128 || v == 256) {
      driver.microsteps(v);
      Serial.print(F("microsteps="));
      Serial.println(driver.microsteps());
    } else {
      Serial.println(F("usage: m <8|16|32|64|128|256>"));
    }
  } else if (c == 't' || c == 'T') {
    testUart();
  } else if (c == 'i' || c == 'I') {
    printStatus();
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

  printStatus();
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

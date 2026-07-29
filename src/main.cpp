/*
  Pico + TMC2209 (UART config) + FastAccelStepper (STEP/DIR motion)

  Wiring (normal):
    STEP -> GP3
    DIR  -> GP2
    EN   -> GND
    Pico GP4 (TX) -> TMC Rx
    Pico GP5 (RX) <- TMC Tx
    VIO -> 3.3V , VM -> motor PSU , GND common
    CLK unconnected

  IMPORTANT: On TMC2209, MS1/MS2 set the UART *address*:
    MS1=0 MS2=0 -> addr 0
    MS1=1 MS2=0 -> addr 1
    MS1=0 MS2=1 -> addr 2
    MS1=1 MS2=1 -> addr 3
  Command `t` scans all 4 addresses.

  Serial 115200, Newline. Use `h` before any UART command.
*/

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

static const uint8_t STEP_PIN = 3;
static const uint8_t DIR_PIN = 2;
static const int8_t ENABLE_PIN = -1;

static const uint8_t TMC_TX_PIN_DEFAULT = 4;
static const uint8_t TMC_RX_PIN_DEFAULT = 5;
static const float R_SENSE = 0.11f;

static const uint16_t DEFAULT_RMS_MA = 300;
static const uint16_t DEFAULT_MICROSTEPS = 16;
static const uint32_t DEFAULT_SPEED_HZ = 200;
static const uint32_t DEFAULT_ACCEL = 400;
static const int32_t DEFAULT_STEP_SIZE = 8;

#define SERIAL_PORT Serial1

uint8_t tmcTxPin = TMC_TX_PIN_DEFAULT;
uint8_t tmcRxPin = TMC_RX_PIN_DEFAULT;
uint8_t driverAddress = 0b00;
bool uartPinsSwapped = false;
bool uartPortStarted = false;

// Recreated whenever address changes
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

void ensureUartPort() {
  if (uartPortStarted) {
    return;
  }
  SERIAL_PORT.setTX(tmcTxPin);
  SERIAL_PORT.setRX(tmcRxPin);
  SERIAL_PORT.begin(115200);
  delay(50);
  while (SERIAL_PORT.available()) {
    SERIAL_PORT.read();
  }
  uartPortStarted = true;
}

void restartUartPort() {
  if (uartPortStarted) {
    SERIAL_PORT.end();
    delay(20);
    uartPortStarted = false;
  }
  ensureUartPort();
}

uint8_t readVersionAtAddress(uint8_t addr) {
  recreateDriver(addr);
  ensureUartPort();
  while (SERIAL_PORT.available()) {
    SERIAL_PORT.read();
  }
  delay(5);
  return driver->version();
}

void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  h      help (no UART)"));
  Serial.println(F("  p      motion status (no UART)"));
  Serial.println(F("  + / -  nudge"));
  Serial.println(F("  n/s/a/g/z/x   stepSize/speed/accel/goto/zero/stop"));
  Serial.println(F("  t      scan UART addresses 0..3 (version)"));
  Serial.println(F("  w      swap Pico TX/RX pins and retry wiring"));
  Serial.println(F("  u      apply default TMC settings (uses last OK addr)"));
  Serial.println(F("  c mA   set RMS current"));
  Serial.println(F("  m N    set microsteps"));
  Serial.println(F("  i      driver status"));
  Serial.println(F("\nMS1/MS2 = UART address. If jumpers set for microsteps, addr may be 1/2/3."));
}

void printMotionStatus() {
  Serial.print(F("pos="));
  Serial.print(stepper ? stepper->getCurrentPosition() : 0);
  if (stepper) {
    Serial.print(F("  running="));
    Serial.print(stepper->isRunning() ? F("yes") : F("no"));
    Serial.print(F("  speedHz="));
    Serial.print(stepper->getSpeedInMilliHz() / 1000UL);
  }
  Serial.print(F("  stepSize="));
  Serial.print(stepSize);
  Serial.print(F("  addr="));
  Serial.print(driverAddress);
  Serial.print(F("  uartSwapped="));
  Serial.println(uartPinsSwapped ? F("yes") : F("no"));
}

void testUart() {
  ensureUartPort();
  Serial.println(F("UART scan: addresses 0..3"));
  Serial.print(F("  Pico TX=GP"));
  Serial.print(tmcTxPin);
  Serial.print(F(" -> should go to TMC Rx"));
  Serial.print(F(" | Pico RX=GP"));
  Serial.print(tmcRxPin);
  Serial.println(F(" <- TMC Tx"));
  Serial.flush();

  int found = -1;
  for (uint8_t addr = 0; addr < 4; addr++) {
    Serial.print(F("  addr "));
    Serial.print(addr);
    Serial.print(F(": version=0x"));
    Serial.flush();

    const uint8_t ver = readVersionAtAddress(addr);
    Serial.print(ver, HEX);

    if (ver == 0x21 || ver == 0x20) {
      Serial.println(F("  OK"));
      found = addr;
      break;
    }
    Serial.println(F("  fail"));
    delay(20);
  }

  if (found >= 0) {
    recreateDriver((uint8_t)found);
    Serial.print(F("Using address "));
    Serial.println(found);
    Serial.println(F("Next: u   then   c 300   then   +"));
  } else {
    Serial.println(F("No response on any address."));
    Serial.println(F("Try: w  (swap TX/RX) then t again"));
    Serial.println(F("Also check VIO=3.3V, GND common, EN=GND, VM on"));
  }
}

void swapUartPins() {
  uartPinsSwapped = !uartPinsSwapped;
  if (uartPinsSwapped) {
    // Swapped: treat silk labels as backwards
    tmcTxPin = TMC_RX_PIN_DEFAULT;  // GP5 as TX
    tmcRxPin = TMC_TX_PIN_DEFAULT;  // GP4 as RX
  } else {
    tmcTxPin = TMC_TX_PIN_DEFAULT;
    tmcRxPin = TMC_RX_PIN_DEFAULT;
  }
  Serial.print(F("UART pins now: TX=GP"));
  Serial.print(tmcTxPin);
  Serial.print(F(" RX=GP"));
  Serial.println(tmcRxPin);
  restartUartPort();
  Serial.println(F("Run t again"));
}

void applyDriverDefaults() {
  ensureUartPort();
  if (!driver) {
    recreateDriver(driverAddress);
  }
  Serial.println(F("UART: applying defaults..."));
  Serial.flush();

  driver->begin();
  driver->pdn_disable(true);       // PDN pin used for UART
  driver->I_scale_analog(false);   // digital current via UART
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

  Serial.print(F("Defaults applied at addr "));
  Serial.println(driverAddress);
}

void printDriverStatus() {
  ensureUartPort();
  if (!driver) {
    recreateDriver(driverAddress);
  }
  Serial.println(F("UART: driver status..."));
  Serial.flush();
  Serial.print(F("version=0x"));
  Serial.print(driver->version(), HEX);
  Serial.print(F("  microsteps="));
  Serial.print(driver->microsteps());
  Serial.print(F("  SG_RESULT="));
  Serial.println(driver->SG_RESULT());
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  delay(200);

  Serial.println(F("\nUSB serial OK (no TMC UART yet)"));
  Serial.flush();

  recreateDriver(0);

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

  printHelp();
  printMotionStatus();
  Serial.println(F("Next: t   (or w then t if needed)"));
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
  } else if (c == 'g' || c == 'G') {
    if (stepper) {
      stepper->moveTo(cmd.substring(1).toInt());
    }
  } else if (c == 'z' || c == 'Z') {
    if (stepper) {
      stepper->setCurrentPosition(0);
    }
  } else if (c == 'p' || c == 'P') {
    printMotionStatus();
  } else if (c == 'w' || c == 'W') {
    swapUartPins();
  } else if (c == 'u' || c == 'U') {
    applyDriverDefaults();
  } else if (c == 'c' || c == 'C') {
    const int v = cmd.substring(1).toInt();
    if (v > 0 && v < 2000) {
      rmsMa = (uint16_t)v;
      ensureUartPort();
      if (!driver) {
        recreateDriver(driverAddress);
      }
      Serial.println(F("UART: set current..."));
      Serial.flush();
      driver->rms_current(rmsMa);
      uint8_t hold = (uint8_t)(driver->irun() * 3 / 10);
      if (hold < 1) {
        hold = 1;
      }
      driver->ihold(hold);
      driverConfigured = true;
      Serial.print(F("rms_mA="));
      Serial.println(rmsMa);
    } else {
      Serial.println(F("usage: c 300"));
    }
  } else if (c == 'm' || c == 'M') {
    const int v = cmd.substring(1).toInt();
    if (v == 8 || v == 16 || v == 32 || v == 64 || v == 128 || v == 256) {
      ensureUartPort();
      if (!driver) {
        recreateDriver(driverAddress);
      }
      driver->microsteps(v);
      Serial.print(F("microsteps="));
      Serial.println(v);
    } else {
      Serial.println(F("usage: m 16"));
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
    Serial.println(F("unknown (h for help)"));
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

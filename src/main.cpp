/*
  TMC2209 UART (Serial2 / UART1) + FastAccelStepper.

  GP8/GP9 are UART1, so this uses Serial2 — Serial1 is UART0 and will hang
  if you setTX(8). No soft UART.

  Wiring:
    GP8 --[1k]--+---- TMC UART
    GP9 --------+
    STEP GP3, DIR GP2, EN -> GND
    VIO 3.3V, VM motor PSU, GND common

  USB 115200. Commands: t = UART test, h = help.
*/

#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

#define DIR_PIN    2
#define STEP_PIN   3
#define ENABLE_PIN -1

#define TX_PIN 8
#define RX_PIN 9

#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b00

#define TMC_SERIAL Serial2

TMC2209Stepper driver(&TMC_SERIAL, R_SENSE, DRIVER_ADDRESS);

int32_t move_to_step = 3200 * 5;
int32_t set_velocity = 3200;
int32_t set_accel = 3200 * 100;
int32_t set_current = 300;
uint16_t motor_microsteps = 16;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

void say(const char *msg) {
  Serial.println(msg);
  Serial.flush();
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
  stepper->setSpeedInHz(set_velocity);
  stepper->setAcceleration(set_accel);
  stepper->setCurrentPosition(0);
  say("Stepper ready");
}

void uartTest() {
  say("t: test_connection()...");
  const uint8_t conn = driver.test_connection();
  Serial.print("t: test_connection = ");
  Serial.print(conn);
  if (conn == 0) {
    Serial.println("  OK");
  } else if (conn == 1) {
    Serial.println("  FAIL (0xFFFFFFFF — no UART reply)");
  } else if (conn == 2) {
    Serial.println("  FAIL (0 — no UART reply)");
  } else {
    Serial.println("  unexpected");
  }
  Serial.flush();

  const uint8_t ver = driver.version();
  Serial.print("t: TMC version = 0x");
  Serial.println(ver, HEX);
  if (ver == 0x21) {
    say("t: UART OK (TMC2209)");
  } else {
    say("t: no valid version");
  }
}

void setupDriver() {
  TMC_SERIAL.setPollingMode(true);
  if (!TMC_SERIAL.setTX(TX_PIN) || !TMC_SERIAL.setRX(RX_PIN)) {
    say("Serial2 setTX/setRX failed — GP8/GP9 must use Serial2 (UART1)");
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
  driver.TPWMTHRS(0);
  driver.semin(0);
  driver.en_spreadCycle(false);
  driver.pdn_disable(true);
  driver.VACTUAL(0);
  driver.rms_current(set_current);
  driver.TCOOLTHRS(0);  // StallGuard off until UART + motion are confirmed

  uartTest();
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  say("TMC2209 Serial2 UART + motion (no soft UART)");

  // STEP/DIR first so a UART problem cannot block pulses.
  setupStepper();
  setupDriver();

  say("Setup complete. Motor should cycle. Type t to retest UART, h for help.");
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  const char c = (char)Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') {
    return;
  }

  switch (c) {
    case 'h':
    case '?':
      say("t  UART test");
      say("h  help");
      break;
    case 't':
      uartTest();
      break;
    default:
      Serial.print("unknown '");
      Serial.print(c);
      Serial.println("' — h for help");
      Serial.flush();
      break;
  }
}

void setup1() {}

void loop1() {
  if (!stepper) {
    delay(10);
    return;
  }

  stepper->moveTo(move_to_step);
  while (stepper->isRunning()) {
    delay(1);
  }
  delay(3000);

  stepper->moveTo(0);
  while (stepper->isRunning()) {
    delay(1);
  }
  delay(3000);
}

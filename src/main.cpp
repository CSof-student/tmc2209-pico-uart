#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

// ---------------- Pins ----------------
#define DIR_PIN     2
#define STEP_PIN    3
#define ENABLE_PIN  -1

#define TX_PIN      8
#define RX_PIN      9

// ---------------- TMC2209 ----------------
#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b00

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

// ---------------- Motion ----------------
int32_t move_to_step = 3200 * 5;
int32_t set_velocity = 3200;
int32_t set_accel = 3200 * 100;
int32_t set_current = 300;
uint16_t motor_microsteps = 16;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

void setup() {
  // USB Serial Monitor
  Serial.begin(115200);

  // Pico/Pico 2 hardware UART pin assignment
  Serial1.setTX(TX_PIN);
  Serial1.setRX(RX_PIN);
  Serial1.begin(115200);

  pinMode(ENABLE_PIN, OUTPUT);

  // ---------------- TMC2209 config ----------------
  driver.begin();

  driver.toff(4);
  driver.blank_time(24);

  driver.I_scale_analog(false);
  driver.internal_Rsense(false);

  driver.mstep_reg_select(true);
  driver.microsteps(motor_microsteps);

  // StealthChop / StallGuard-related config
  driver.TPWMTHRS(0);
  driver.semin(0);
  driver.en_spreadCycle(false);

  driver.pdn_disable(true);
  driver.VACTUAL(0);

  driver.rms_current(set_current);

  // StallGuard enabled over velocity range
  driver.TCOOLTHRS(0xFFFFF);

  // ---------------- FastAccelStepper ----------------
  engine.init();

  stepper = engine.stepperConnectToPin(STEP_PIN);

  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(ENABLE_PIN);
    stepper->setAutoEnable(true);

    stepper->setSpeedInHz(set_velocity);
    stepper->setAcceleration(set_accel);
    stepper->setCurrentPosition(0);
  }

  Serial.println("Setup complete");
}

// Core 0
void loop() {
  // Serial monitor / Wi-Fi / other logic can live here
}

// Core 1
void setup1() {
}

// This runs independently on the second Pico core.
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
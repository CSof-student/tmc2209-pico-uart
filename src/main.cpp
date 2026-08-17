#include <Arduino.h>
#include <TMCStepper.h>

// ---------- UART ----------
#define TMC_TX 8
#define TMC_RX 9

#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b00

// Pico hardware UART
#define TMC_SERIAL Serial1

TMC2209Stepper driver(&TMC_SERIAL, R_SENSE, DRIVER_ADDRESS);

void setup() {
  Serial.begin(115200);

  delay(2000);

  Serial.println();
  Serial.println("TMC2209 UART test");

  // Tell Serial1 which Pico pins to use
  Serial1.setTX(TMC_TX);
  Serial1.setRX(TMC_RX);

  // TMC2209 UART
  Serial1.begin(115200);

  delay(100);

  // Initialize driver
  driver.begin();

  // Required so UART-controlled settings are used
  driver.pdn_disable(true);

  // Select microstep resolution from UART registers
  driver.mstep_reg_select(true);

  Serial.println("Driver initialized");

  // -------------------------
  // Communication test
  // -------------------------

  uint32_t version = driver.version();

  Serial.print("TMC version = 0x");
  Serial.println(version, HEX);

  uint32_t drvStatus = driver.DRV_STATUS();

  Serial.print("DRV_STATUS = 0x");
  Serial.println(drvStatus, HEX);

  // -------------------------
  // Try writing some settings
  // -------------------------

  driver.rms_current(300);
  driver.microsteps(16);

  Serial.print("Microsteps = ");
  Serial.println(driver.microsteps());

  Serial.print("Current scale = ");
  Serial.println(driver.cs_actual());

  Serial.println();
  Serial.println("UART test complete");
}

void loop() {
}
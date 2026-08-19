/*
  UART test: Serial1 + TMCStepper only.

  No TmcSoftUart, no FastAccelStepper. If the motor sketch never prints
  "Setup complete", Serial1 / driver.version() is the usual hang.

  Wiring (single-wire UART pin on the TMC):
    Pico GP8 (TX) --[1k]--+---- TMC UART
    Pico GP9 (RX) --------+

  USB serial 115200. Commands:
    k  pin levels
    b  Serial1 loopback (TMC UART wires OFF, jumper GP8 to GP9)
    t  TMC UART test (test_connection + version)
*/

#include <Arduino.h>
#include <TMCStepper.h>

#define TX_PIN 8
#define RX_PIN 9

#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b00

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

void say(const char *msg) {
  Serial.println(msg);
  Serial.flush();
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  k  pin check (GP8 TX / GP9 RX levels)");
  Serial.println("  b  Serial1 loopback — disconnect TMC UART, jumper GP8-GP9");
  Serial.println("  t  TMC UART test (needs TMC wired, VM + VIO on)");
  Serial.println("  h  help");
  Serial.flush();
}

void pinCheck() {
  const int tx = digitalRead(TX_PIN);
  const int rx = digitalRead(RX_PIN);
  Serial.print("GP8 TX = ");
  Serial.println(tx ? "HIGH" : "LOW");
  Serial.print("GP9 RX = ");
  Serial.println(rx ? "HIGH" : "LOW");
  Serial.println("Idle UART should be HIGH on both (pull-up / Serial1 idle).");
  Serial.flush();
}

// Prove Pico UART1 (Serial1) without TMCStepper.
// Disconnect TMC from GP8/GP9 and jumper GP8 to GP9 first.
void serial1Loopback() {
  say("Serial1 loopback: writing 0xA5 (GP8 must be jumpered to GP9, TMC UART OFF)");

  while (Serial1.available()) {
    Serial1.read();
  }

  // Do not call Serial1.flush() — that has hung this Pico on the TMC bus.
  const size_t n = Serial1.write((uint8_t)0xA5);
  Serial.print("Serial1.write returned ");
  Serial.println(n);
  Serial.flush();

  int got = -1;
  const uint32_t start = millis();
  while (millis() - start < 200) {
    if (Serial1.available()) {
      got = Serial1.read();
      break;
    }
    delay(1);
  }

  if (got == 0xA5) {
    say("Serial1 loopback OK — hardware UART on GP8/GP9 works");
  } else {
    Serial.print("Serial1 loopback FAIL (got ");
    Serial.print(got);
    Serial.println("). If this line never printed after 'writing 0xA5', Serial1.write hung.");
    Serial.flush();
  }
}

void uartTest() {
  say("t: calling driver.begin() (UART writes)...");
  driver.begin();
  say("t: driver.begin() returned");

  say("t: test_connection() (reads DRV_STATUS)...");
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

  say("t: driver.version()...");
  const uint8_t ver = driver.version();
  Serial.print("t: TMC version = 0x");
  Serial.println(ver, HEX);
  if (ver == 0x21) {
    say("t: UART OK (TMC2209)");
  } else {
    say("t: no valid version — Serial1 may be fine (run b) but the TMC is not answering");
  }

  Serial.print("t: DRV_STATUS = 0x");
  Serial.println(driver.DRV_STATUS(), HEX);
  Serial.print("t: IFCNT = ");
  Serial.println(driver.IFCNT());
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  say("");
  say("TMC2209 UART test (Serial1, no soft UART, no motion)");

  Serial.print("before Serial1.setTX/RX  GP8=");
  Serial.print(TX_PIN);
  Serial.print(" GP9=");
  Serial.println(RX_PIN);
  Serial.flush();

  Serial1.setTX(TX_PIN);
  Serial1.setRX(RX_PIN);
  say("after setTX/RX, before Serial1.begin(115200)");

  Serial1.begin(115200);
  delay(50);
  say("Serial1.begin returned — hardware UART started");

  pinCheck();
  printHelp();
  say("Ready. Run b (loopback) first if you want to prove Serial1, then t.");
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
      printHelp();
      break;
    case 'k':
      pinCheck();
      break;
    case 'b':
      serial1Loopback();
      break;
    case 't':
      uartTest();
      break;
    default:
      Serial.print("unknown command '");
      Serial.print(c);
      Serial.println("' — h for help");
      Serial.flush();
      break;
  }
}

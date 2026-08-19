/*
  UART test: hardware UART + TMCStepper only.

  On Arduino-Pico, Serial1 is UART0 (GP0/GP1). GP8/GP9 are UART1, which is
  Serial2. Using Serial1.setTX(8) is an illegal pin and can hang the core.

  Wiring (single-wire UART pin on the TMC):
    Pico GP8 (TX) --[1k]--+---- TMC UART
    Pico GP9 (RX) --------+

  USB serial 115200. Commands:
    k  pin levels
    b  UART loopback (TMC UART wires OFF, jumper GP8 to GP9; 1k optional)
    t  TMC UART test (test_connection + version)
*/

#include <Arduino.h>
#include <TMCStepper.h>

#define TX_PIN 8
#define RX_PIN 9

#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b00

// UART1 — the UART that can actually use GP8/GP9
#define TMC_SERIAL Serial2

TMC2209Stepper driver(&TMC_SERIAL, R_SENSE, DRIVER_ADDRESS);

void say(const char *msg) {
  Serial.println(msg);
  Serial.flush();
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  k  pin check (GP8 TX / GP9 RX levels)");
  Serial.println("  b  Serial2 loopback — disconnect TMC UART, jumper GP8-GP9");
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
  Serial.println("Idle UART should be HIGH on both.");
  Serial.flush();
}

void drainRx() {
  int n = 0;
  while (n < 64 && TMC_SERIAL.available() > 0) {
    (void)TMC_SERIAL.read();
    n++;
  }
}

void serial2Loopback() {
  say("b: drain RX (max 64 bytes, not forever)");
  drainRx();
  say("b: waiting for TX ready (200 ms timeout)");

  const uint32_t readyStart = millis();
  while (TMC_SERIAL.availableForWrite() < 1) {
    if (millis() - readyStart > 200) {
      say("b: HANG AVOIDED — UART TX never writable. Pin mux / UART still stuck.");
      return;
    }
  }

  say("b: writing 0xA5 (GP8 jumpered to GP9, TMC UART OFF)");
  const size_t n = TMC_SERIAL.write((uint8_t)0xA5);
  Serial.print("b: write returned ");
  Serial.println(n);
  Serial.flush();

  int got = -1;
  const uint32_t start = millis();
  while (millis() - start < 200) {
    if (TMC_SERIAL.available()) {
      got = TMC_SERIAL.read();
      break;
    }
    delay(1);
  }

  if (got == 0xA5) {
    say("b: Serial2 loopback OK — UART1 on GP8/GP9 works");
  } else {
    Serial.print("b: Serial2 loopback FAIL (got ");
    Serial.print(got);
    Serial.println(")");
    Serial.println("   Direct jumper is fine for this test; 1k is also OK.");
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
    say("t: no valid version — UART port may be fine (run b) but the TMC is not answering");
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
  say("TMC2209 UART test (Serial2 = UART1 on GP8/GP9, no soft UART, no motion)");

  Serial.print("before Serial2.setTX/RX  GP8=");
  Serial.print(TX_PIN);
  Serial.print(" GP9=");
  Serial.println(RX_PIN);
  Serial.flush();

  // Polling avoids UART IRQ + FreeRTOS mutex deadlocks that look like a hang.
  TMC_SERIAL.setPollingMode(true);

  const bool txOk = TMC_SERIAL.setTX(TX_PIN);
  const bool rxOk = TMC_SERIAL.setRX(RX_PIN);
  Serial.print("setTX(8) = ");
  Serial.println(txOk ? "ok" : "FAIL");
  Serial.print("setRX(9) = ");
  Serial.println(rxOk ? "ok" : "FAIL");
  Serial.flush();
  if (!txOk || !rxOk) {
    say("Illegal UART pins — this sketch would hang if it used Serial1 (UART0).");
  }

  say("before Serial2.begin(115200)");
  TMC_SERIAL.begin(115200);
  delay(50);
  say("Serial2.begin returned — UART1 started");

  pinCheck();
  printHelp();
  say("Ready. Run b (loopback) first, then t with the TMC wired.");
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
      serial2Loopback();
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

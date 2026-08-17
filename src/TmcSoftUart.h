/*
  Half-duplex soft UART Stream for TMC2209 on Pico.
  Drop-in Stream for TMCStepper (Serial1 often hangs on single-wire Pico).

  Wiring:
    txPin --[1k]--+---- TMC UART
    rxPin --------+
                  +--[1k..2.2k]-- 3.3V  (many modules already have a pull-up)
*/

#pragma once

#include <Arduino.h>
#include <Stream.h>

class TmcSoftUart : public Stream {
 public:
  TmcSoftUart(uint8_t txPin, uint8_t rxPin) : _tx(txPin), _rx(rxPin) {}

  void begin(unsigned long baud) {
    _bitUs = (uint16_t)((1000000UL + baud / 2) / baud);
    if (_bitUs < 20) {
      _bitUs = 20;
    }
    pinMode(_tx, INPUT);
    pinMode(_rx, INPUT_PULLUP);
    _txHolding = false;
    _rxLen = 0;
    _rxIdx = 0;
  }

  void end() {
    releaseBus();
    pinMode(_rx, INPUT);
  }

  int available() override {
    releaseBus();
    return (int)(_rxLen - _rxIdx);
  }

  int peek() override {
    if (_rxIdx >= _rxLen) {
      return -1;
    }
    return _buf[_rxIdx];
  }

  int read() override {
    releaseBus();  // must not drive TX while TMC replies
    if (_rxIdx >= _rxLen) {
      uint8_t b = 0;
      if (!recvByte(b, 80)) {
        return -1;
      }
      return b;
    }
    return _buf[_rxIdx++];
  }

  void flush() override {
    // Intentionally empty — HardwareSerial.flush() hung on this bus
  }

  size_t write(uint8_t b) override {
    // New host TX: discard stale RX
    _rxLen = 0;
    _rxIdx = 0;
    claimBus();
    sendByte(b);  // leaves idle HIGH, still driving (until releaseBus)
    return 1;
  }

  using Print::write;

 private:
  uint8_t _tx;
  uint8_t _rx;
  uint16_t _bitUs = 104;
  bool _txHolding = false;
  uint8_t _buf[16];
  uint8_t _rxLen = 0;
  uint8_t _rxIdx = 0;

  static inline void waitUntil(uint32_t deadline) {
    while ((int32_t)(time_us_32() - deadline) < 0) {
    }
  }

  void claimBus() {
    if (!_txHolding) {
      pinMode(_tx, OUTPUT);
      digitalWrite(_tx, HIGH);
      _txHolding = true;
      delayMicroseconds(20);
    }
  }

  void releaseBus() {
    if (_txHolding) {
      pinMode(_tx, INPUT);  // high-Z; pull-up holds idle HIGH
      _txHolding = false;
    }
  }

  void sendByte(uint8_t b) {
    noInterrupts();
    digitalWrite(_tx, LOW);
    waitUntil(time_us_32() + _bitUs);
    for (uint8_t i = 0; i < 8; i++) {
      digitalWrite(_tx, (b & 0x01) ? HIGH : LOW);
      b >>= 1;
      waitUntil(time_us_32() + _bitUs);
    }
    digitalWrite(_tx, HIGH);  // stop / idle — keep driving until releaseBus()
    waitUntil(time_us_32() + _bitUs);
    interrupts();
  }

  bool recvByte(uint8_t &out, uint32_t timeoutMs) {
    const uint32_t start = millis();
    while (digitalRead(_rx) == LOW) {
      if (millis() - start > timeoutMs) {
        return false;
      }
    }
    while (digitalRead(_rx) == HIGH) {
      if (millis() - start > timeoutMs) {
        return false;
      }
    }
    noInterrupts();
    const uint32_t t0 = time_us_32();
    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; i++) {
      waitUntil(t0 + _bitUs + (_bitUs / 2) + (uint32_t)i * _bitUs);
      if (digitalRead(_rx)) {
        b |= (uint8_t)(1u << i);
      }
    }
    waitUntil(t0 + _bitUs * 10);
    interrupts();
    out = b;
    return true;
  }
};

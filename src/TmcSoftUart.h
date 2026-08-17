/*
  Half-duplex soft UART Stream for TMC2209 on Pico.
  Use with TMCStepper instead of Serial1 (Serial1 often hangs on single-wire).

  Wiring:
    txPin --[1k]--+---- TMC UART
    rxPin --------+
                  +--[1k..2.2k]-- 3.3V
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
    _rxLen = 0;
    _rxIdx = 0;
  }

  void end() {
    pinMode(_tx, INPUT);
    pinMode(_rx, INPUT);
  }

  int available() override { return (int)(_rxLen - _rxIdx); }

  int peek() override {
    if (_rxIdx >= _rxLen) {
      return -1;
    }
    return _buf[_rxIdx];
  }

  int read() override {
    if (_rxIdx >= _rxLen) {
      // Blocking read with timeout — TMCStepper expects this after a write
      if (!recvByte(_last, 50)) {
        return -1;
      }
      return _last;
    }
    return _buf[_rxIdx++];
  }

  void flush() override {
    // no-op: never block (HardwareSerial.flush hung on this bus)
  }

  size_t write(uint8_t b) override {
    // New TX datagram: drop any unread RX
    _rxLen = 0;
    _rxIdx = 0;
    sendByte(b);
    return 1;
  }

  using Print::write;

 private:
  uint8_t _tx;
  uint8_t _rx;
  uint16_t _bitUs = 104;  // ~9600
  uint8_t _buf[16];
  uint8_t _rxLen = 0;
  uint8_t _rxIdx = 0;
  uint8_t _last = 0;

  static inline void waitUntil(uint32_t deadline) {
    while ((int32_t)(time_us_32() - deadline) < 0) {
    }
  }

  void sendByte(uint8_t b) {
    noInterrupts();
    pinMode(_tx, OUTPUT);
    digitalWrite(_tx, LOW);
    waitUntil(time_us_32() + _bitUs);
    for (uint8_t i = 0; i < 8; i++) {
      digitalWrite(_tx, (b & 0x01) ? HIGH : LOW);
      b >>= 1;
      waitUntil(time_us_32() + _bitUs);
    }
    digitalWrite(_tx, HIGH);
    waitUntil(time_us_32() + _bitUs);
    pinMode(_tx, INPUT);  // release bus for TMC reply
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

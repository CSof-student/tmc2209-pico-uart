/*
  Half-duplex Stream for TMC2209 + TMCStepper on Pico single-wire UART.

  Why not SoftwareSerial/Serial1 alone?
    Those keep TX driven idle-HIGH. On this bus the TMC often cannot reply
    through the 1k. This Stream bitbangs TX, then releases TX (high-Z) so the
    driver can answer; RX listens on the junction. TMCStepper still owns all
    register read/write.

  TMCStepper writes 4- or 8-byte datagrams one byte at a time and never
  flush()es — we buffer and send atomically.

  Wiring:
    txPin --[1k]--+---- TMC UART
    rxPin --------+
                  (+ module pull-up is usually enough; external optional)
*/

#pragma once

#include <Arduino.h>
#include <Stream.h>
#include <string.h>

class TmcSoftUart : public Stream {
 public:
  TmcSoftUart(uint8_t txPin, uint8_t rxPin) : _tx(txPin), _rx(rxPin) {}

  void begin(unsigned long baud) {
    _bitUs = (uint16_t)((1000000UL + baud / 2) / baud);
    if (_bitUs < 20) {
      _bitUs = 20;
    }
    idle();
    _txCount = 0;
    _rxLen = 0;
    _rxIdx = 0;
    _awaitingReply = false;
  }

  void end() { idle(); }

  int available() override { return (int)(_rxLen - _rxIdx); }

  int peek() override {
    if (_rxIdx >= _rxLen) {
      return -1;
    }
    return _rxBuf[_rxIdx];
  }

  int read() override {
    if (_rxIdx >= _rxLen) {
      if (_awaitingReply) {
        _awaitingReply = false;
        recvReply();
      }
      if (_rxIdx >= _rxLen) {
        return -1;
      }
    }
    return _rxBuf[_rxIdx++];
  }

  void flush() override {}

  size_t write(uint8_t b) override {
    if (_txCount >= sizeof(_txBuf)) {
      _txCount = 0;
    }
    _txBuf[_txCount++] = b;
    if (_txCount == 4 || _txCount == 8) {
      const uint8_t n = _txCount;
      _txCount = 0;
      _rxLen = 0;
      _rxIdx = 0;
      sendBytes(_txBuf, n);
      _awaitingReply = (n == 4);
    }
    return 1;
  }

  using Print::write;

  // Debug: GP8 jumpered to GP9, TMC disconnected.
  bool loopbackTest(uint8_t sent = 0xA5) {
    idle();
    delay(2);
    noInterrupts();
    writeByteUnlocked(sent);
    pinMode(_tx, INPUT);
    interrupts();

    uint8_t got = 0;
    if (!readByte(got, 50, true)) {
      return false;
    }
    return got == sent;
  }

  // Debug: send IOIN read, dump up to maxBytes of raw RX (before TMCStepper).
  // Returns number of bytes captured into out[].
  uint8_t rawIoinProbe(uint8_t addr, uint8_t *out, uint8_t maxBytes) {
    idle();
    delay(2);
    uint8_t req[4] = {0x05, (uint8_t)(addr & 0x03), 0x06, 0};
    req[3] = crc(req, 3);
    sendBytes(req, 4);

    uint8_t n = 0;
    const uint32_t t0 = millis();
    while (n < maxBytes && (millis() - t0) < 200) {
      uint8_t b = 0;
      if (!readByte(b, 40, n == 0)) {
        if (n > 0) {
          break;
        }
        continue;
      }
      out[n++] = b;
    }
    return n;
  }

 private:
  uint8_t _tx;
  uint8_t _rx;
  uint16_t _bitUs = 104;
  uint8_t _txBuf[8];
  uint8_t _txCount = 0;
  uint8_t _rxBuf[16];
  uint8_t _rxLen = 0;
  uint8_t _rxIdx = 0;
  bool _awaitingReply = false;

  static inline void waitUntil(uint32_t deadline) {
    while ((int32_t)(time_us_32() - deadline) < 0) {
    }
  }

  static uint8_t crc(const uint8_t *data, uint8_t len) {
    uint8_t c = 0;
    for (uint8_t i = 0; i < len; i++) {
      uint8_t current = data[i];
      for (uint8_t j = 0; j < 8; j++) {
        if ((c >> 7) ^ (current & 0x01)) {
          c = (c << 1) ^ 0x07;
        } else {
          c <<= 1;
        }
        current >>= 1;
      }
    }
    return c;
  }

  void idle() {
    pinMode(_tx, INPUT);
    pinMode(_rx, INPUT_PULLUP);
  }

  void writeByteUnlocked(uint8_t b) {
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
  }

  void sendBytes(const uint8_t *data, uint8_t len) {
    noInterrupts();
    for (uint8_t i = 0; i < len; i++) {
      writeByteUnlocked(data[i]);
    }
    pinMode(_tx, INPUT);  // release bus for TMC reply
    interrupts();

    const uint32_t t0 = millis();
    while (digitalRead(_rx) == LOW) {
      if (millis() - t0 > 20) {
        break;
      }
    }
    delayMicroseconds(100);
  }

  bool readByteUnlocked(uint8_t &out, bool waitFalling) {
    if (waitFalling) {
      const uint32_t giveUp = time_us_32() + (uint32_t)_bitUs * 30;
      if (digitalRead(_rx) == HIGH) {
        while (digitalRead(_rx) == HIGH) {
          if ((int32_t)(time_us_32() - giveUp) >= 0) {
            return false;
          }
        }
      }
    }
    const uint32_t t0 = time_us_32();
    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; i++) {
      waitUntil(t0 + _bitUs + (_bitUs / 2) + (uint32_t)i * _bitUs);
      if (digitalRead(_rx)) {
        b |= (uint8_t)(1u << i);
      }
    }
    waitUntil(t0 + _bitUs * 10);
    out = b;
    return true;
  }

  bool readByte(uint8_t &out, uint32_t timeoutMs, bool requireIdleHigh) {
    const uint32_t start = millis();
    if (requireIdleHigh) {
      while (digitalRead(_rx) == LOW) {
        if (millis() - start > timeoutMs) {
          return false;
        }
      }
    }
    if (digitalRead(_rx) == HIGH) {
      while (digitalRead(_rx) == HIGH) {
        if (millis() - start > timeoutMs) {
          return false;
        }
      }
    }
    noInterrupts();
    const bool ok = readByteUnlocked(out, false);
    interrupts();
    return ok;
  }

  void recvReply() {
    _rxLen = 0;
    _rxIdx = 0;
    const uint32_t huntStart = millis();
    while (millis() - huntStart < 200) {
      uint8_t b0 = 0;
      if (!readByte(b0, 40, true) || b0 != 0x05) {
        continue;
      }
      uint8_t frame[8];
      frame[0] = 0x05;
      uint8_t n = 1;
      noInterrupts();
      for (; n < 8; n++) {
        if (!readByteUnlocked(frame[n], true)) {
          break;
        }
      }
      interrupts();
      if (n < 8 || frame[1] != 0xFF) {
        continue;
      }
      memcpy(_rxBuf, frame, 8);
      _rxLen = 8;
      _rxIdx = 0;
      return;
    }
  }
};

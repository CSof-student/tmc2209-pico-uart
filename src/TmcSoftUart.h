/*
  Half-duplex Stream for TMC2209 + TMCStepper on Pico single-wire UART.

  TX: push-pull for the datagram, then release (high-Z) so the TMC can reply.
  RX: bitbang with start-bit resync between bytes (fixed bit-count wait was
  drifting into the next start bit → 05 FF then garbage / bad CRC).

  Wiring:
    txPin --[1k]--+---- TMC UART
    rxPin --------+
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

  // Hunt 05 FF, grab 8 bytes atomically, report CRC. Copies frame to out[8].
  // Returns true if header+CRC OK.
  bool rawIoinProbe(uint8_t addr, uint8_t out[8], bool &crcOk) {
    crcOk = false;
    idle();
    delay(2);
    uint8_t req[4] = {0x05, (uint8_t)(addr & 0x03), 0x06, 0};
    req[3] = crc(req, 3);
    sendBytes(req, 4);

    uint8_t frame[8];
    if (!huntReplyFrame(frame)) {
      memset(out, 0, 8);
      return false;
    }
    memcpy(out, frame, 8);
    crcOk = (crc(frame, 7) == frame[7]);
    return true;
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
    pinMode(_tx, INPUT);
    interrupts();

    const uint32_t t0 = millis();
    while (digitalRead(_rx) == LOW) {
      if (millis() - t0 > 20) {
        break;
      }
    }
    delayMicroseconds(100);
  }

  // waitFalling: resync on stop HIGH → start LOW (avoids late t0 mid-start-bit).
  // edgeAlreadySeen: first byte after detecting start outside.
  bool readByteUnlocked(uint8_t &out, bool waitFalling) {
    if (waitFalling) {
      const uint32_t giveUp = time_us_32() + (uint32_t)_bitUs * 40;
      // Complete prior stop bit (must see HIGH) before next start
      while (digitalRead(_rx) == LOW) {
        if ((int32_t)(time_us_32() - giveUp) >= 0) {
          return false;
        }
      }
      while (digitalRead(_rx) == HIGH) {
        if ((int32_t)(time_us_32() - giveUp) >= 0) {
          return false;
        }
      }
    }

    // t0 = falling edge (start bit) as tightly as possible
    const uint32_t t0 = time_us_32();
    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; i++) {
      waitUntil(t0 + _bitUs + (_bitUs / 2) + (uint32_t)i * _bitUs);
      if (digitalRead(_rx)) {
        b |= (uint8_t)(1u << i);
      }
    }
    // Stop bit middle only — do NOT run to bit 10 (overshoots into next start)
    waitUntil(t0 + _bitUs * 9 + (_bitUs / 2));
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

    // Wait for start edge, then lock IRQs immediately
    for (;;) {
      if (millis() - start > timeoutMs) {
        return false;
      }
      if (digitalRead(_rx) == HIGH) {
        continue;
      }
      noInterrupts();
      if (digitalRead(_rx) == LOW) {
        const bool ok = readByteUnlocked(out, false);
        interrupts();
        return ok;
      }
      interrupts();
    }
  }

  bool huntReplyFrame(uint8_t frame[8]) {
    const uint32_t huntStart = millis();
    while (millis() - huntStart < 200) {
      uint8_t b0 = 0;
      if (!readByte(b0, 40, true) || b0 != 0x05) {
        continue;
      }

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
      return true;
    }
    return false;
  }

  void recvReply() {
    _rxLen = 0;
    _rxIdx = 0;
    uint8_t frame[8];
    if (!huntReplyFrame(frame)) {
      return;
    }
    // Only accept CRC-valid frames (TMCStepper also checks; avoid feeding junk)
    if (crc(frame, 7) != frame[7]) {
      return;
    }
    memcpy(_rxBuf, frame, 8);
    _rxLen = 8;
    _rxIdx = 0;
  }
};

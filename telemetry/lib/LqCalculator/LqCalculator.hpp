/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

template <uint8_t N>
class LqCalculator {
 public:
  /* Set the bit for the current period to true and update the running LQ */
  void add() {
    if (currentIsSet()) {
      return;
    }
    LQArray[index] |= LQmask;
    LQ += 1;
  }

  /* Start a new period */
  void inc() {
    // Increment the counter by shifting one bit higher
    // If we've shifted out all the bits, move to next idx
    LQmask = LQmask << 1U;
    if (LQmask == 0) {
      LQmask = (1U << 0U);
      index += 1;
    }

    // At idx N / 32 and bit N % 32, wrap back to idx=0, bit=0
    if ((index == (N / 32)) && static_cast<bool>(LQmask & (1U << (N % 32U)))) {
      index = 0;
      LQmask = (1U << 0U);
    }

    if ((LQArray[index] & LQmask) != 0) {
      LQArray[index] &= ~LQmask;
      LQ -= 1;
    }

    if (count < N) {
      ++count;
    }
  }

  /* Return the current running total of bits set, in percent */
  uint8_t getLQ() const { return static_cast<uint32_t>(LQ) * 100U / count; }

  /* Return the current running total of bits set, up to N */
  uint8_t getLQRaw() const { return LQ; }

  /* Return the number of periods recorded so far, up to N */
  uint8_t getCount() const { return count; }

  /* Return N, the size of the LQ history */
  uint8_t getSize() const { return N; }

  /* Initialize and zero the history */
  void reset() {
    // count is intentonally not zeroed here to start LQ counting up from 0
    // after a failsafe, instead of down from 100
    LQ = 0;
    index = 0;
    LQmask = (1U << 0U);
    for (uint32_t i = 0; i < (sizeof(LQArray) / sizeof(LQArray[0])); i++) {
      LQArray[i] = 0;
    }
  }

  /*  Return true if the current period was add()ed */
  bool currentIsSet() const { return LQArray[index] & LQmask; }

 private:
  uint8_t LQ{0};
  uint8_t index{0};  // current position in LQArray
  // count is reset here only once on construction to start LQ counting
  // at 100% on first connect, but 0 out of N after a failsafe
  uint8_t count{1};
  uint32_t LQmask{1U << 0U};
  uint32_t LQArray[(N + 31) / 32]{};
};

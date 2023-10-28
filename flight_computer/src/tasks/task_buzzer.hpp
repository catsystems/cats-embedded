/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "drivers/buzzer.hpp"
#include "task.hpp"

namespace task {

class Buzzer final : public Task<Buzzer, 256> {
 public:
  /// Scoped enum with the beep codes
  enum class BeepCode : uint8_t {
    kNone = 0,
    kBootup,
    kReady,
    kChangedReady,
    kTesting,
    kTestingArmed,
  };

  /** Constructor
   *
   * @param buzzer Reference to the buzzer @injected
   */
  explicit Buzzer(driver::Buzzer& buzzer) : m_buzzer{buzzer}, m_buzzer_event_id{osEventFlagsNew(nullptr)} {}

  /** Beep the buzzer
   *
   * @param code the beep code
   */
  void Beep(BeepCode code) const;

 private:
  [[noreturn]] void Run() noexcept override;

  /// Scoped enum with the beep durations
  enum class Duration {
    kShortBeep = 100U,
    kLongBeep = 250U,
    kPause = 200U,
  };

  /// Lookup table for the notes
  static constexpr uint32_t s_frequency_lookup[8] = {
      2349U,  // D A
      2489U,  // D# B
      2637U,  // E C
      2793U,  // F D
      2959U,  // F# E
      3135U,  // G F
      1200U,  // Error G
      2217U,  // C#  H
  };

  /// Lookup table with the beep codes
  static constexpr char s_status_codes[6][5] = {
      " ",
      "caef",  // bootup
      "aa",    // ready
      "ace",   // moving -> ready
      "G",     // Testing - Not Armed
      "a",     // Testing - Armed
  };

  /// Lookup table with the length of the beep codes
  static constexpr uint8_t s_status_code_length[6] = {0U, 4U, 2U, 3U, 1U, 1U};

  /// Reference to the buzzer
  driver::Buzzer& m_buzzer;
  /// Freertos event flag for requesting new beeps
  osEventFlagsId_t m_buzzer_event_id;
};

}  // namespace task

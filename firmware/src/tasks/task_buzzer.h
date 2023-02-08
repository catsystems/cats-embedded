/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/buzzer.h"
#include "task.h"

namespace task {

class Buzzer final : public Task<Buzzer, 256> {
 public:
  /// Scoped enum with the beep codes
  enum class BeepCode : uint8_t {
    kNone = 0,
    kBootup,
    kReady,
    kChangedMoving,
    kChangedReady,
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
    kShortPause = 200U,
    kLongPause = 1000U,
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
  static constexpr char s_status_codes[5][5] = {
      " ",
      "caef",  // bootup
      "aa",    // ready
      "Eca",   // ready -> moving
      "ace",   // moving -> ready
  };

  /// Lookup table with the length of the beep codes
  static constexpr uint8_t s_status_code_length[5] = {0U, 4U, 2U, 3U, 3U};

  /// Reference to the buzzer
  driver::Buzzer& m_buzzer;
  /// Freertos event flag for requesting new beeps
  osEventFlagsId_t m_buzzer_event_id;
};

}  // namespace task

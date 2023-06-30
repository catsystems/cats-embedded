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

#include "task.hpp"
#include "task_buzzer.hpp"

namespace task {

class HealthMonitor final : public Task<HealthMonitor, 256> {
 public:
  explicit HealthMonitor(const Buzzer& task_buzzer) : m_task_buzzer(task_buzzer) { DeterminePyroCheck(); }

 private:
  [[noreturn]] void Run() noexcept override;

  const Buzzer& m_task_buzzer;

  /**
   * Determines if pyros should be checked for continuity by checking if there is an action that triggers them.
   */
  void DeterminePyroCheck();

  bool m_check_pyro_1{false};
  bool m_check_pyro_2{false};
};

}  // namespace task

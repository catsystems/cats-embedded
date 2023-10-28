/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

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

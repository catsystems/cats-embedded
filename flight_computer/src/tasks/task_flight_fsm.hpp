/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "task.hpp"
#include "task_preprocessing.hpp"
#include "task_state_est.hpp"

namespace task {

class FlightFsm final : public Task<FlightFsm, 512> {
 public:
  explicit FlightFsm(const Preprocessing& task_preprocessing, StateEstimation& task_state_estimation)
      : m_task_preprocessing{task_preprocessing}, m_task_state_estimation{task_state_estimation} {}

 private:
  const Preprocessing& m_task_preprocessing;
  StateEstimation& m_task_state_estimation;
  [[noreturn]] void Run() noexcept override;
};

}  // namespace task

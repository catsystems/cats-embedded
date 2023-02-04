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

#include "task.h"

#include "control/kalman_filter.h"
#include "control/orientation_filter.h"
#include "tasks/task_preprocessing.h"
#include "util/error_handler.h"
#include "util/log.h"
#include "util/types.h"

namespace task {

class StateEstimation final : public Task<StateEstimation, 512> {
 public:
  [[nodiscard]] estimation_output_t GetEstimationOutput() const noexcept;

 private:
  [[noreturn]] void Run() noexcept override;

  void GetEstimationInputData();

  /* Initialize State Estimation */
  kalman_filter_t m_filter = {.t_sampl = 1.0f / (float)(CONTROL_SAMPLING_FREQ)};
  orientation_filter_t m_orientation_filter = {};
};

}  // namespace task

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

#include <atomic>
#include "task.hpp"

#include "control/kalman_filter.hpp"
#include "control/orientation_filter.hpp"
#include "tasks/task_preprocessing.hpp"
#include "util/error_handler.hpp"
#include "util/log.h"
#include "util/types.hpp"

#include "task_preprocessing.hpp"

namespace task {

class StateEstimation;
extern StateEstimation* global_state_estimation;

class StateEstimation final : public Task<StateEstimation, 512> {
 public:
  explicit StateEstimation(const Preprocessing& task_preprocessing)
      : m_task_preprocessing{task_preprocessing},
        m_filter{.t_sampl = 1.0F / static_cast<float>(CONTROL_SAMPLING_FREQ)},
        m_orientation_filter{} {
    global_state_estimation = this;
  }
  [[nodiscard]] estimation_output_t GetEstimationOutput() const noexcept;

 private:
  [[noreturn]] void Run() noexcept override;

  void GetEstimationInputData();

  const Preprocessing& m_task_preprocessing;

  /* Initialize State Estimation */
  kalman_filter_t m_filter;
  orientation_filter_t m_orientation_filter;
};

}  // namespace task

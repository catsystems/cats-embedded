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
#include "util/types.hpp"

void start_simulation(char *args);

namespace task {

class Simulator final : public Task<Simulator, 512> {
 public:
  explicit Simulator(cats_sim_config_t sim_config) : m_sim_config{sim_config} {};

 private:
  [[noreturn]] void Run() noexcept override;

  void SetCoefficients(int32_t sim_decision);
  void ComputeSimValues(float32_t time);

#define POLYNOM_SIZE 16

  struct sim_coeff_t {
    float32_t pressure_coeff[POLYNOM_SIZE] = {};
    float32_t acceleration_coeff_thrusting[POLYNOM_SIZE] = {};
    float32_t acceleration_coeff_coasting[POLYNOM_SIZE] = {};
    float32_t switch_time = 0.0F;
    float32_t end_time = 0.0F;
  };

  const float32_t m_idle_time = 20.0F;  // [s]
  const float32_t m_reset_time = 3.0F;  // [s]
  const float32_t m_acc_noise = 0.02F;  // [g]
  const float32_t m_acc_factor = 1024.0F;
  const float32_t m_press_noise = 50.0F;  // [Pa]

  float32_t m_current_acc = 0.0F;    // [g]
  float32_t m_current_press = 0.0F;  // [Pa]

  cats_sim_config_t m_sim_config;
  sim_coeff_t m_sim_coeff = {};
};

}  // namespace task

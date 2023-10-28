/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

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

  static constexpr uint8_t POLYNOM_SIZE{16};

  struct sim_coeff_t {
    float64_t pressure_coeff[POLYNOM_SIZE] = {};
    float64_t acceleration_coeff_thrusting[POLYNOM_SIZE] = {};
    float64_t acceleration_coeff_coasting[POLYNOM_SIZE] = {};
    float32_t switch_time = 0.0F;
    float32_t acc_end_time = 0.0F;
    float32_t end_time = 0.0F;
  };

  const float32_t m_idle_time = 20.0F;  // [s]
  const float32_t m_reset_time = 3.0F;  // [s]
  const float32_t m_acc_noise = 0.0F;   // [g]
  const float32_t m_acc_factor = 1024.0F;
  const float32_t m_press_noise = 50.0F;  // [Pa]

  float64_t m_current_acc = 0.0;    // [g]
  float64_t m_current_press = 0.0;  // [Pa]

  cats_sim_config_t m_sim_config;
  sim_coeff_t m_sim_coeff = {};
};

}  // namespace task

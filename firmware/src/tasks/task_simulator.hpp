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

  cats_sim_config_t m_sim_config;
};

}  // namespace task

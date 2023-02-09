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

#include <array>
#include "cmsis_os.h"
#include "config/globals.h"

#include "task.h"

extern const uint32_t EVENT_QUEUE_SIZE;

namespace task {

class Peripherals final : public Task<Peripherals, 256> {
  [[noreturn]] void Run() noexcept override;

 private:
  uint32_t m_event_tracking = 0U;
};

}  // namespace task

osStatus_t trigger_event(cats_event_e ev);

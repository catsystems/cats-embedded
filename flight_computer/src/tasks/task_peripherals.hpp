/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "cmsis_os.h"
#include "config/globals.hpp"

#include "task.hpp"

extern const uint32_t EVENT_QUEUE_SIZE;

namespace task {

class Peripherals final : public Task<Peripherals, 256> {
  [[noreturn]] void Run() noexcept override;
};

}  // namespace task

osStatus_t trigger_event(cats_event_e ev, bool event_unique = true);

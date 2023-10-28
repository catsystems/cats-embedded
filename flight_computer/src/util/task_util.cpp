/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "util/task_util.hpp"

#include "target.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
volatile bool rtos_started = false;

void sysDelay(uint32_t delay) {
  if (rtos_started) {
    osDelay(delay);
  } else {
    HAL_Delay(delay);
  }
}

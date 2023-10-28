/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

#include "FreeRTOSConfig.h"
#include "cmsis_os.h"

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern volatile bool rtos_started;

void sysDelay(uint32_t delay);

constexpr uint32_t sysGetTickFreq() { return configTICK_RATE_HZ; }

/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "arm_math.h"
#include "types.hpp"

enum battery_level_e { BATTERY_CRIT = 1, BATTERY_LOW, BATTERY_OK };

void battery_monitor_init(battery_type_e type);

float32_t battery_voltage();
uint16_t battery_voltage_short();
uint8_t battery_voltage_byte();

battery_level_e battery_level();

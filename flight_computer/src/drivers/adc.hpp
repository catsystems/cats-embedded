/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "target.hpp"

enum adc_channels_e { ADC_BATTERY = 0, ADC_PYRO2, ADC_PYRO1 };

void adc_init();
uint32_t adc_get(adc_channels_e channel);

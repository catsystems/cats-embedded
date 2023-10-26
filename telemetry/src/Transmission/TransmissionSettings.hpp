/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "Telemetry_reg.h"

#include <Sx1280Driver.hpp>
#include <cstdint>

class TransmissionSettings {
 public:
  transmission_direction_e transmissionDirection = TX;
  transmission_mode_e transmissionMode = UNIDIRECTIONAL;
  int8_t paGain = 34;      // dB, default +34dB SKY65383-11
  int8_t powerLevel = 20;  // dBm, default +20dBm = 100mW
  uint32_t linkPhraseCrC = 0;
  bool transmissionEnabled = false;

  /* Predefined transmission modes */
  uint32_t modeIndex = 1;
  modulation_settings_s modulationConfig[2] = {
      {SX1280_LORA_BW_0800, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_7, 100000, 12, 17},
      {SX1280_LORA_BW_0800, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_7, 50000, 12, 17}};
};

/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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

#include "SX1280Driver/SX1280_Regs.h"
#include "telemetry_reg.h"
#include <cstdint>

class TransmissionSettings {
public:
  transmission_direction_e transmissionDirection = TX;
  transmission_mode_e transmissionMode = UNIDIRECTIONAL;
  int8_t paGain = 34;     // dB, default +34dB SKY65383-11
  int8_t powerLevel = 20; // dBm, default +20dBm = 100mW
  uint32_t linkPhraseCrC = 0;
  bool transmissionEnabled = false;

  /* Predefined transmission modes */
  uint32_t modeIndex = 1;
  modulation_settings_s modulationConfig[2] = {
      {SX1280_LORA_BW_0800, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_7, 100000, 12,
       17},
      {SX1280_LORA_BW_0800, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_7, 50000, 12,
       17}};
};

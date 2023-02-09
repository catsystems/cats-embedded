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

#include "util/battery.h"
#include "drivers/adc.h"

#include <cmath>

#define ADC_LINFIT_A 0.00826849f
#define ADC_LINFIT_B 0.169868f

#define BATTERY_VOLTAGE_HYSTERESIS 0.2f

static uint32_t cell_count = 1;

/* Supported batteries and their voltages
 * --------------------------------
 * BATTERY        MIN   LOW   MAX
 * Li-Ion         3.2   3.4   4.3
 * Li-Po          3.4   3.6   4.3
 * Alkaline (9V)  7.0   7.5   9.5
 * --------------------------------
 */
// clang-format off
const float32_t voltage_lookup[3][3] = {{3.2F, 3.4F, 4.3F},
                                        {3.4F, 3.6F, 4.3F},
                                        {7.0F, 7.5F, 9.5F}};
// clang-format on
enum battery_level_index_e { BAT_IDX_CRIT = 0, BAT_IDX_LOW, BAT_IDX_OK };

static battery_type_e battery_type;

/* Automatically check how many cells are connected */
void battery_monitor_init(battery_type_e type) {
  battery_type = type;
  if (battery_type == ALKALINE) {
    return;
  }
  uint32_t i = 1;
  float32_t voltage = battery_voltage();
  while ((static_cast<float32_t>(i) * voltage_lookup[battery_type][BAT_IDX_OK]) < voltage) {
    i++;
  }
  cell_count = i;
}

/* Returns battery voltage */
float32_t battery_voltage() {
  // https://www.wolframalpha.com/input?i=linear+fit+%28707%2C6%29%2C%281068%2C9%29%2C%281430%2C12%29%2C%281792%2C15%29%2C%282154%2C18%29%2C%282520%2C21%29%2C%282884%2C24%29
  return static_cast<float32_t>(adc_get(ADC_BATTERY)) * ADC_LINFIT_A + ADC_LINFIT_B;
}

/* Returns battery voltage as uint16, used for recording */
uint16_t battery_voltage_short() { return static_cast<uint16_t>(round(battery_voltage() * 1000)); }

float32_t battery_cell_voltage() { return battery_voltage() / static_cast<float32_t>(cell_count); }

battery_level_e battery_level() {
  static battery_level_e level = BATTERY_OK;
  float32_t voltage = battery_cell_voltage();

  /* Battery level can only go back up when voltage + hysteresis voltage is reached */
  switch (level) {
    case BATTERY_CRIT:
      if (voltage > (voltage_lookup[battery_type][BAT_IDX_CRIT] + BATTERY_VOLTAGE_HYSTERESIS)) {
        level = BATTERY_LOW;
      }
      break;
    case BATTERY_LOW:
      if (voltage <= voltage_lookup[battery_type][BAT_IDX_CRIT]) {
        level = BATTERY_CRIT;
      } else if (voltage > (voltage_lookup[battery_type][BAT_IDX_LOW] + BATTERY_VOLTAGE_HYSTERESIS)) {
        level = BATTERY_OK;
      }
      break;
    case BATTERY_OK:
      if (voltage <= voltage_lookup[battery_type][BAT_IDX_LOW]) {
        level = BATTERY_LOW;
      }
      break;
    default:
      break;
  }

  return level;
}

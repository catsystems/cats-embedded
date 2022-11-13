/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
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

#include "drivers/adc.h"
#include "util/battery.h"

#define ADC_LINFIT_A 0.00818474f
#define ADC_LINFIT_B 0.476469f

#define BATTERY_VOLTAGE_HYSTERESIS 0.2f

typedef enum { LI_ION = 0, LI_PO, ALKALINE } battery_type_e;

// TODO The battery type needs to be defined in the config
const battery_type_e battery_type = LI_PO;

static uint32_t cell_count = 1;

/* Supported batteries and their voltages
 * --------------------------------
 * BATTERY        MIN   LOW   MAX
 * Li-Ion         3.2   3.4   4.3
 * Li-Po          3.4   3.6   4.3
 * Alkaline (9V)  7.0   7.5   9.5
 * --------------------------------
 */
const float voltage_lookup[3][3] = {{3.2f, 3.4f, 4.3f}, {3.4f, 3.6f, 4.3f}, {7.0f, 7.5f, 9.5f}};
typedef enum { BAT_IDX_CRIT = 0, BAT_IDX_LOW, BAT_IDX_OK } battery_level_index_e;

// Automatically check how many cells are connected
void battery_monitor_init() {
  if (battery_type == ALKALINE) return;
  uint32_t i = 1;
  float voltage = battery_voltage();
  while (((float)i * voltage_lookup[battery_type][BAT_IDX_OK]) < voltage) {
    i++;
  }
  cell_count = i;
}

// Returns battery voltage
float battery_voltage() {
  // https://www.wolframalpha.com/input/?i=linear+fit+%28675%2C6%29%2C%28919%2C8%29%2C%281408%2C12%29%2C%282141%2C18%29
  return (float)adc_get(ADC_BATTERY) * ADC_LINFIT_A + ADC_LINFIT_B;
}

float battery_cell_voltage() { return ((float)adc_get(ADC_BATTERY) * ADC_LINFIT_A + ADC_LINFIT_B) / (float)cell_count; }

battery_level_e battery_level() {
  float voltage;
  static battery_level_e level = BATTERY_OK;
  voltage = battery_cell_voltage();

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

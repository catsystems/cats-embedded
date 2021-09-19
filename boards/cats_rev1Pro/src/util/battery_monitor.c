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

#define ADC_BATTERY_A 0.00500571f
#define ADC_BATTERY_B 0.222025f

// TODO The battery type needs to be defined in the config

typedef enum { LIION = 0, LIPO, ALKALINE } battery_type_e;

battery_type_e battery_type = LIPO;

static uint32_t cell_count = 1;

/*Supported batteries
 * ------------------------------------
 * BATTERY		MIN		LOW		MAX
 * Li-Ion		3.2		3.4		4.3
 * Li-Po		3.4		3.6		4.3
 * Alkaline (9V)	7.0		7.5		9.5
 * ------------------------------------
 */
const float voltage_lookup[3][3] = {{3.3f, 3.5f, 4.3f}, {3.4f, 3.6f, 4.3f}, {7.0f, 7.5f, 9.5f}};

// Automatically check how many cells are connected
void battery_monitor_init() {
  if (battery_type == ALKALINE) return;
  uint32_t i = 1;
  float voltage = battery_voltage();
  while (((float)i * voltage_lookup[battery_type][2]) < voltage) {
    i++;
  }
  cell_count = i;
}

// Returns battery voltage
float battery_voltage() {
  // https://www.wolframalpha.com/input/?i=linear+fit+%281215%2C6.35%29%2C%281840%2C9.35%29%2C%282342%2C11.96%29%2C%282820%2C14.36%29
  // TODO remove the +0.65 when there is a final solution (ideal diode, diode etc.)
  return (float)adc_get(ADC_BATTERY) * ADC_BATTERY_A + ADC_BATTERY_B + 0.65f;
}

float battery_cell_voltage() {
  return ((float)adc_get(ADC_BATTERY) * ADC_BATTERY_A + ADC_BATTERY_B + 0.65f) / (float)cell_count;
}

battery_level_e battery_level() {
  float voltage;
  voltage = battery_cell_voltage();

  if (voltage <= voltage_lookup[battery_type][0]) return BATTERY_CRIT;
  if (voltage <= voltage_lookup[battery_type][1]) return BATTERY_LOW;
  return BATTERY_OK;
}

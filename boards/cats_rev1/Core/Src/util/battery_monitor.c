/*
 * battery_monitor.c
 *
 *  Created on: Jan 7, 2021
 *      Author: Luca
 */

#include "drivers/adc.h"
#include "util/battery.h"

#define ADC_BATTERY_A 0.00500571f
#define ADC_BATTERY_B 0.222025f

// TODO The battery type needs to be defined in the config

typedef enum { LIION = 0, LIPO, ALKALINE } battery_type_e;

battery_type_e battery_type = LIPO;

/*Supported batteries
 * ------------------------------------
 * BATTERY		MIN		LOW		MAX
 * Li-Ion		3.2		3.4		4.3
 * Li-Po		3.4		3.6		4.3
 * Alkaline (9V)	7.0		7.5		9.5
 * ------------------------------------
 */

static uint32_t cell_count = 1;

const float voltage_lookup[3][3] = {
    {3.3f, 3.5f, 4.3f}, {3.4f, 3.6f, 4.3f}, {7.0f, 7.5f, 9.5f}};

// Automatically check how many cells are connected
void battery_monitor_init() {
  if (battery_type == ALKALINE) return;
  int i = 1;
  float voltage = battery_voltage();
  while ((i * voltage_lookup[battery_type][2]) < voltage) {
    i++;
  }
  cell_count = i;
}

// Returns battery voltage
float battery_voltage() {
  // https://www.wolframalpha.com/input/?i=linear+fit+%281215%2C6.35%29%2C%281840%2C9.35%29%2C%282342%2C11.96%29%2C%282820%2C14.36%29
  // TODO remove the +0.65 when there is a final solution (ideal diode, diode
  // etc.)
  return (float)adc_get(ADC_BATTERY) * ADC_BATTERY_A + ADC_BATTERY_B + 0.65f;
}

float battery_cell_voltage() {
  return ((float)adc_get(ADC_BATTERY) * ADC_BATTERY_A + ADC_BATTERY_B + 0.65f) /
         cell_count;
}

battery_level_e battery_level() {
  float voltage;
  voltage = battery_cell_voltage();

  if (voltage <= voltage_lookup[battery_type][0]) return BATTERY_CRIT;
  if (voltage <= voltage_lookup[battery_type][1]) return BATTERY_LOW;
  return BATTERY_OK;
}

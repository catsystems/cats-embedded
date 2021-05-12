/*
 * battery.h
 *
 *  Created on: Jan 7, 2021
 *      Author: Luca
 */

#pragma once

typedef enum { BATTERY_OK = 0, BATTERY_LOW, BATTERY_CRIT } battery_level_e;

void battery_monitor_init();
float battery_voltage();
battery_level_e battery_level();

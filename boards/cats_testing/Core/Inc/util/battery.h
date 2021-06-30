/*
 * battery.h
 *
 *  Created on: Jan 7, 2021
 *      Author: Luca
 */

#ifndef INC_UTIL_BATTERY_H_
#define INC_UTIL_BATTERY_H_

typedef enum { BATTERY_OK = 0, BATTERY_LOW, BATTERY_CRIT } battery_level_e;

void battery_monitor_init();
float battery_voltage();
battery_level_e battery_level();

#endif /* INC_UTIL_BATTERY_H_ */

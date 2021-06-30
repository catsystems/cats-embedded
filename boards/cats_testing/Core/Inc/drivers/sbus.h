/*
 * sbus.h
 *
 *  Created on: 9 Apr 2021
 *      Author: Luca
 */

#ifndef INC_DRIVERS_SBUS_H_
#define INC_DRIVERS_SBUS_H_

#include "util/types.h"

typedef struct {
  int16_t ch[16];
  uint8_t failsafe;
  uint8_t framelost;
} receiver_data_t;

void sbus_init();
void sbus_update(receiver_data_t* data);

#endif /* INC_DRIVERS_SBUS_H_ */

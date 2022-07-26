/*
 * gps.cpp
 *
 *  Created on: 5 Jul 2022
 *      Author: Luca
 */

#include "gps.h"
#include <stdio.h>

uint8_t c1;
uint32_t last_package = 0;

TinyGPSPlus gps;

extern UART_HandleTypeDef huart1;

void gpsSetup() {

  uint8_t command[20];

  /* Request UART speed of 115200 */
  sprintf((char *)command, "$PCAS01,5*19\r\n");
  HAL_UART_Transmit(&huart1, command, 14, 100);

  HAL_Delay(200);

  /* Change bus speed to 115200 */
  USART1->CR1 &= ~(USART_CR1_UE);
  USART1->BRR = 417; // Set baud to 115200
  USART1->CR1 |= USART_CR1_UE;

  HAL_Delay(200);

  /* Request 10Hz update rate */
  sprintf((char *)command, "$PCAS02,100*1E\r\n");
  HAL_UART_Transmit(&huart1, command, 16, 100);

  HAL_Delay(10);

  /* Request airbourne <4g mode*/
  sprintf((char *)command, "$PCAS11,7*1A\r\n");
  HAL_UART_Transmit(&huart1, command, 14, 100);

  /* Start receiving data from the module */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&c1, 1);
}

bool gpsRun() {
  if (HAL_GetTick() > (last_package + 100)) {
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&c1, 1);
  }

  return gps.location.isValid();
}

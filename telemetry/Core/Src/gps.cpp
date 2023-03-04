/*
 * gps.cpp
 *
 *  Created on: 5 Jul 2022
 *      Author: Luca
 */

#include "gps.h"
#include <stdio.h>

uint8_t c1;
uint32_t lr1;

static uint8_t ublox_request_115200_baud[] = {
    0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
    0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x07, 0x00,
    0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc4, 0x96, 0xb5, 0x62,
    0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22};
static uint8_t ublox_request_5Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8,
                                      0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
static uint8_t ublox_request_airbourne[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4E, 0xFD};

TinyGPSPlus gps;

extern UART_HandleTypeDef huart1;

void gpsSetup() {

  uint8_t command[20];

  // Check hardware version
  if (HAL_GPIO_ReadPin(HARDWARE_ID_GPIO_Port, HARDWARE_ID_Pin)) {
    // Flight computer
    HAL_UART_Transmit(&huart1, ublox_request_115200_baud,
                      sizeof(ublox_request_115200_baud), 100);
  } else {
    // Groundstation
    /* Request UART speed of 115200 */
    snprintf((char *)command, 15, "$PCAS01,5*19\r\n");
    HAL_UART_Transmit(&huart1, command, 14, 100);
  }

  HAL_Delay(200);

  /* Change bus speed to 115200 */
  USART1->CR1 &= ~(USART_CR1_UE);
  USART1->BRR = 417; // Set baud to 115200
  USART1->CR1 |= USART_CR1_UE;

  HAL_Delay(200);

  // Check hardware version
  if (HAL_GPIO_ReadPin(HARDWARE_ID_GPIO_Port, HARDWARE_ID_Pin)) {
    // Flight computer
    /* Request 5 Hz mode */
    HAL_UART_Transmit(&huart1, ublox_request_5Hz, sizeof(ublox_request_5Hz),
                      100);
    /* Request airbourne, not working yet */
    // HAL_UART_Transmit(&huart1, ublox_request_airbourne,
    // sizeof(ublox_request_airbourne), 100);
  } else {
    // Groundstation
    /* Request 10Hz update rate */
    snprintf((char *)command, 17, "$PCAS02,100*1E\r\n");
    HAL_UART_Transmit(&huart1, command, 16, 100);
  }
}

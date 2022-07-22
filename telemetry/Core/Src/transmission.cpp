/*
 * transmission.cpp
 *
 *  Created on: Mar 2, 2022
 *      Author: Luca
 */
/*
#include "common.h"
#include "transmission.h"


#include "main.h"

#include <stdio.h>

volatile bool busyTransmitting;

extern TIM_HandleTypeDef htim2;



static transmission_direction_e transmission_mode;

uint32_t timeout = 0;
uint32_t link_crc = 0;
uint8_t link_xor = 0;

uint32_t t = 0;

#define SYNC0 0xCA
#define SYNC1 0x75

char data[30];
extern UART_HandleTypeDef huart2;



void ProcessRFPacket()
{


    // Store the LQ/RSSI/Antenna

    LQCalc.add();

    uint8_t lq = LQCalc.getLQ();
    int8_t rssi = Radio.LastPacketRSSI;


    int n = snprintf(data, 30, "LQ: %d RSSI: %d SNR: %d\n", (int)lq, (int)rssi, (int)Radio.LastPacketSNR);
    HAL_UART_Transmit(&huart2, (uint8_t*)data, n, 5);
}


*/


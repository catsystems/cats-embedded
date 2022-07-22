/*
 * transmission.cpp
 *
 *  Created on: Mar 2, 2022
 *      Author: Luca
 */
#include "common.h"
#include "transmission.h"
#include "FHSS/FHSS.h"
#include "FHSS/crc.h"
#include "LQCALC/LQCALC.h"
#include "main.h"

#include <stdio.h>

volatile bool busyTransmitting;

extern TIM_HandleTypeDef htim2;

LQCALC<30> LQCalc;

static mode_e transmission_mode;

uint32_t timeout = 0;
uint32_t link_crc = 0;
uint8_t link_xor = 0;

uint32_t t = 0;

#define SYNC0 0xCA
#define SYNC1 0x75

char data[30];
extern UART_HandleTypeDef huart2;

void SetRFLinkRate(mode_e mode) // Set speed of RF link (hz)
{
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(1);

  if ((ModParams == ExpressLRS_currAirRate_Modparams)) return;

  if(mode == TX){
	  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(),
	                 ModParams->PreambleLen, 0, ModParams->PayloadLength, ModParams->interval);
  }
  else {
	  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(),
	                 ModParams->PreambleLen, 0, ModParams->PayloadLength, 0);
  }
}


void ProcessRFPacket()
{
	LQCalc.inc();
	uint8_t package_crc = link_xor ^ (uint8_t)crc32((const uint8_t*)Radio.RXdataBuffer, 13);

	if(package_crc == Radio.RXdataBuffer[13]){
		connectionState = connected;
		timeout = 0;
    } else {
    	return;
    }

    // Store the LQ/RSSI/Antenna

    LQCalc.add();

    uint8_t lq = LQCalc.getLQ();
    int8_t rssi = Radio.LastPacketRSSI;


    int n = snprintf(data, 30, "LQ: %d RSSI: %d SNR: %d\n", (int)lq, (int)rssi, (int)Radio.LastPacketSNR);
    HAL_UART_Transmit(&huart2, (uint8_t*)data, n, 5);
}

void RXdoneISR()
{
  //ProcessTLMpacket();
  busyTransmitting = false;
  t = TIM2->CNT;
  HAL_TIM_Base_Stop_IT(&htim2);
  TIM2->CNT = 0;
  HAL_TIM_Base_Start_IT(&htim2);

  ProcessRFPacket();
  if(connectionState == connected){
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }


  Radio.SetFrequencyReg(FHSSgetNextFreq());

  Radio.RXnb();
}

void TXdoneISR()
{
  //HandleFHSS();
  //HandlePrepareForTLM();
  Radio.SetFrequencyReg(FHSSgetNextFreq());
  busyTransmitting = false;
}

void transmissionSetup(mode_e mode){

  uint8_t pass[] = "test123";

  link_crc = crc32(pass, 7);
  link_xor = link_crc & 0xFF;
  FHSSrandomiseFHSSsequence(link_crc);

  Radio.RXdoneCallback = &RXdoneISR;
  Radio.TXdoneCallback = &TXdoneISR;

  while (Radio.Begin() == false){
	  uint8_t msg[] = "Error\n";
	  HAL_UART_Transmit(&huart2, (uint8_t*)data, sizeof(msg), 5);
	  HAL_Delay(500);
  }

  Radio.SetOutputPower(-5);
  SetRFLinkRate(mode);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  transmission_mode = mode;

  if(mode == TX) {
	  TIM2->ARR = 1000;
	   HAL_TIM_Base_Start_IT(&htim2);
  } else {
	  TIM2->ARR = 1005;
	    HAL_Delay(10);
	    Radio.RXnb();
	    HAL_TIM_Base_Start_IT(&htim2);
  }

}

void rx_timeout(){
	if(timeout == 50){
			LQCalc.reset();
			HAL_TIM_Base_Stop_IT(&htim2);
			TIM2->CNT = 0;
			connectionState = disconnected;
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			FHSSsetCurrIndex(0);
			Radio.SetFrequencyReg(GetInitialFreq());
		}
	  if(connectionState == connected){
		  LQCalc.inc();
		  Radio.SetFrequencyReg(FHSSgetNextFreq());
	  } else {
		  if(timeout > 5){
			  timeout = 0;
			  Radio.SetFrequencyReg(FHSSgetNextFreq());
		  }
	  }
		timeout++;
}

void tx_transmit(){
	Radio.TXdataBuffer[0] = SYNC0;
	Radio.TXdataBuffer[1] = SYNC1;

	Radio.TXdataBuffer[2] = (link_crc >> 24) & 0xFF;
	Radio.TXdataBuffer[3] = (link_crc >> 16) & 0xFF;
	Radio.TXdataBuffer[4] = (link_crc >> 8) & 0xFF;
	Radio.TXdataBuffer[5] = (link_crc) & 0xFF;

	Radio.TXdataBuffer[6] = 0x0a;
	Radio.TXdataBuffer[7] = 0x0b;
	Radio.TXdataBuffer[8] = 0x0c;
	Radio.TXdataBuffer[9] = 0x0d;
	Radio.TXdataBuffer[10] = 0x0e;
	Radio.TXdataBuffer[11] = 0x0f;
	Radio.TXdataBuffer[12] = 0xa;



	Radio.TXdataBuffer[13] = link_xor  ^ (uint8_t)crc32((const uint8_t*)Radio.TXdataBuffer, 13);
	if(!busyTransmitting) Radio.TXnb();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(transmission_mode == TX){
		tx_transmit();
	} else {
		rx_timeout();
	}

}

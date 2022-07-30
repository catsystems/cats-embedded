/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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

#include "Transmission.h"
#include "FHSS/FHSS.h"
#include "FHSS/crc.h"
#include "main.h"
#include <cstring>

static Transmission *pTransmission;

static inline void rxCallback() { pTransmission->rxDoneISR(); }

static inline void txCallback() { pTransmission->txDoneISR(); }

bool Transmission::begin(TIM_HandleTypeDef *t) {

  /* Catch if already initalized */
  if (radioInitialized == true)
    return radioInitialized;

  timer = t;
  pTransmission = this;
  Radio.RXdoneCallback = &rxCallback;
  Radio.TXdoneCallback = &txCallback;

  if (Radio.Begin() == true) {
    radioInitialized = true;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }

  return radioInitialized;
}

void Transmission::setLinkPhrase(const uint8_t *linkPhrase, uint32_t length) {
  // Reset the linkPhrase
  memset(Settings.linkPhrase, 0, 8);
  // Copy new linkPhrase
  memcpy(Settings.linkPhrase, linkPhrase, length);

  /* If the transmission was already enabled, restart it */
  if (Settings.transmissionEnabled) {
    resetTransmission();
  }
}

void Transmission::setDirection(
    transmission_direction_e transmissionDirection) {
  if (Settings.transmissionDirection != transmissionDirection) {
    Settings.transmissionDirection = transmissionDirection;
    if (Settings.transmissionEnabled) {
      resetTransmission();
    }
  }
}

void Transmission::setMode(transmission_mode_e transmissionMode) {
  Settings.transmissionMode = transmissionMode;
}

void Transmission::setPAGain(int8_t gain) {
  Settings.paGain = gain;

  if (Settings.transmissionEnabled) {
    Radio.SetOutputPower(Settings.powerLevel - Settings.paGain);
  }
}

void Transmission::setPowerLevel(int8_t gain) { Settings.powerLevel = gain; }

void Transmission::writeBytes(const uint8_t *data, uint32_t length) {
  if (length > payloadLength)
    return;
  memcpy(txData, data, length);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

bool Transmission::available() { return dataAvailable; }

bool Transmission::readBytes(uint8_t *buffer, uint32_t length) {
  if (dataAvailable) {
    memcpy(buffer, rxData, length);
    dataAvailable = false;
    return true;
  }
  return false;
}

void Transmission::enableTransmission() {

  if (radioInitialized == false)
    return;

  if (Settings.transmissionEnabled)
    return;

  Settings.transmissionEnabled = true;

  linkCRC = crc32(Settings.linkPhrase, 8);
  linkXOR = linkCRC & 0xFF;

  FHSSrandomiseFHSSsequence(linkCRC);

  Radio.SetOutputPower(Settings.powerLevel - Settings.paGain);

  HAL_Delay(10);

  /* Get the modulation settings */
  modulation_settings_s *const modParams =
      &Settings.modulationConfig[Settings.modeIndex];

  if (Settings.transmissionDirection == TX) {
    Radio.Config(modParams->bw, modParams->sf, modParams->cr, GetInitialFreq(),
                 modParams->PreambleLen, 0, modParams->PayloadLength,
                 modParams->interval);
  } else {
    Radio.Config(modParams->bw, modParams->sf, modParams->cr, GetInitialFreq(),
                 modParams->PreambleLen, 0, modParams->PayloadLength, 0);
  }

  payloadLength = modParams->PayloadLength;

  HAL_Delay(10);

  if (Settings.transmissionDirection == TX) {
    TIM2->ARR = 1000;
    HAL_TIM_Base_Start_IT(timer);
  } else {
    TIM2->ARR = 1005;
    Radio.RXnb();
    HAL_TIM_Base_Start_IT(timer);
  }
}

void Transmission::disableTransmission() {
  /* Wait until done transmitting / receiving*/
  while (busyTransmitting)
    ;

  if (!Settings.transmissionEnabled)
    return;

  Settings.transmissionEnabled = false;

  /* Disable Timer */
  HAL_TIM_Base_Stop_IT(timer);
  TIM2->CNT = 0;

  /* Put Radio in Idle Mode */
  Radio.SetIdleMode();

  LQCalc.reset();
  connectionState = disconnected;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void Transmission::processRFPacket() {
  LQCalc.inc();

  uint8_t crc =
      (uint8_t)crc32((const uint8_t *)Radio.RXdataBuffer, payloadLength - 1);

  if ((linkXOR ^ crc) == Radio.RXdataBuffer[payloadLength - 1]) {
    if (connectionState == tentative)
      connectionState = connected;
    else if (connectionState == disconnected)
      connectionState = tentative;
    dataAvailable = true;
    timeout = 0;

    memcpy(rxData, (const uint8_t *)Radio.RXdataBuffer, payloadLength);

    LQCalc.add();
  }
}

void Transmission::rxDoneISR() {
  busyTransmitting = false;

  /* Reset the timer */
  HAL_TIM_Base_Stop_IT(timer);
  TIM2->CNT = 0;
  HAL_TIM_Base_Start_IT(timer);

  processRFPacket();

  if (connectionState == connected) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }

  Radio.SetFrequencyReg(FHSSgetNextFreq());

  Radio.RXnb();
}

void Transmission::txDoneISR() {
  busyTransmitting = false;

  Radio.SetFrequencyReg(FHSSgetNextFreq());
}

void Transmission::rxTimeout() {

  if (timeout == 50) {
    LQCalc.reset();
    connectionState = disconnected;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    FHSSsetCurrIndex(0);
    Radio.SetFrequencyReg(GetInitialFreq());
  }

  if (connectionState == connected) {
    LQCalc.inc();
    Radio.SetFrequencyReg(FHSSgetNextFreq());
  } else {
    if (timeout > 5) {
      timeout = 0;
      Radio.SetFrequencyReg(FHSSgetNextFreq());
    }
  }

  timeout++;
}

void Transmission::txTransmit() {
  /* Add payload to tx buffer */
  for (uint32_t i = 0; i < payloadLength - 1; i++) {
    Radio.TXdataBuffer[i] = txData[i];
  }

  /* Calculate CRC and store in last position */
  uint8_t crc = (uint8_t)crc32((const uint8_t *)txData, payloadLength - 1);
  Radio.TXdataBuffer[payloadLength - 1] = linkXOR ^ crc;

  /* Transmit message */
  if (!busyTransmitting)
    Radio.TXnb();
}

transmission_direction_e Transmission::getDirection() {
  return Settings.transmissionDirection;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (pTransmission->getDirection() == TX) {
    pTransmission->txTransmit();
  } else {
    pTransmission->rxTimeout();
  }
}

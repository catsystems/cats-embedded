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

#pragma once
#include "LQCALC/LQCALC.h"
#include "SX1280Driver/SX1280.h"
#include "TransmissionSettings.h"
#include "stm32g0xx_hal.h"

#define MAX_PAYLOAD_SIZE 20

typedef struct {
  uint8_t rssi;
  uint8_t lq;
  int8_t snr;
} linkInfo_t;

class Transmission {
public:
  bool begin(TIM_HandleTypeDef *t);

  void setDirection(transmission_direction_e transmissionDirection);
  void setMode(transmission_mode_e transmissionMode);
  void setPAGain(int8_t gain);
  void setPowerLevel(int8_t gain);
  void setLinkPhraseCrc(const uint32_t PhraseCrc);

  /* Functions to read and write transmission data */
  bool available();
  void writeBytes(const uint8_t *buffer, uint32_t length);
  bool readBytes(uint8_t *buffer, uint32_t length);
  bool infoAvailable();
  bool readInfo(linkInfo_t *info);

  transmission_direction_e getDirection();

  void enableTransmission();
  void disableTransmission();

  void txTransmit();
  void rxTimeout();

  void rxDoneISR();
  void txDoneISR();

private:
  void processRFPacket();

  void resetTransmission() {
    disableTransmission();
    HAL_Delay(10);
    enableTransmission();
  }

  SX1280Driver Radio;
  LQCALC<30> LQCalc;
  TransmissionSettings Settings;

  TIM_HandleTypeDef *timer;
  uint32_t timeout = 0;

  bool radioInitialized = false;

  connectionState_e connectionState = disconnected;
  volatile bool busyTransmitting;
  uint8_t linkXOR[2];
  uint32_t linkCRC;

  volatile bool dataAvailable = false;
  volatile bool linkInfoAvailable = false;
  uint32_t payloadLength = 0;
  uint8_t txData[MAX_PAYLOAD_SIZE];
  uint8_t rxData[MAX_PAYLOAD_SIZE];
};

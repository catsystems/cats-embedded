/// CATS Flight Software
/// Copyright (C) 2022 Control and Telemetry Systems
///
/// This program is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// This program is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include "TransmissionSettings.hpp"
#include "stm32g0xx_hal.h"

#include <LqCalculator.hpp>
#include <Sx1280Driver.hpp>

struct linkInfo_t {
  int8_t rssi;
  uint8_t lq;
  int8_t snr;
};

class Transmission {
 public:
  bool begin(TIM_HandleTypeDef *t);

  void setDirection(transmission_direction_e transmissionDirection);
  void setMode(transmission_mode_e transmissionMode);
  void setPAGain(int8_t gain);
  void setPowerLevel(int8_t gain);
  void setLinkPhraseCrc(uint32_t phraseCrc);

  /* Functions to read and write transmission data */
  [[nodiscard]] bool available() const;
  void writeBytes(const uint8_t *buffer, uint32_t length);
  bool readBytes(uint8_t *buffer, uint32_t length);
  [[nodiscard]] bool infoAvailable() const;
  bool readInfo(linkInfo_t *info);

  [[nodiscard]] transmission_direction_e getDirection() const;

  void enableTransmission();
  void disableTransmission();

  void txTransmit();
  void rxTimeout();

  void rxDoneISR();
  void txDoneISR();

 private:
  bool processRFPacket();

  void resetTransmission() {
    disableTransmission();
    HAL_Delay(10);
    enableTransmission();
  }

  SX1280Driver Radio;
  LqCalculator<30> LqCalc;
  TransmissionSettings Settings;

  TIM_HandleTypeDef *timer{nullptr};
  uint32_t timeout{0};

  bool radioInitialized{false};

  connectionState_e connectionState = disconnected;
  volatile bool busyTransmitting{false};
  uint8_t linkXOR[2]{};
  uint32_t linkCRC{0};

  volatile bool dataAvailable{false};
  volatile bool linkInfoAvailable{false};
  uint32_t payloadLength{0};

  constexpr static uint8_t kMaxPayloadSize{20};
  uint8_t txData[kMaxPayloadSize]{};
  uint8_t rxData[kMaxPayloadSize]{};
};

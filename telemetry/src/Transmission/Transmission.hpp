/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

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

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

#include "Transmission.hpp"
#include "Fhss/Fhss.hpp"
#include "Main.hpp"

#include <Crc.hpp>
#include <cstring>

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static Transmission *pTransmission;

static inline void rxCallback() { pTransmission->rxDoneISR(); }

static inline void txCallback() { pTransmission->txDoneISR(); }

bool Transmission::begin(TIM_HandleTypeDef *t) {
  /* Catch if already initalized */
  if (radioInitialized) {
    return radioInitialized;
  }

  timer = t;
  pTransmission = this;
  Radio.RXdoneCallback = &rxCallback;
  Radio.TXdoneCallback = &txCallback;

  if (Radio.Begin()) {
    radioInitialized = true;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }

  return radioInitialized;
}

void Transmission::setLinkPhraseCrc(const uint32_t phraseCrC) {
  // Set new linkPhrase
  Settings.linkPhraseCrC = phraseCrC;

  /* If the transmission was already enabled, restart it */
  if (Settings.transmissionEnabled) {
    resetTransmission();
  }
}

void Transmission::setDirection(transmission_direction_e transmissionDirection) {
  if (Settings.transmissionDirection != transmissionDirection) {
    Settings.transmissionDirection = transmissionDirection;
    if (Settings.transmissionEnabled) {
      resetTransmission();
    }
  }
}

void Transmission::setMode(transmission_mode_e transmissionMode) {
  if (Settings.transmissionMode != transmissionMode) {
    Settings.transmissionMode = transmissionMode;
    if (Settings.transmissionEnabled) {
      resetTransmission();
    }
  }
}

void Transmission::setPAGain(int8_t gain) {
  Settings.paGain = gain;

  if (Settings.transmissionEnabled) {
    Radio.SetOutputPower(static_cast<int8_t>(Settings.powerLevel - Settings.paGain));
  }
}

void Transmission::setPowerLevel(int8_t gain) { Settings.powerLevel = gain; }

void Transmission::writeBytes(const uint8_t *buffer, uint32_t length) {
  if (length > payloadLength) {
    return;
  }
  memcpy(txData, buffer, length);
}

bool Transmission::available() const { return dataAvailable; }

bool Transmission::infoAvailable() const { return linkInfoAvailable; }

bool Transmission::readBytes(uint8_t *buffer, uint32_t length) {
  if (dataAvailable) {
    memcpy(buffer, rxData, length);
    dataAvailable = false;
    return true;
  }
  return false;
}

bool Transmission::readInfo(linkInfo_t *info) {
  *info = {.rssi = Radio.LastPacketRSSI, .lq = LqCalc.getLQ(), .snr = Radio.LastPacketSNR};
  linkInfoAvailable = false;
  return true;
}

void Transmission::enableTransmission() {
  if (!radioInitialized) {
    return;
  }

  if (Settings.transmissionEnabled) {
    return;
  }

  Settings.transmissionEnabled = true;

  linkCRC = Settings.linkPhraseCrC;
  linkXOR[0] = (linkCRC >> 8U) & 0xFFU;
  linkXOR[1] = linkCRC & 0xFFU;

  FHSSrandomiseFHSSsequence(linkCRC);

  Radio.SetOutputPower(static_cast<int8_t>(Settings.powerLevel - Settings.paGain));

  HAL_Delay(10);

  /* Get the modulation settings */
  modulation_settings_s *const modParams = &Settings.modulationConfig[Settings.modeIndex];

  if (Settings.transmissionDirection == TX) {
    Radio.Config(modParams->bw, modParams->sf, modParams->cr, GetInitialFreq(), modParams->PreambleLen, false,
                 modParams->PayloadLength, modParams->interval);
  } else {
    Radio.Config(modParams->bw, modParams->sf, modParams->cr, GetInitialFreq(), modParams->PreambleLen, false,
                 modParams->PayloadLength, 0);
  }

  payloadLength = modParams->PayloadLength;

  HAL_Delay(10);

  if (Settings.transmissionDirection == TX) {
    TIM2->ARR = 1000U;
    HAL_TIM_Base_Start_IT(timer);
  } else {
    TIM2->ARR = 1005U;
    Radio.RXnb();
    HAL_TIM_Base_Start_IT(timer);
  }
}

void Transmission::disableTransmission() {
  /* Wait until done transmitting / receiving*/
  while (busyTransmitting) {
  };

  if (!Settings.transmissionEnabled) {
    return;
  }

  Settings.transmissionEnabled = false;

  /* Disable Timer */
  HAL_TIM_Base_Stop_IT(timer);
  TIM2->CNT = 0;

  /* Put Radio in Idle Mode */
  Radio.SetIdleMode();

  LqCalc.reset();
  connectionState = disconnected;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

/**
 * Process the received package
 *
 * @return true if package is valid
 */
bool Transmission::processRFPacket() {
  LqCalc.inc();

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast) safe to cast as RXdataBuffer is updated only in the ISR
  const auto crc = static_cast<uint16_t>(crc32(const_cast<const uint8_t *>(Radio.RXdataBuffer), payloadLength - 2));

  bool is_valid = (linkXOR[0] ^ static_cast<uint8_t>(crc >> 8U)) == Radio.RXdataBuffer[payloadLength - 2];
  is_valid &= (linkXOR[1] ^ static_cast<uint8_t>(crc)) == Radio.RXdataBuffer[payloadLength - 1];

  if (is_valid) {
    if (connectionState == tentative) {
      connectionState = connected;
    } else if (connectionState == disconnected) {
      connectionState = tentative;
    }
    dataAvailable = true;
    linkInfoAvailable = true;
    timeout = 0;

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast) safe to cast as RXdataBuffer is updated only in the ISR
    memcpy(rxData, const_cast<const uint8_t *>(Radio.RXdataBuffer), payloadLength);

    LqCalc.add();
    return true;
  }
  return false;
}

void Transmission::rxDoneISR() {
  busyTransmitting = false;

  if (Settings.transmissionDirection == RX) {
    /* Reset the timer */

    if (processRFPacket()) {
      HAL_TIM_Base_Stop_IT(timer);
      TIM2->CNT = 0;
      HAL_TIM_Base_Start_IT(timer);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      if (Settings.transmissionMode == BIDIRECTIONAL) {
        txTransmit();
      } else {
        Radio.SetFrequencyReg(FHSSgetNextFreq());
        Radio.RXnb();
      }
    }
  }
  if (Settings.transmissionDirection == TX) {
    processRFPacket();
  }
}

void Transmission::txDoneISR() {
  busyTransmitting = false;

  if (Settings.transmissionMode == UNIDIRECTIONAL) {
    // Unidirectional TX mode
  }
  if (Settings.transmissionDirection == TX) {
    // Bidirectional TX mode -> Go to RX (Keep the timeout)
    Radio.RXnb();
  } else {
    // Bidirectional RX mode -> After transmitting go to next freq
    Radio.SetFrequencyReg(FHSSgetNextFreq());
    Radio.RXnb();
  }
}

void Transmission::rxTimeout() {
  if (timeout == 50) {
    LqCalc.reset();
    connectionState = disconnected;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    FHSSsetCurrIndex(0);
    Radio.SetFrequencyReg(GetInitialFreq());
  }

  if (connectionState == connected) {
    LqCalc.inc();
    Radio.SetFrequencyReg(FHSSgetNextFreq());
    linkInfoAvailable = true;
  } else {
    if (connectionState == tentative) {
      connectionState = disconnected;
    }
    if (timeout > 5) {
      timeout = 0;
      Radio.SetFrequencyReg(FHSSgetNextFreq());
    }
  }

  timeout++;
}

void Transmission::txTransmit() {
  /* Add payload to tx buffer */
  if (Settings.transmissionDirection == TX) {
    Radio.SetFrequencyReg(FHSSgetNextFreq());
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }

  for (uint32_t i = 0; i < payloadLength - 1; i++) {
    Radio.TXdataBuffer[i] = txData[i];
  }

  /* Calculate CRC and store in last position */
  const auto crc = static_cast<uint16_t>(crc32(static_cast<const uint8_t *>(txData), payloadLength - 2));
  Radio.TXdataBuffer[payloadLength - 2] = linkXOR[0] ^ static_cast<uint8_t>(crc >> 8U);
  Radio.TXdataBuffer[payloadLength - 1] = linkXOR[1] ^ static_cast<uint8_t>(crc);

  /* Transmit message */
  if (!busyTransmitting) {
    Radio.TXnb();
  }
}

transmission_direction_e Transmission::getDirection() const { return Settings.transmissionDirection; }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  (void)htim;
  if (pTransmission->getDirection() == TX) {
    pTransmission->txTransmit();
  } else {
    pTransmission->rxTimeout();
  }
}

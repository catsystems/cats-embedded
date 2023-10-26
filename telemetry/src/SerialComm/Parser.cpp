/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <Crc.hpp>

#include "Common.hpp"
#include "Parser.hpp"

void Parser::parse() {
  cmd_table[opCodeIndex].cmd(&buffer[2], dataIndex);

  /* Reset the parser buffer */
  reset();
}

int32_t Parser::getOpCodeIndex(uint8_t opCode) {
  for (int32_t i = 0; i < CMD_NUMBER; i++) {
    if (opCode == cmd_table[i].identifier) {
      return i;
    }
  }
  return -1;
}

void Parser::process(uint8_t ch) {
  switch (state) {
    case STATE_OP:
      opCodeIndex = getOpCodeIndex(ch);
      if (opCodeIndex >= 0) {
        buffer[INDEX_OP] = ch;
        state = STATE_LEN;
      }
      break;
    case STATE_LEN:
      if (ch <= 16) {
        buffer[INDEX_LEN] = ch;
        if (ch > 0) {
          state = STATE_DATA;
        } else {
          state = STATE_CRC;
        }
      }
      break;
    case STATE_DATA:
      if ((buffer[INDEX_LEN] - dataIndex) > 0) {
        buffer[dataIndex + 2] = ch;
        dataIndex++;
      }
      if ((buffer[INDEX_LEN] - dataIndex) == 0) {
        state = STATE_CRC;
      }
      break;
    case STATE_CRC: {
      const uint8_t crc = crc8(buffer, dataIndex + 2);
      if (crc == ch) {
        parse();
      } else {
        reset();
      }
    } break;
    default:
      break;
  }
}

void Parser::cmdDirection(uint8_t *args, uint32_t length) {
  if (length != 1) {
    return;
  }

  if (args[0] == TX || args[0] == RX) {
    link.setDirection(static_cast<transmission_direction_e>(args[0]));
  }
}

void Parser::cmdPAGain(uint8_t *args, uint32_t length) {
  if (length != 1) {
    return;
  }

  if (args[0] < 50) {
    link.setPAGain(static_cast<int8_t>(args[0]));
  }
}

void Parser::cmdPowerLevel(uint8_t *args, uint32_t length) {
  if (length != 1) {
    return;
  }

  link.setPowerLevel(static_cast<int8_t>(args[0]));
}

void Parser::cmdMode(uint8_t *args, uint32_t length) {
  if (length != 1) {
    return;
  }

  link.setMode(static_cast<transmission_mode_e>(args[0]));
}

void Parser::cmdModeIndex(uint8_t *args, uint32_t length) {
  // UNUSED
}

// NOLINTNEXTLINE(readability-non-const-parameter) can't be const since the function pointer has a non-const param
void Parser::cmdLinkPhrase(uint8_t *args, uint32_t length) {
  if (length != 4) {
    return;
  }

  if (args[0] != 0) {
    uint32_t phrasecrc = args[0] << 24U;
    phrasecrc += args[1] << 16U;
    phrasecrc += args[2] << 8U;
    phrasecrc += args[3];
    link.setLinkPhraseCrc(phrasecrc);
  }
}

void Parser::cmdEnable([[maybe_unused]] uint8_t *args, uint32_t length) {
  if (length != 0) {
    return;
  }
  link.enableTransmission();
}

void Parser::cmdDisable([[maybe_unused]] uint8_t *args, uint32_t length) {
  if (length != 0) {
    return;
  }

  link.disableTransmission();
}

void Parser::cmdTX(uint8_t *args, uint32_t length) { link.writeBytes(args, length); }

void Parser::cmdVersionNum([[maybe_unused]] uint8_t *args, [[maybe_unused]] uint32_t length) {
  send_version_num = true;
}

void Parser::cmdBootloader(uint8_t *args, uint32_t length) {
  // UNUSED
}

void Parser::cmdRX(uint8_t *args, uint32_t length) {
  // UNUSED
}

void Parser::cmdInfo(uint8_t *args, uint32_t length) {
  // UNUSED
}

void Parser::cmdGNSSLoc(uint8_t *args, uint32_t length) {
  // UNUSED
}

void Parser::cmdGNSSTime(uint8_t *args, uint32_t length) {
  // UNUSED
}

void Parser::cmdGNSSInfo(uint8_t *args, uint32_t length) {
  // UNUSED
}

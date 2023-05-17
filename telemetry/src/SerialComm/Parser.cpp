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

#include "Parser.hpp"
#include <Crc.hpp>
#include "Common.hpp"

void Parser::parse() {
  cmd_table[opCodeIndex].cmd(&buffer[2], dataIndex);

  /* Reset the parser buffer */
  reset();
}

int32_t Parser::getOpCodeIndex(uint8_t opCode) {
  for (int32_t i = 0; i < CMD_NUMBER; i++) {
    if (opCode == cmd_table[i].identifier) return i;
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
      uint8_t crc = crc8(buffer, dataIndex + 2);
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
  if (length != 1) return;

  if (args[0] == TX || args[0] == RX) {
    link.setDirection((transmission_direction_e)args[0]);
  }
}

void Parser::cmdPAGain(uint8_t *args, uint32_t length) {
  if (length != 1) return;

  if (args[0] < 50) {
    link.setPAGain(args[0]);
  }
}

void Parser::cmdPowerLevel(uint8_t *args, uint32_t length) {
  if (length != 1) return;

  link.setPowerLevel(args[0]);
}

void Parser::cmdMode(uint8_t *args, uint32_t length) {
  if (length != 1) return;

  link.setMode((transmission_mode_e)args[0]);
}

void Parser::cmdModeIndex(uint8_t *args, uint32_t length) {
  // UNUSED
}

void Parser::cmdLinkPhrase(uint8_t *args, uint32_t length) {
  if (length != 4) return;

  if (args[0] != 0) {
    uint32_t phrasecrc = args[0] << 24;
    phrasecrc += args[1] << 16;
    phrasecrc += args[2] << 8;
    phrasecrc += args[3];
    link.setLinkPhraseCrc(phrasecrc);
  }
}

void Parser::cmdEnable(uint8_t *args, uint32_t length) {
  if (length != 0) return;
  link.enableTransmission();
}

void Parser::cmdDisable(uint8_t *args, uint32_t length) {
  if (length != 0) return;
  link.disableTransmission();
}

void Parser::cmdTX(uint8_t *args, uint32_t length) { link.writeBytes(args, length); }

void Parser::cmdVersionNum(uint8_t *args, uint32_t length) { send_version_num = true; }

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

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

#include <cstdint>
#include "telemetry_reg.h"

typedef void cmd_fn(uint8_t *args, uint32_t length);

typedef struct {
  const char identifier;
  cmd_fn *cmd;
} cmd_t;

#define MAX_CMD_BUFFER 20

#define CMD_DEF(identifier, cmd)                                               \
  { identifier, cmd }

/* The parser is not interrupt safe! */

class Parser {
public:
  void process(uint8_t ch);

  void parse();

  void reset() {
    dataIndex = 0;
    opCodeIndex = -1;
    state = STATE_OP;
  }

  // bool ready() { return (length - bufferIndex) == 0 && validSize; }

  static void cmdDirection(uint8_t *args, uint32_t length);
  static void cmdPAGain(uint8_t *args, uint32_t length);
  static void cmdPowerLevel(uint8_t *args, uint32_t length);
  static void cmdMode(uint8_t *args, uint32_t length);
  static void cmdModeIndex(uint8_t *args, uint32_t length);
  static void cmdLinkPhrase(uint8_t *args, uint32_t length);

  static void cmdEnable(uint8_t *args, uint32_t length);
  static void cmdDisable(uint8_t *args, uint32_t length);

  static void cmdTX(uint8_t *args, uint32_t length);
  static void cmdRX(uint8_t *args, uint32_t length);
  static void cmdInfo(uint8_t *args, uint32_t length);

  static void cmdGNSSLoc(uint8_t *args, uint32_t length);
  static void cmdGNSSTime(uint8_t *args, uint32_t length);
  static void cmdGNSSInfo(uint8_t *args, uint32_t length);

  static void cmdVersionNum(uint8_t *args, uint32_t length);

private:
  int32_t getOpCodeIndex(uint8_t opCode);

  uint8_t buffer[MAX_CMD_BUFFER];
  uint32_t dataIndex = 0;

  int32_t opCodeIndex = -1;

  typedef enum {
    STATE_OP,
    STATE_LEN,
    STATE_DATA,
    STATE_CRC,
  } state_e;

  enum {
    INDEX_OP = 0,
    INDEX_LEN = 1,
  };

  state_e state = STATE_OP;

  enum {
    CMD_NUMBER = 15,
  };

  const cmd_t cmd_table[CMD_NUMBER] = {
    CMD_DEF(CMD_DIRECTION, cmdDirection),    CMD_DEF(CMD_PA_GAIN, cmdPAGain),
	CMD_DEF(CMD_POWER_LEVEL, cmdPowerLevel), CMD_DEF(CMD_MODE, cmdMode),
	CMD_DEF(CMD_MODE_INDEX, cmdModeIndex),   CMD_DEF(CMD_LINK_PHRASE, cmdLinkPhrase),

	CMD_DEF(CMD_ENABLE, cmdEnable),          CMD_DEF(CMD_DISBALE, cmdDisable),

	CMD_DEF(CMD_TX, cmdTX),                  CMD_DEF(CMD_RX, cmdRX),
	CMD_DEF(CMD_INFO, cmdInfo),

	CMD_DEF(CMD_GNSS_LOC, cmdGNSSLoc),       CMD_DEF(CMD_GNSS_TIME, cmdGNSSTime),
	CMD_DEF(CMD_GNSS_INFO, cmdGNSSInfo),     CMD_DEF(CMD_VERSION_INFO, cmdVersionNum),
  };
};

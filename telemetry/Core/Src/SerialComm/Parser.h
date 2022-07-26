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

  bool ready() { return (length - bufferIndex) == 0 && validSize; }

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

private:
  void reset() {
    bufferIndex = 0;
    validSize = false;
    opCodeIndex = -1;
  }

  int32_t getOpCodeIndex(uint8_t opCode);

  uint8_t buffer[MAX_CMD_BUFFER];
  uint32_t bufferIndex = 0;

  bool validSize = false;

  int32_t opCodeIndex = -1;
  uint8_t opCode;
  uint8_t length;
  uint8_t remainingLength;

  enum {
    CMD_NUMBER = 14,
  };

  const cmd_t cmd_table[CMD_NUMBER] = {
      CMD_DEF(0x10, cmdDirection),  CMD_DEF(0x11, cmdPAGain),
      CMD_DEF(0x12, cmdPowerLevel), CMD_DEF(0x13, cmdMode),
      CMD_DEF(0x14, cmdModeIndex),  CMD_DEF(0x15, cmdLinkPhrase),

      CMD_DEF(0x20, cmdEnable),     CMD_DEF(0x21, cmdDisable),

      CMD_DEF(0x30, cmdTX),         CMD_DEF(0x31, cmdRX),
      CMD_DEF(0x32, cmdInfo),

      CMD_DEF(0x40, cmdGNSSLoc),    CMD_DEF(0x41, cmdGNSSTime),
      CMD_DEF(0x42, cmdGNSSInfo),
  };
};

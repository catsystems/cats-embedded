/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstdint>

#include "Telemetry_reg.h"

using cmd_fn = void (*)(uint8_t *args, uint32_t length);

struct cmd_t {
  const uint8_t identifier;
  cmd_fn cmd;
};

constexpr uint8_t kMaxCmdBuffer = 20;

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

  static void cmdBootloader(uint8_t *args, uint32_t length);

 private:
  int32_t getOpCodeIndex(uint8_t opCode);

  uint8_t buffer[kMaxCmdBuffer];
  uint32_t dataIndex = 0;

  int32_t opCodeIndex = -1;

  enum state_e {
    STATE_OP,
    STATE_LEN,
    STATE_DATA,
    STATE_CRC,
  };

  enum {
    INDEX_OP = 0,
    INDEX_LEN = 1,
  };

  state_e state = STATE_OP;

  enum {
    CMD_NUMBER = 16,
  };

  const cmd_t cmd_table[CMD_NUMBER] = {
      {CMD_DIRECTION, cmdDirection},
      {CMD_PA_GAIN, cmdPAGain},
      {CMD_POWER_LEVEL, cmdPowerLevel},
      {CMD_MODE, cmdMode},
      {CMD_MODE_INDEX, cmdModeIndex},
      {CMD_LINK_PHRASE, cmdLinkPhrase},
      {CMD_ENABLE, cmdEnable},
      {CMD_DISBALE, cmdDisable},

      {CMD_TX, cmdTX},
      {CMD_RX, cmdRX},
      {CMD_INFO, cmdInfo},

      {CMD_GNSS_LOC, cmdGNSSLoc},
      {CMD_GNSS_TIME, cmdGNSSTime},
      {CMD_GNSS_INFO, cmdGNSSInfo},
      {CMD_VERSION_INFO, cmdVersionNum},

      {CMD_BOOTLOADER, cmdBootloader},
  };
};

/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "telemetryData.hpp"
#include "telemetry_reg.hpp"

#include <cstdint>
#include <ctime>
#include <functional>

struct link_info_t {
  uint8_t rssi;
  uint8_t lq;
  int8_t snr;
};

inline constexpr uint8_t MAX_CMD_BUFFER = 20;

/* The parser is not interrupt safe! */

class Parser {
 public:
  void process(uint8_t ch);

  void parse();

  void init(TelemetryData* d, TelemetryInfo* i, TelemetryLocation* l = nullptr, TelemetryTime* t = nullptr) {
    data = d;
    info = i;
    location = l;
    time = t;
  }

  void reset() {
    dataIndex = 0;
    opCodeIndex = -1;
    state = STATE_OP;
  }

  void cmdRX(uint8_t* args, uint32_t length);
  void cmdInfo(uint8_t* args, uint32_t length);

  void cmdGNSSLoc(uint8_t* args, uint32_t length);
  void cmdGNSSTime(uint8_t* args, uint32_t length);
  void cmdGNSSInfo(uint8_t* args, uint32_t length);

 private:
  static int32_t getOpCodeIndex(uint8_t opCode);

  TelemetryData* data{nullptr};
  TelemetryInfo* info{nullptr};
  TelemetryLocation* location{nullptr};
  TelemetryTime* time{nullptr};

  uint8_t buffer[MAX_CMD_BUFFER]{};
  uint32_t dataIndex = 0;

  int32_t opCodeIndex = -1;

  link_info_t linkInfo{};

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
};

enum {
  CMD_NUMBER = 5,
};

using cmd_fn = void (Parser::*)(uint8_t* args, uint32_t length);

const cmd_fn commandFunction[] = {&Parser::cmdRX, &Parser::cmdInfo, &Parser::cmdGNSSLoc, &Parser::cmdGNSSTime,
                                  &Parser::cmdGNSSInfo};

const uint8_t cmdIndex[] = {CMD_RX, CMD_INFO, CMD_GNSS_LOC, CMD_GNSS_TIME, CMD_GNSS_INFO};

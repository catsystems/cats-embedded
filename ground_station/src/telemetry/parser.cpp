/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "parser.hpp"

#include "console.hpp"
#include "crc.hpp"

namespace {
constexpr uint32_t kRadioPayloadLength = 16U;

constexpr bool isTelemetryPayloadLengthValid(uint32_t length) {
  return length >= sizeof(packedRXMessage) && length <= kRadioPayloadLength;
}

static_assert(isTelemetryPayloadLengthValid(sizeof(packedRXMessage)));
static_assert(isTelemetryPayloadLengthValid(kRadioPayloadLength));
static_assert(!isTelemetryPayloadLengthValid(sizeof(packedRXMessage) - 1U));
}  // namespace

void Parser::parse() {
  (this->*commandFunction[opCodeIndex])(&buffer[2], dataIndex);

  /* Reset the parser buffer */
  reset();
}

int32_t Parser::getOpCodeIndex(uint8_t opCode) {
  for (int32_t i = 0; i < CMD_NUMBER; i++) {
    if (opCode == cmdIndex[i]) {
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

void Parser::cmdRX(uint8_t *args, uint32_t length) {
  // The Vega radio always forwards a full 16-byte RF payload. The decoded
  // telemetry structure currently occupies 15 bytes; accept the trailing RF
  // padding byte without copying it past the structure boundary.
  if (!isTelemetryPayloadLengthValid(length)) {
    return;
  }
  data->commit(args, sizeof(packedRXMessage));
  if (sink != nullptr) {
    sink->onTelemetryPacket(data->getRxData(), source);
  }
}

void Parser::cmdInfo(uint8_t *args, uint32_t length) { info->commit(args, length); }

void Parser::cmdGNSSLoc(uint8_t *args, uint32_t length) {
  if (location != nullptr) {
    location->commit(args, length);
  }
}

void Parser::cmdGNSSTime(uint8_t *args, uint32_t length) {
  if (time != nullptr) {
    time->commit(args, length);
  }
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static) function ptr doesn't work if it's static
void Parser::cmdGNSSInfo(uint8_t *args [[maybe_unused]], uint32_t length [[maybe_unused]]) {
  // TODO: Do Something
}

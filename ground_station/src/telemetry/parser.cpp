/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "parser.hpp"

#include <algorithm>
#include <cmath>
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
  const uint32_t now = millis();
  if (state != STATE_OP && now - lastByteMs > 100U) reset();
  lastByteMs = now;
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
      } else {
        reset();
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
  const uint32_t now = millis();
  portENTER_CRITICAL(&diagnosticsMux);
  auto &stats = observation.radio;
  ++stats.packets;
  stats.maxGapMs = std::max(stats.maxGapMs, now - stats.lastPacketMs);
  stats.lastPacketMs = now;
  portEXIT_CRITICAL(&diagnosticsMux);
  if (sink != nullptr) {
    sink->onTelemetryPacket(data->getRxData(), source);
  }
}

void Parser::cmdInfo(uint8_t *args, uint32_t length) {
  if (length != sizeof(TelemetryInfoData) || args[0] > 100) return;
  info->commit(args, length);
  portENTER_CRITICAL(&diagnosticsMux);
  auto &stats = observation.radio;
  ++stats.infoSamples;
  stats.lqSum += args[0];
  stats.minimumSnr = std::min(stats.minimumSnr, static_cast<int8_t>(args[2]));
  portEXIT_CRITICAL(&diagnosticsMux);
}

void Parser::cmdGNSSLoc(uint8_t *args, uint32_t length) {
  if (length != sizeof(TelemetryLocationData)) return;
  TelemetryLocationData value{};
  memcpy(&value, args, sizeof(value));
  if (!std::isfinite(value.lat) || !std::isfinite(value.lon) || std::fabs(value.lat) > 90 ||
      std::fabs(value.lon) > 180 || (value.lat == 0 && value.lon == 0))
    return;
  const uint32_t now = millis();
  portENTER_CRITICAL(&diagnosticsMux);
  ++observation.fixCount;
  observation.lastFixMs = now;
  observation.latitude = value.lat;
  observation.longitude = value.lon;
  portEXIT_CRITICAL(&diagnosticsMux);
  if (location != nullptr) {
    location->commit(args, length);
  }
}

void Parser::cmdGNSSTime(uint8_t *args, uint32_t length) {
  if (length != 3 || args[0] > 59 || args[1] > 59 || args[2] > 23) return;
  const uint32_t now = millis();
  portENTER_CRITICAL(&diagnosticsMux);
  ++observation.timeUpdates;
  observation.lastTimeMs = now;
  observation.utcSeconds = args[2] * 3600U + args[1] * 60U + args[0];
  portEXIT_CRITICAL(&diagnosticsMux);
  if (time != nullptr) {
    time->commit(args, length);
  }
}

void Parser::cmdGNSSInfo(uint8_t *args, uint32_t length) {
  if (length != 1) return;
  portENTER_CRITICAL(&diagnosticsMux);
  observation.satellites = args[0];
  portEXIT_CRITICAL(&diagnosticsMux);
}

void Parser::cmdVersion(uint8_t *args, uint32_t length) {
  if (length == 0 || length > 16 ||
      !std::all_of(args, args + length, [](uint8_t value) { return value >= 32 && value <= 126; }) ||
      !std::any_of(args, args + length, [](uint8_t value) { return value > 32; }))
    return;
  portENTER_CRITICAL(&diagnosticsMux);
  memset(observation.version, 0, sizeof(observation.version));
  memcpy(observation.version, args, length);
  ++observation.versionReplies;
  portEXIT_CRITICAL(&diagnosticsMux);
}

void Parser::cmdTemperature(uint8_t *args [[maybe_unused]], uint32_t length [[maybe_unused]]) {
  // Consume the existing periodic frame without treating its payload as opcodes.
}

SelfTestLinkObservation Parser::diagnostics() const {
  portENTER_CRITICAL(&diagnosticsMux);
  const auto copy = observation;
  portEXIT_CRITICAL(&diagnosticsMux);
  return copy;
}

void Parser::resetReceptionStats(uint32_t now) {
  portENTER_CRITICAL(&diagnosticsMux);
  observation.radio = {};
  observation.radio.lastPacketMs = now;
  portEXIT_CRITICAL(&diagnosticsMux);
}

/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "telemetryData.hpp"

class ITelemetryPacketSink {
 public:
  virtual ~ITelemetryPacketSink() = default;
  virtual void onTelemetryPacket(const packedRXMessage& packet, uint8_t source) = 0;
};

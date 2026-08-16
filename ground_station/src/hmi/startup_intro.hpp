/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

namespace StartupIntro {

enum class Phase : uint8_t { kRocketFlight, kCloudTransition, kLogoDescent, kLogoSettle, kLogoHold, kComplete };

inline constexpr uint32_t kFrameIntervalMs = 50;
inline constexpr uint32_t kRocketFlightEndMs = 1200;
inline constexpr uint32_t kLogoDescentStartMs = 1400;
inline constexpr uint32_t kLogoDescentEndMs = 2850;
inline constexpr uint32_t kLogoSettleEndMs = 3150;
inline constexpr uint32_t kDurationMs = 3500;
inline constexpr uint32_t kStaticLogoDurationMs = 2000;
inline constexpr uint32_t kMaximumAllowedDurationMs = 5000;

[[nodiscard]] constexpr Phase PhaseAt(uint32_t elapsedMs) {
  if (elapsedMs < kRocketFlightEndMs) {
    return Phase::kRocketFlight;
  }
  if (elapsedMs < kLogoDescentStartMs) {
    return Phase::kCloudTransition;
  }
  if (elapsedMs < kLogoDescentEndMs) {
    return Phase::kLogoDescent;
  }
  if (elapsedMs < kLogoSettleEndMs) {
    return Phase::kLogoSettle;
  }
  if (elapsedMs < kDurationMs) {
    return Phase::kLogoHold;
  }
  return Phase::kComplete;
}

[[nodiscard]] constexpr const char* PhaseName(Phase phase) {
  switch (phase) {
    case Phase::kRocketFlight:
      return "rocket_flight";
    case Phase::kCloudTransition:
      return "cloud_transition";
    case Phase::kLogoDescent:
      return "logo_descent";
    case Phase::kLogoSettle:
      return "logo_settle";
    case Phase::kLogoHold:
      return "logo_hold";
    case Phase::kComplete:
      return "complete";
  }
  return "complete";
}

static_assert(kDurationMs <= kMaximumAllowedDurationMs, "The startup intro must complete within five seconds");
static_assert(kDurationMs % kFrameIntervalMs == 0, "The startup duration must end on an animation frame");
static_assert(kStaticLogoDurationMs <= kMaximumAllowedDurationMs,
              "The static startup must complete within five seconds");

}  // namespace StartupIntro

/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <vqf.hpp>

// GS angles in radians. North retains the navigation/recovery convention:
// NWU yaw, zero along magnetic north, positive counterclockwise. Pitch is
// positive when pointing up, negative when pointing down; roll retains the
// existing right-hand rotation about the aligned IMU X axis.
struct Attitude {
  float north = 0;
  float pitch = 0;
  float roll = 0;
};

// Owned by the navigation task. Input vectors use native sensor axes:
// factory-corrected gyro in degrees/s, acceleration in g, calibrated magnetic
// field in arbitrary units. Calibration must precede the board-axis mapping.
class AttitudeFilter {
 public:
  static constexpr unsigned sampleRateHz = 50;
  Attitude update(const std::array<float, 3> &gyro, const std::array<float, 3> &acceleration,
                  const std::array<float, 3> &magnetic);
  void reset() { filter.resetState(); }

 private:
  VQF filter{1.0 / sampleRateHz};
};

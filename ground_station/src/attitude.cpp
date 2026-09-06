/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "attitude.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

Attitude AttitudeFilter::update(const std::array<float, 3> &gyro, const std::array<float, 3> &acceleration,
                                const std::array<float, 3> &magnetic) {
  constexpr double pi = std::numbers::pi;
  constexpr double radiansPerDegree = pi / 180.0;
  constexpr double gravity = 9.80665;
  // Preserve the board mounting used by the previous estimator. Both mappings
  // are proper rotations (right-handed); VQF requires rad/s and m/s^2.
  const vqf_real_t gyr[3]{static_cast<double>(gyro[1]) * radiansPerDegree,
                          static_cast<double>(gyro[0]) * radiansPerDegree,
                          -static_cast<double>(gyro[2]) * radiansPerDegree};
  const vqf_real_t acc[3]{static_cast<double>(acceleration[1]) * gravity,
                          static_cast<double>(acceleration[0]) * gravity,
                          -static_cast<double>(acceleration[2]) * gravity};
  const vqf_real_t mag[3]{-magnetic[1], -magnetic[0], -magnetic[2]};
  const auto finite = [](const auto &values) {
    return std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); });
  };
  if (finite(gyro)) filter.updateGyr(gyr);
  if (finite(acceleration)) filter.updateAcc(acc);
  if (finite(magnetic)) filter.updateMag(mag);

  vqf_real_t q[4];
  filter.getQuat9D(q);  // Scalar-first quaternion, sensor to ENU earth frame.
  const auto [w, x, y, z] = q;
  const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return {
      // ENU north is +Y. Subtract its quarter-turn to retain NWU yaw.
      .north = static_cast<float>(std::remainder(yaw - pi / 2, 2 * pi)),
      // Pointing up was negative with the old Euler pitch getter. Define the
      // GS sign here once, for every consumer, rather than in screen drawing.
      .pitch = static_cast<float>(-std::asin(std::clamp(2 * (w * y - x * z), -1.0, 1.0))),
      .roll = static_cast<float>(std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
  };
}

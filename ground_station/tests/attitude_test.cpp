// Copyright (C) 2026 Control and Telemetry Systems GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#include "attitude.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numbers>

namespace {
constexpr double rad = std::numbers::pi / 180;
using Vector = std::array<float, 3>;

void nearAngle(float actual, double expectedDegrees, const char *label, double tolerance = 0.1) {
  const double error = std::remainder(actual / rad - expectedDegrees, 360.0);
  if (!std::isfinite(actual) || std::abs(error) > tolerance) {
    std::cerr << label << ": expected " << expectedDegrees << ", got " << actual / rad << " degrees\n";
    std::exit(1);
  }
}

// Independent rotation-matrix fixture. Heading is NWU yaw; elevation is the
// user-facing positive-up angle. Return sensor-native gravity and magnetic
// measurements for the existing board mounting, without calling adapter math.
struct Pose {
  Vector acc;
  Vector mag;
  Pose(double heading, double elevation, double roll) {
    const double cy = std::cos(heading * rad), sy = std::sin(heading * rad);
    const double cp = std::cos(-elevation * rad), sp = std::sin(-elevation * rad);
    const double cr = std::cos(roll * rad), sr = std::sin(roll * rad);
    const double r00 = cy * cp, r01 = cy * sp * sr - sy * cr, r02 = cy * sp * cr + sy * sr;
    const double r20 = -sp, r21 = cp * sr, r22 = cp * cr;
    acc = {static_cast<float>(r21), static_cast<float>(r20), static_cast<float>(-r22)};
    // Magnetic north has a horizontal component of 40 and vertical of -20.
    mag = {static_cast<float>(-40 * r01 + 20 * r21), static_cast<float>(-40 * r00 + 20 * r20),
           static_cast<float>(-40 * r02 + 20 * r22)};
  }
};

Attitude settle(AttitudeFilter &filter, const Pose &pose, const Vector &gyro = {}, unsigned count = 500) {
  Attitude result;
  for (unsigned i = 0; i < count; ++i) result = filter.update(gyro, pose.acc, pose.mag);
  return result;
}

void staticPoses() {
  for (double heading : {0., 45., 90., 135., 180., -135., -90., -45.}) {
    for (double elevation : {-60., -30., 0., 30., 60.}) {
      for (double roll : {-35., 0., 35.}) {
        AttitudeFilter filter;
        const auto result = settle(filter, Pose(heading, elevation, roll));
        nearAngle(result.north, heading, "tilt-compensated heading");
        nearAngle(result.pitch, elevation, "positive-up pitch");
        nearAngle(result.roll, roll, "roll");
      }
    }
  }
  for (double elevation : {-90., 90.}) {
    AttitudeFilter filter;
    const auto result = settle(filter, Pose(0, elevation, 0));
    nearAngle(result.pitch, elevation, "vertical pitch");
    if (!std::isfinite(result.north) || !std::isfinite(result.roll)) std::exit(1);
  }
}

void gyroUnitsAndAxes() {
  const Pose level(0, 0, 0);
  // Raw native gyro rotations: Z inverted -> yaw; X inverted -> elevation;
  // native Y -> roll. No accelerometer/magnetometer correction during motion.
  for (unsigned axis = 0; axis < 3; ++axis) {
    AttitudeFilter filter;
    settle(filter, level);
    Vector gyro{};
    gyro[axis] = axis == 1 ? 30 : -30;
    Attitude result;
    for (unsigned i = 0; i < AttitudeFilter::sampleRateHz; ++i) result = filter.update(gyro, {}, {});
    nearAngle(result.pitch, axis == 0 ? 30 : 0, "gyro pitch");
    nearAngle(result.roll, axis == 1 ? 30 : 0, "gyro roll");
    nearAngle(result.north, axis == 2 ? 30 : 0, "gyro heading");
  }
}

void restBiasAndReset() {
  AttitudeFilter filter;
  const Pose pose(40, 20, -15);
  // Existing stored factory offsets have already been removed. VQF must learn
  // this remaining 0.5 deg/s offset, including the otherwise unobservable yaw.
  const Vector bias{0, 0, -0.5F};
  const auto before = settle(filter, pose, bias, 3000);
  Attitude after;
  for (unsigned i = 0; i < 500; ++i) after = filter.update(bias, pose.acc, {});
  nearAngle(after.north, before.north / rad, "rest bias with missing magnetometer", 0.2);
  nearAngle(after.pitch, 20, "rest pitch", 0.2);
  filter.reset();  // New calibration must discard the old learned offset.
  const auto reset = settle(filter, Pose(-90, -30, 20));
  nearAngle(reset.north, -90, "reset heading");
  nearAngle(reset.pitch, -30, "reset pitch");
  nearAngle(reset.roll, 20, "reset roll");
}

void magneticDisturbance() {
  AttitudeFilter filter;
  settle(filter, Pose(0, 0, 0));
  // VQF requires movement to establish a trusted magnetic reference. Rotate
  // at 45 deg/s for two turns, then stop at north before applying interference.
  for (unsigned i = 1; i <= 800; ++i) {
    const Pose pose(i * 45.0 / AttitudeFilter::sampleRateHz, 0, 0);
    filter.update({0, 0, -45}, pose.acc, pose.mag);
  }
  const auto before = settle(filter, Pose(0, 0, 0));
  Pose disturbed(90, 0, 0);
  for (auto &value : disturbed.mag) value *= 3;
  Attitude result;
  for (unsigned i = 0; i < 250; ++i) result = filter.update({}, disturbed.acc, disturbed.mag);
  nearAngle(result.north, before.north / rad, "short magnetic disturbance rejection", 0.5);
  nearAngle(result.pitch, 0, "magnetic disturbance pitch");
  nearAngle(settle(filter, Pose(0, 0, 0)).north, 0, "magnetic recovery", 0.5);
}

void invalidInputs() {
  AttitudeFilter filter;
  const Pose pose(45, 30, 20);
  settle(filter, pose);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  for (unsigned i = 0; i < 100; ++i) filter.update({nan, 0, 0}, {0, inf, 0}, {0, 0, nan});
  const auto result = settle(filter, pose);
  nearAngle(result.north, 45, "invalid input recovery heading");
  nearAngle(result.pitch, 30, "invalid input recovery pitch");
}
}  // namespace

int main() {
  staticPoses();
  gyroUnitsAndAxes();
  restBiasAndReset();
  magneticDisturbance();
  invalidInputs();
  std::cout << "VQF attitude tests passed: 120 poses, vertical limits, gyro units/axes, bias, reset, magnetic disturbance, invalid input.\n"
            << "AttitudeFilter storage: " << sizeof(AttitudeFilter) << " bytes\n";
}

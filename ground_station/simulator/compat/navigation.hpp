#pragma once

#include "utils.hpp"

struct EarthPoint3D {
  float lat = 0.0F;
  float lon = 0.0F;
  float alt = 0.0F;
};

class Navigation {
 public:
  void setPointA(float lat, float lon, float alt = 0.0F) { pointA_ = {lat, lon, alt}; }
  void setPointB(float lat, float lon, float alt = 0.0F) { pointB_ = {lat, lon, alt}; }
  void setOrientation(float north, float azimuth, float elevation) {
    north_ = north;
    azimuth_ = azimuth;
    elevation_ = elevation;
  }
  void setDistance(float distance) { distance_ = distance; }
  void setAcceleration(float x, float y, float z) { ax_ = x; ay_ = y; az_ = z; }
  void setGyroscope(float x, float y, float z) { gx_ = x; gy_ = y; gz_ = z; }
  void setMagnetometer(float x, float y, float z) { mx_ = x; my_ = y; mz_ = z; }
  void setCalibrationPercentage(float percentage) { calibrationPercentage_ = percentage; }

  [[nodiscard]] EarthPoint3D getPointA() const { return pointA_; }
  [[nodiscard]] EarthPoint3D getPointB() const { return pointB_; }
  float getNorth() { return north_; }
  float getAzimuth() { return azimuth_; }
  float getElevation() { return elevation_; }
  float getDistance() { return distance_; }
  float computeBearing() { return (azimuth_ + north_ - PI_F / 2.0F) / (2.0F * PI_F / 360.0F); }
  [[nodiscard]] float getAX() const { return ax_; }
  [[nodiscard]] float getAY() const { return ay_; }
  [[nodiscard]] float getAZ() const { return az_; }
  [[nodiscard]] float getGX() const { return gx_; }
  [[nodiscard]] float getGY() const { return gy_; }
  [[nodiscard]] float getGZ() const { return gz_; }
  [[nodiscard]] float getMX() const { return mx_; }
  [[nodiscard]] float getMY() const { return my_; }
  [[nodiscard]] float getMZ() const { return mz_; }
  [[nodiscard]] float getCalibrationPercentage() const { return calibrationPercentage_; }

 private:
  EarthPoint3D pointA_{};
  EarthPoint3D pointB_{};
  float north_ = 0.0F;
  float azimuth_ = 0.0F;
  float elevation_ = 0.0F;
  float distance_ = 0.0F;
  float ax_ = 0.0F;
  float ay_ = 0.0F;
  float az_ = 1.0F;
  float gx_ = 0.0F;
  float gy_ = 0.0F;
  float gz_ = 0.0F;
  float mx_ = 0.0F;
  float my_ = 0.0F;
  float mz_ = 0.0F;
  float calibrationPercentage_ = 0.0F;
};

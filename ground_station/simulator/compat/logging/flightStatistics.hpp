#pragma once

#include <cstdint>

class FlightStatistics {
 public:
  void set(int32_t maxAltitude, float timeToApogee, int32_t maxVelocity, float drogueRate, float mainRate,
           float latitude, float longitude, float flightTime) {
    maxAltitude_ = maxAltitude;
    timeToApogee_ = timeToApogee;
    maxVelocity_ = maxVelocity;
    drogueRate_ = drogueRate;
    mainRate_ = mainRate;
    latitude_ = latitude;
    longitude_ = longitude;
    flightTime_ = flightTime;
  }
  [[nodiscard]] int32_t getMaxAltitude() const { return maxAltitude_; }
  [[nodiscard]] float getTimeToApogee() const { return timeToApogee_; }
  [[nodiscard]] int32_t getMaxVelocity() const { return maxVelocity_; }
  [[nodiscard]] float getDrogueDescentRate() const { return drogueRate_; }
  [[nodiscard]] float getMainDescentRate() const { return mainRate_; }
  [[nodiscard]] float getLastLatitude() const { return latitude_; }
  [[nodiscard]] float getLastLongitude() const { return longitude_; }
  [[nodiscard]] float getFlightTime() const { return flightTime_; }

 private:
  int32_t maxAltitude_ = 0;
  float timeToApogee_ = 0.0F;
  int32_t maxVelocity_ = 0;
  float drogueRate_ = 0.0F;
  float mainRate_ = 0.0F;
  float latitude_ = 0.0F;
  float longitude_ = 0.0F;
  float flightTime_ = 0.0F;
};

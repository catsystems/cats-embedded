#pragma once

#include <cmath>
#include <cstdint>

class FlightStatistics {
 public:
  void set(int32_t maxAltitude, float timeToApogee, int32_t maxVelocity, float drogueRate, float mainRate,
           float latitude, float longitude, float flightTime, bool maxAltitudeValid = true,
           bool timeToApogeeValid = true, bool maxVelocityValid = true, bool drogueRateValid = true,
           bool mainRateValid = true, bool lastLocationValid = true, bool flightTimeValid = true) {
    maxAltitude_ = maxAltitude;
    timeToApogee_ = timeToApogee;
    maxVelocity_ = maxVelocity;
    drogueRate_ = drogueRate;
    mainRate_ = mainRate;
    latitude_ = latitude;
    longitude_ = longitude;
    flightTime_ = flightTime;
    maxAltitudeValid_ = maxAltitudeValid;
    timeToApogeeValid_ = timeToApogeeValid;
    maxVelocityValid_ = maxVelocityValid;
    drogueRateValid_ = drogueRateValid;
    mainRateValid_ = mainRateValid;
    lastLocationValid_ = lastLocationValid;
    flightTimeValid_ = flightTimeValid;
  }
  [[nodiscard]] int32_t getMaxAltitude() const { return maxAltitude_; }
  [[nodiscard]] float getTimeToApogee() const { return timeToApogee_; }
  [[nodiscard]] int32_t getMaxVelocity() const { return maxVelocity_; }
  [[nodiscard]] float getDrogueDescentRate() const { return drogueRate_; }
  [[nodiscard]] float getMainDescentRate() const { return mainRate_; }
  [[nodiscard]] float getLastLatitude() const { return latitude_; }
  [[nodiscard]] float getLastLongitude() const { return longitude_; }
  [[nodiscard]] bool hasLastLocation() const {
    return lastLocationValid_ && std::isfinite(latitude_) && std::isfinite(longitude_) &&
           (latitude_ != 0.0F || longitude_ != 0.0F) && latitude_ >= -90.0F && latitude_ <= 90.0F &&
           longitude_ >= -180.0F && longitude_ <= 180.0F;
  }
  [[nodiscard]] float getFlightTime() const { return flightTime_; }
  [[nodiscard]] bool hasMaxAltitude() const { return maxAltitudeValid_; }
  [[nodiscard]] bool hasTimeToApogee() const { return timeToApogeeValid_; }
  [[nodiscard]] bool hasMaxVelocity() const { return maxVelocityValid_; }
  [[nodiscard]] bool hasDrogueDescentRate() const { return drogueRateValid_; }
  [[nodiscard]] bool hasMainDescentRate() const { return mainRateValid_; }
  [[nodiscard]] bool hasFlightTime() const { return flightTimeValid_; }

 private:
  int32_t maxAltitude_ = 0;
  float timeToApogee_ = 0.0F;
  int32_t maxVelocity_ = 0;
  float drogueRate_ = 0.0F;
  float mainRate_ = 0.0F;
  float latitude_ = 0.0F;
  float longitude_ = 0.0F;
  float flightTime_ = 0.0F;
  bool maxAltitudeValid_ = false;
  bool timeToApogeeValid_ = false;
  bool maxVelocityValid_ = false;
  bool drogueRateValid_ = false;
  bool mainRateValid_ = false;
  bool lastLocationValid_ = false;
  bool flightTimeValid_ = false;
};

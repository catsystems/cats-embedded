#pragma once

#include <cstdint>

inline constexpr float PI_F = 3.14159265358979323846F;

class Utils {
 public:
  static void startBootloader() {}

  template <typename T>
  [[nodiscard]] constexpr static T MetersToFeet(T meters) {
    return static_cast<T>(meters * static_cast<T>(3.28084));
  }
};

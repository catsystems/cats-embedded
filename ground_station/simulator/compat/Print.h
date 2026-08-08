#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

class Print {
 public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t value) = 0;
  virtual size_t write(const uint8_t* buffer, size_t size) {
    for (size_t i = 0; i < size; ++i) write(buffer[i]);
    return size;
  }

  size_t print(const char* value) { return write(reinterpret_cast<const uint8_t*>(value), std::char_traits<char>::length(value)); }
  size_t print(const std::string& value) { return print(value.c_str()); }
  size_t print(char value) { return write(static_cast<uint8_t>(value)); }
  size_t print(int value) { return print(std::to_string(value)); }
  size_t print(unsigned int value) { return print(std::to_string(value)); }
  size_t print(long value) { return print(std::to_string(value)); }
  size_t print(unsigned long value) { return print(std::to_string(value)); }
  size_t print(float value) { return print(std::to_string(value)); }
  size_t print(float value, int digits) {
    char buffer[64] = {};
    std::snprintf(buffer, sizeof(buffer), "%.*f", digits, static_cast<double>(value));
    return print(buffer);
  }
  size_t print(double value, int digits) {
    char buffer[64] = {};
    std::snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
    return print(buffer);
  }
  size_t println() { return print("\n"); }
  template <typename T>
  size_t println(const T& value) { return print(value) + println(); }
};

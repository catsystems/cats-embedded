#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#define ARDUINO 100
#define PROGMEM
#define radians(degrees) ((degrees) * 0.017453292519943295769236907684886)
using boolean = bool;
using byte = uint8_t;

class __FlashStringHelper {};

class String {
 public:
  String() = default;
  String(const char* value) : value_(value == nullptr ? "" : value) {}
  String(const std::string& value) : value_(value) {}
  String(int value) : value_(std::to_string(value)) {}
  String(unsigned int value) : value_(std::to_string(value)) {}
  String(long value) : value_(std::to_string(value)) {}
  String(unsigned long value) : value_(std::to_string(value)) {}
  String(float value) : value_(std::to_string(value)) {}

  [[nodiscard]] size_t length() const { return value_.size(); }
  [[nodiscard]] const char* c_str() const { return value_.c_str(); }
  operator const char*() const { return value_.c_str(); }
  String& operator+=(char value) { value_ += value; return *this; }
  String& operator+=(const String& value) { value_ += value.value_; return *this; }

 private:
  std::string value_;
};

inline String operator+(const char* left, const String& right) { return String(std::string(left) + right.c_str()); }

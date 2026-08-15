/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

// Include Nayuki before Arduino/Adafruit headers, which define a LOW macro that
// otherwise collides with QrCode::Ecc::LOW.
#include <qrcodegen.hpp>

#include "location_qr.hpp"

#include <cmath>
#include <cstring>
#include <exception>
#include <vector>

namespace LocationQr {
namespace {

constexpr int kMinQrVersion = 4;
constexpr int kMaxQrVersion = 5;
constexpr int16_t kVersion4ModuleSize = 4;
constexpr int16_t kVersion5ModuleSize = 3;
constexpr int16_t kQuietZoneModules = 4;
constexpr int16_t kVersion4Extent = (33 + 2 * kQuietZoneModules) * kVersion4ModuleSize;
constexpr char kGoogleMapsPrefix[] = "https://www.google.com/maps/search/?api=1&query=";

bool FormatCoordinate(float coordinate, char *text, size_t textSize) {
  const auto scaled = static_cast<int32_t>(lroundf(coordinate * 10000.0F));
  const bool negative = scaled < 0;
  const uint32_t magnitude =
      negative ? static_cast<uint32_t>(-static_cast<int64_t>(scaled)) : static_cast<uint32_t>(scaled);
  uint32_t whole = magnitude / 10000U;
  const uint32_t fraction = magnitude % 10000U;

  char reversed[3] = {};
  size_t digits = 0;
  do {
    reversed[digits++] = static_cast<char>('0' + whole % 10U);
    whole /= 10U;
  } while (whole != 0U);

  const size_t required = (negative ? 1U : 0U) + digits + 1U + 4U + 1U;
  if (text == nullptr || textSize < required) {
    return false;
  }

  size_t position = 0;
  if (negative) {
    text[position++] = '-';
  }
  while (digits > 0) {
    text[position++] = reversed[--digits];
  }
  text[position++] = '.';
  text[position++] = static_cast<char>('0' + (fraction / 1000U) % 10U);
  text[position++] = static_cast<char>('0' + (fraction / 100U) % 10U);
  text[position++] = static_cast<char>('0' + (fraction / 10U) % 10U);
  text[position++] = static_cast<char>('0' + fraction % 10U);
  text[position] = '\0';
  return true;
}

}  // namespace

bool IsValid(float latitude, float longitude) {
  return std::isfinite(latitude) && std::isfinite(longitude) && (latitude != 0.0F || longitude != 0.0F) &&
         latitude >= -90.0F && latitude <= 90.0F && longitude >= -180.0F && longitude <= 180.0F;
}

bool BuildGoogleMapsUrl(float latitude, float longitude, char *url, size_t urlSize) {
  if (!IsValid(latitude, longitude) || url == nullptr || urlSize == 0) {
    return false;
  }

  char latitudeText[12] = {};
  char longitudeText[13] = {};
  if (!FormatCoordinate(latitude, latitudeText, sizeof(latitudeText)) ||
      !FormatCoordinate(longitude, longitudeText, sizeof(longitudeText))) {
    return false;
  }

  const size_t prefixLength = sizeof(kGoogleMapsPrefix) - 1U;
  const size_t latitudeLength = strlen(latitudeText);
  const size_t longitudeLength = strlen(longitudeText);
  const size_t totalLength = prefixLength + latitudeLength + 3U + longitudeLength;
  if (totalLength + 1U > urlSize) {
    return false;
  }

  size_t position = 0;
  memcpy(url + position, kGoogleMapsPrefix, prefixLength);
  position += prefixLength;
  memcpy(url + position, latitudeText, latitudeLength);
  position += latitudeLength;
  memcpy(url + position, "%2C", 3U);
  position += 3U;
  memcpy(url + position, longitudeText, longitudeLength);
  position += longitudeLength;
  url[position] = '\0';
  return true;
}

bool DrawGoogleMapsQr(Adafruit_GFX &display, float latitude, float longitude, int16_t x, int16_t y, uint16_t black,
                      uint16_t white) {
  char url[kGoogleMapsUrlSize] = {};
  if (!BuildGoogleMapsUrl(latitude, longitude, url, sizeof(url))) {
    return false;
  }

  try {
    const size_t prefixLength = sizeof(kGoogleMapsPrefix) - 1U;
    const auto prefixBytes = std::vector<uint8_t>(url, url + prefixLength);
    const auto segments = std::vector<qrcodegen::QrSegment>{
        qrcodegen::QrSegment::makeBytes(prefixBytes),
        qrcodegen::QrSegment::makeAlphanumeric(url + prefixLength),
    };
    const auto qr = qrcodegen::QrCode::encodeSegments(segments, qrcodegen::QrCode::Ecc::MEDIUM, kMinQrVersion,
                                                      kMaxQrVersion, -1, false);

    const int16_t moduleSize = qr.getSize() == 33 ? kVersion4ModuleSize : kVersion5ModuleSize;
    const auto qrExtent = static_cast<int16_t>((qr.getSize() + 2 * kQuietZoneModules) * moduleSize);
    const auto renderX = static_cast<int16_t>(x + (kVersion4Extent - qrExtent) / 2);
    display.fillRect(renderX, y, qrExtent, qrExtent, white);
    for (int moduleY = 0; moduleY < qr.getSize(); ++moduleY) {
      for (int moduleX = 0; moduleX < qr.getSize(); ++moduleX) {
        if (qr.getModule(moduleX, moduleY)) {
          display.fillRect(static_cast<int16_t>(renderX + (moduleX + kQuietZoneModules) * moduleSize),
                           static_cast<int16_t>(y + (moduleY + kQuietZoneModules) * moduleSize), moduleSize, moduleSize,
                           black);
        }
      }
    }
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

}  // namespace LocationQr

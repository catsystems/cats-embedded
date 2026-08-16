#pragma once

#include <cstddef>
#include <cstdint>

#include "display.hpp"

// WebAssembly uses the same one-bit Adafruit canvas as the production renderer.
// Framebuffer access remains simulator-only and never enters the firmware API.
class SimulatorDisplay final : public IDisplay {
 public:
  SimulatorDisplay() : canvas_(400, 240) {}

  Adafruit_GFX& gfx() override { return canvas_; }
  void begin() override { canvas_.setRotation(0); }
  void clear() override { canvas_.fillScreen(1); }
  void clearBuffer() override { canvas_.fillScreen(1); }
  void present() override { ++frameRevision_; }

  [[nodiscard]] uint32_t revision() const { return frameRevision_; }
  [[nodiscard]] const uint8_t* framebuffer() const { return canvas_.getBuffer(); }
  [[nodiscard]] size_t framebufferSize() const { return (400U * 240U + 7U) / 8U; }

 private:
  GFXcanvas1 canvas_;
  uint32_t frameRevision_ = 0;
};

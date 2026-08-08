#pragma once

#include "hmi_controller.hpp"
#include "simulator_display.hpp"
#include "window.hpp"

class WindowHmiRenderer final : public IHmiRenderer {
 public:
  WindowHmiRenderer(SimulatorDisplay& display, IClock& clock);

  void begin() override;
  void render(const HmiSnapshot& state) override;
  [[nodiscard]] uint32_t revision() const override { return display_.revision(); }

 private:
  static void syncConfiguration(const GsConfigSnapshot& source);
  void syncNavigation(const NavigationSnapshot& source);
  void syncLink(const LinkSnapshot& source, size_t index);
  void syncStatistics(const FlightStatisticsSnapshot& source, size_t index);
  void drawStatusBar(const DeviceStatusSnapshot& status);

  SimulatorDisplay& display_;
  Window window_;
  Navigation navigation_;
  TelemetryData telemetry_[2]{};
  TelemetryInfo linkInfo_[2]{};
  FlightStatistics statistics_[2]{};
  char keyboardText_[kMaxPhraseLen + 1]{};
};

// Copyright (C) 2026 Control and Telemetry Systems GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

// Hardware stand-ins for compiling the real Hmi::live/initLive/update bodies.
// The test runner extracts those bodies verbatim; no navigation logic is copied.
#pragma once
#include "logging/recorder.hpp"
#include <string>

struct HostButton {
  bool pressed = false;
  bool wasPressed() { bool result = pressed; pressed = false; return result; }
  void read() {}
};
struct HostLink {
  struct Datum {
    bool isUpdated() { return false; }
    bool isValid() { return false; }
    int hour() { return 0; }
    int minute() { return 0; }
    int second() { return 0; }
  } data, info, time, location;
};
inline HostLink link1, link2;
inline int navigation;
struct Window {
  enum class LiveState { kShowGnss, kShowDownRange };
  std::string message;
  void initDataMessage(const char* title, const char*) { message = title; }
  void initLiveConfirmExit() { message = "confirm"; }
  void initLive() { message = "live"; }
  void initMenu(int) { message = "menu"; }
  void refresh() {}
  template <typename... T> void updateLive(T...) {}
  template <typename... T> bool updateLiveNavigation(T...) { return false; }
  template <typename... T> void UpdateLiveState(T...) {}
  template <typename... T> void updateBar(T...) {}
  void initBar() {}
  void logo() {}
  void drawStartupIntroFrame(uint32_t) {}
};
namespace StartupIntro {
constexpr uint32_t kDurationMs = 0, kFrameIntervalMs = 0, kStaticLogoDurationMs = 0;
}
constexpr uint32_t kLiveRefreshIntervalMs = 200;
struct Hmi {
  enum State { MENU, LIVE, SELF_TEST };
  enum class LiveExitState { None, Confirm, StartFailed, StopFailed };
  explicit Hmi(Recorder& recorder) : recorder(recorder) {}
  Recorder& recorder;
  Window window;
  State state = LIVE;
  LiveExitState liveExitState = LiveExitState::None;
  HostButton upButton, downButton, leftButton, rightButton, centerButton, okButton, backButton;
  bool automaticUsbSharePending = false, initialized = true, isLogging = false;
  int menuIndex = 0, flashFreeMemory = 0;
  uint32_t lastLiveRefresh = 0;
  bool liveButtonPending() const { return false; }
  void initMenu() {} // Preserve the test's chosen screen for the update-loop probe.
  void fsm() { host::clockOffset += 1000; initialized = false; }
  void initLive();
  void live();
  static void update(void*);
};

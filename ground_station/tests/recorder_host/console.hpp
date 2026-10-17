// Copyright (C) 2026 Control and Telemetry Systems GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
struct HostConsole {
  struct Output {
    void print(const char*) {}
    void println(const char*) {}
  } warning;
};
inline HostConsole console;

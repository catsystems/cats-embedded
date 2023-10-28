/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "comm/stream.hpp"

/**
 * A wrapper for a pair of input and output streams.
 */
struct stream_group_t {
  const stream_t *in;
  const stream_t *out;
};

extern const stream_group_t USB_SG;

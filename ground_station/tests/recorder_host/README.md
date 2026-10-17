<!-- Copyright (C) 2026 Control and Telemetry Systems GmbH -->
<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Recorder and filesystem regression test

Run after PlatformIO has installed the pinned Ground Station dependencies:

```sh
python3 ground_station/tests/recorder_host/run.py
```

Requires Linux (or WSL), Python 3, GCC/G++, and AddressSanitizer/UndefinedBehaviorSanitizer.
Generated files and results belong under `ground_station/.pio/recorder-host/`.

This compiles the production `src/logging/recorder.cpp`, its real packet and configuration
headers, pinned SdFat sources, and pinned `Adafruit_FlashCache.cpp`. The recorder runs on
a worker thread with host queue/semaphore adapters. Flash erase/program operations use
an in-memory 960 KiB FAT12 volume formatted by the project's FatFs implementation.

The runner also compiles the production `Hmi::live`, `Hmi::initLive`, `Hmi::update`, and
`Utils::getFlashMemoryUsage` function bodies, extracted verbatim on every run. Display,
buttons, timing, and unrelated HMI work use stand-ins. This checks the actual exit and
free-space guards without introducing test hooks into firmware.

Coverage includes DOWN-only entry in both receiver modes and stop settings, mid-flight
segmentation, terminal samples, queued liftoff before exit, touchdown before exit-reply
delivery, cancellation, failed flush/retry, queue failure, and suppression outside Live.
Four existing logs and every generated log are checked by exact filename and complete
contents through fresh filesystem and flash-cache instances after each closed segment.
Three negative controls independently restore the old DOWN, exit, and free-space behavior;
each must fail its corresponding assertion.

This does not emulate ESP32 FreeRTOS scheduling, physical flash failures, LCD/GPIO timing,
or Windows USB MSC. USB handoff/reclaim is a synchronous adapter that flushes, unmounts,
and remounts the actual filesystem. Physical qualification remains necessary.

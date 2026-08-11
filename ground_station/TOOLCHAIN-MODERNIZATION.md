# Ground Station modernization status

## Implemented pins

- PlatformIO Core 6.1.19 on Python 3.11
- Tasmota platform-espressif32 2026.05.50
- Arduino-ESP32 3.3.8 (`f2a3fa2b`) and GCC 15.2
- Arduino-ESP32 native TinyUSB CDC/MSC/runtime-DFU support, SPIFlash 5.1.1,
  SdFat 2.3.103
- ArduinoJson 7.4.3, JC_Button 2.1.6, Time 1.6.1
- Adafruit GFX 1.12.6, BusIO 1.17.4, Sharp Memory Display 1.1.4
- Arduino_LSM6DS3 1.0.3 and Madgwick 1.2.0 with documented CATS deltas
- FatFs R0.16 with upstream patch-2 (July 10, 2026)
- Emscripten 6.0.6 at emsdk commit `9981799f744be74ac67b1c1813ff172f63be0630`

`QMC5883Compass` is protected by a checked manifest and is not upgraded or
modified. The two TinyUF2 0.10.2 combined images are also hash-locked.

The standalone Adafruit TinyUSB library is intentionally not installed. It
uses a second descriptor registry that is incompatible with manual USB startup
in Arduino-ESP32 3.3.8. USB CDC and MSC now use the pinned core directly. The
documented `USB.cpp` override selects runtime DFU when the core configuration
enables both DFU variants and retains the TinyUF2 reset hint `0x11F2`.

## Automated result

The clean Windows build uses 543,680 bytes of flash (37.7%) and 63,928 bytes
of static RAM (19.5%). Relative to the recorded baseline, flash is 19,926
bytes smaller and static RAM is 23,980 bytes larger. The RAM increase is
accounted for by fixed MSC, transfer, and endpoint buffers in the pinned
core's precompiled native USB implementation; 263,752 bytes (80.5%) remain
available. This intentionally replaces the incompatible mixed core/Adafruit
USB descriptor registries that failed during hardware testing. All 12
deterministic simulator scenarios pass with the unchanged
framebuffer SHA-256
`4d81f455bcf9e62e7489cd6eb5bb939be6e54f7f89ab32523e0cd5b0b48d03d2`,
and the Emscripten WebAssembly bundle builds successfully. The host storage
test mounts and updates an R0.13c-generated volume, creates and remounts an
R0.16 volume, and rejects corrupted media without issuing a write.

Run `python ground_station/check_upgrade_contract.py` before builds to verify
the dependency pins, USB identity, protected compass library, and bootloader
images.

## Hardware release gate

Do not release this firmware until a production Ground Station passes the
complete hardware gate: three old/new/old UF2
cycles; configuration and log retention; cold boot and ten power cycles; 25
USB reconnects; CDC, MSC, DFU, manual bootloader, and failed-update recovery;
blank/corrupt storage and formatting; a 60-minute logging run; display and QR
inspection; IMU/compass calibration and known rotations; and 100 JSON
load/save cycles without continuing heap loss.

The FatFs rebase preserves the existing `ffconf` behavior, 512-byte disk
callbacks, code page, super-floppy layout, volume label, and explicit format
workflow. A normal upgrade does not invoke the formatter merely because the
vendored FatFs version changed. The TinyUF2 images are not part of this
migration.

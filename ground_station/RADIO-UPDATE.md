# Ground Station receiver firmware update

The Ground Station can install one production STM32G071 telemetry application on both internal receiver MCUs. It uses
the STM32 factory system-memory bootloader over each receiver's existing UART; the ordinary telemetry protocol and the
application flash address (`0x08000000`) are unchanged.

## Prepare and run an update

1. Build the production `telemetry` PlatformIO environment and take its `firmware.bin`. Keep its SHA-256 with the
   release record as an informational artifact.
2. Connect the Ground Station USB drive, create `/telemetry_firmware`, and copy the raw `.bin` file into that directory.
   Up to 64 `.bin` files are listed alphabetically; subdirectories are not scanned.
3. Safely eject the USB drive from the computer. Keep the Ground Station powered and connected to a stable supply.
4. On the Ground Station, open **Settings > System > Update Firmware > Radio Receivers**, select the file, review the
   size and CRC32, and choose **Install**.
5. Do not disconnect power until both links report a verified application version.

The displayed CRC32 detects a file changing between selection and programming. A raw binary has no product identity,
expected version, signature, or downgrade metadata. The operator is responsible for selecting production telemetry
firmware intended for the Ground Station. Reinstalling the same version is allowed.

## Safety and recovery

The update is refused while a log is active, a radio is in test mode, or a recent radio packet reports an airborne
state. USB mass-storage ownership, logging, and both UARTs are handed over before programming. Link 1 is fully written,
read back, started, and version-checked before Link 2 is touched.

The updater checks the STM32 product ID, 128 KiB flash geometry, ROM revision B1-B4, and unprotected option bytes. It
erases individual 2 KiB pages, waits for the affected ROM revisions, writes in at most 256-byte aligned blocks, and
reads every byte back. The first eight vector-table bytes are written only after the rest of the image and its source
CRC are verified. It never changes option bytes or performs read/write unprotect.

An interrupted factory-ROM update has no rollback slot. Most interruptions leave the vector table erased and permit a
new ROM session, but an interruption during final vector activation can require ST-Link recovery. A link whose entry or
programming result is ambiguous is quarantined until Ground Station restart or repair. This is the unavoidable safety
limitation of using the factory ROM instead of a resident custom bootloader.

Receiver applications predating the guarded `CMD_BOOTLOADER` implementation need a one-time ST-Link installation of a
ROM-capable production telemetry build. Units provisioned with the earlier protected custom-loader proof of concept need
a separate provisioning procedure. Older Vega firmware remains compatible with the ROM-capable telemetry application.

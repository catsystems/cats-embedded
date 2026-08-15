# Ground Station TinyUF2 images

## Current release

The current bootloader artifacts come from Adafruit TinyUF2 `0.35.0`, tag
commit `8542b474ffa19c0dd66a8ccc99b1d6b8caff8d12`:

- `tinyuf2-espressif_saola_1_wroom-0.35.0.zip`, release SHA-256
  `f4fc81da5c1c8f4378e5c79dd6685c3d87ebccd701a7e1ecc930595568fa689e`;
- `tinyuf2-espressif_saola_1_wrover-0.35.0.zip`, release SHA-256
  `8df90daf8fca821f88304f6e9a94a61dec08b7468d923321668bffa679215d91`.

The checked-in `*_combined-ota.bin` files are the release ZIPs'
`combined-ota.bin` artifacts, renamed to retain the board and version. Do not
use the ZIPs' `combined.bin`: it contains one 2816 KiB application slot. The
OTA variants contain the Ground Station's existing two 1408 KiB application
slots, 256 KiB UF2 partition, and 960 KiB FAT partition. Their partition-table
bytes are identical to `partitions_custom.csv` and the 0.10.2 images.

Both images retain 4 MiB flash, DIO mode at 80 MHz, the Saola WROOM/WROVER
board definitions, and application entry through reset hint `0x11F2`.

## Which artifact to use

- `update-tinyuf2-*.uf2` is for an existing TinyUF2 drive. It temporarily
  replaces the Ground Station application with the updater, updates only the
  factory UF2 application, and then opens TinyUF2 again. Reinstall the Ground
  Station application UF2 immediately afterward. It does not replace the
  second-stage ESP bootloader or partition table.
- `*_combined-ota.bin` is the complete 0.35.0 factory/recovery image. Flash it
  only through the ESP32-S2 ROM serial loader with wired recovery available.
  It includes the second-stage bootloader, OTA metadata, partition table, and
  factory UF2 application; it also overwrites the application slots and NVS.
  Reinstall the Ground Station application afterward.

Always select the WROOM or WROVER artifact that matches the hardware. A wrong
variant or interrupted complete-image flash can require wired recovery.

## Rollback

The two 0.10.2 combined images remain checked in as known-good rollback assets.
Do not remove them until 0.35.0 has passed production hardware qualification.

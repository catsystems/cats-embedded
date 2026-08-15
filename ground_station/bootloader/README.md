# Ground Station TinyUF2 updater

## Current release

The current bootloader artifacts come from Adafruit TinyUF2 `0.35.0`, tag
commit `8542b474ffa19c0dd66a8ccc99b1d6b8caff8d12`:

- `tinyuf2-espressif_saola_1_wroom-0.35.0.zip`, release SHA-256
  `f4fc81da5c1c8f4378e5c79dd6685c3d87ebccd701a7e1ecc930595568fa689e`;
- `tinyuf2-espressif_saola_1_wrover-0.35.0.zip`, release SHA-256
  `8df90daf8fca821f88304f6e9a94a61dec08b7468d923321668bffa679215d91`.

The checked-in `update-tinyuf2-*.uf2` files are the normal update artifacts for
Ground Stations that already run TinyUF2. Complete factory images are not kept
in this repository because they are not needed for normal application or
bootloader updates.

## Which artifact to use

- Copy the matching `update-tinyuf2-*.uf2` to an existing TinyUF2 drive. It
  temporarily replaces the Ground Station application with the updater,
  updates only the factory UF2 application, and then opens TinyUF2 again.
- Reinstall the Ground Station application UF2 immediately afterward. The
  updater does not replace the second-stage ESP bootloader or partition table.

Always select the WROOM or WROVER artifact that matches the hardware.

## Wired recovery

If a complete factory image is needed for wired recovery, download the matching
0.35.0 WROOM or WROVER release ZIP from
[Adafruit TinyUF2](https://github.com/adafruit/tinyuf2/releases/tag/0.35.0).
Use its `combined-ota.bin`; the plain `combined.bin` has a different partition
layout and is not compatible with the Ground Station application layout.

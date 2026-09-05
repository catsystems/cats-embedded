# Ground Station WROVER bootloader

## Current release

These artifacts are for the ESP32-S2 Saola WROVER module. They are unmodified
files from [Adafruit TinyUF2 0.35.0](https://github.com/adafruit/tinyuf2/releases/tag/0.35.0),
tag commit `8542b474ffa19c0dd66a8ccc99b1d6b8caff8d12`.

Source: [tinyuf2-espressif_saola_1_wrover-0.35.0.zip](https://github.com/adafruit/tinyuf2/releases/download/0.35.0/tinyuf2-espressif_saola_1_wrover-0.35.0.zip).
The ZIP SHA-256 matches the official release digest:
`8df90daf8fca821f88304f6e9a94a61dec08b7468d923321668bffa679215d91`.

## Which artifact to use

- `combined-ota-espressif_saola_1_wrover-0.35.0.bin` is the archive's
  `combined-ota.bin`, renamed to identify the board and version. Flash it at
  **`0x0`** through the ESP ROM bootloader for blank-board provisioning or wired
  recovery. Then install the Ground Station application UF2. This factory image
  replaces the bootloader, partition table and application regions, including
  NVS settings; it is not a normal application update.
- `update-tinyuf2-espressif_saola_1_wrover-0.35.0.uf2` is the archive's
  `update-tinyuf2.uf2`. Copy it to an existing TinyUF2 drive. It temporarily
  replaces the Ground Station application with the updater, updates only the
  factory UF2 application, and then opens TinyUF2 again. Reinstall the Ground
  Station application UF2 immediately afterward. This updater does not replace
  the second-stage ESP bootloader or partition table.

## Verification

- Factory BIN: 3,116,960 bytes; SHA-256
  `12f95fd2814589422b94d943d1a8218da1887fff2b74987d96143d6dd145e51a`.
- Updater UF2: 613,888 bytes; SHA-256
  `1bb601567b78adad890b68c4f99a6d572e57b3ada608988e11569798b6c0a9bb`.

The factory image's OTA partition table matches `../partitions_custom.csv`:
`ota_0` at `0x010000`, `ota_1` at `0x170000`, `uf2` at `0x2d0000`, and `ffat`
at `0x310000`. The plain `combined.bin` has a different partition layout and is
not compatible with the Ground Station application layout.

Both artifacts were verified against the release archive. Physical flashing
and bootloader recovery remain hardware checks.

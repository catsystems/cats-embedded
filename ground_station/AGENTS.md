# Ground Station contributor guide

These instructions apply to everything under `ground_station/`.

## Work narrowly

- Prefer the smallest localized change that fixes the observed behavior. The firmware has tightly coupled display, USB,
  storage, telemetry, and FreeRTOS timing; broad cleanup makes hardware regressions difficult to isolate.
- Keep commits buildable and self-contained. Upgrade one dependency family or change one behavior at a time, from lower
  risk to higher risk. Do not make merge commits.
- Provide a short description of the changes made by each commit, including what changed and why.
- Put Ground Station-specific documentation here, not in the repository-level `README.md`.
- Do not commit `.pio/`, generated UF2/bin/elf files, generated `compile_commands.json`, or generated simulator JS/WASM.

## Protected compatibility boundaries

- `lib/QMC5883Compass` is rebased on upstream 1.2.3 and contains documented CATS support for both QMC5883L and QMC5883P.
  Do not replace or update it as part of unrelated work. Any future rebase must be a separate commit that preserves the
  CATS delta, updates `lib/QMC5883Compass/CATS-VENDORING.md`, and validates both sensor variants on hardware.
- TinyUF2 0.35.0 is the current bootloader version. Any future bootloader migration must be a separate commit with updated
  provenance and contract checks, and must retain a tested wired recovery path.
- Preserve the partition table and normal-upgrade compatibility with existing configuration and log files. A firmware
  update must not require formatting storage.
- Preserve USB VID `0x239A`, PID `0x80AB`, manufacturer/product/serial behavior, CDC/MSC/DFU interfaces, and the TinyUF2
  reset hint `0x11F2` unless the task explicitly changes the USB contract.
- `src/USB.cpp` is a narrow override of Arduino-ESP32 3.3.8. Keep its delta limited to the runtime-DFU choice and UF2
  reset hint documented in `src/USB-VENDORING.md`. Do not add a local `USBCDC.cpp` or the standalone Adafruit TinyUSB
  library.
- Preserve the existing telemetry protocol and radio behavior unless a task explicitly authorizes a protocol change.
  The receiver MCU can survive an ESP reset, so link initialization must first disable it, configure it, repeat direction
  immediately before enable, and enable only links with a configured phrase.

## Storage and USB ownership

The FAT volume has exactly one owner at a time:

- Firmware-owned: the recorder and Ground Station UI may use filesystem APIs.
- Host-owned: USB MSC callbacks may access blocks; firmware code must not open, scan, delete, or format files.

Before sharing storage with the PC, finish filesystem work, flush it, and unmount it. Before recording or browsing logs on
the Ground Station, reclaim and remount it. Do not drop the first flight packet while ownership is changing.

Expected behavior is:

- Automatically share storage when USB is connected and the recorder is idle.
- Reclaim storage before recording starts, then share it again after finalization.
- Allow the user to leave the USB Drive screen and to disconnect the drive explicitly.
- Refuse deletion while the PC owns the drive. Log deletion from the PC is safe only while the firmware is not using the
  volume.
- Include all regular `.csv` logs in the catalog, including legacy `cats1_*.csv` names. Keep numbered `log_NNN.csv` files
  in descending numeric order and ignore non-CSV files.

Any storage change needs simulator coverage for idle sharing, recording reclaim, deletion blocking, legacy names, failure
paths, and return navigation. FatFs changes also need the R0.13c compatibility test and physical-media qualification.

## HMI and buttons

The Sharp Memory LCD is 400x240 and a full refresh blocks the HMI task long enough to lose quick button presses.

- Read buttons before running the screen state machine.
- Handle navigation/button events before redraw work and return after a state transition or full redraw.
- Rate-limit continuously changing screens instead of adding arbitrary delays. Sensors currently use a 200 ms refresh
  interval while button polling remains at 50 Hz.
- Avoid redundant full-screen refreshes. If a button is still physically held during a blocking transfer, leave time for
  the existing debounce logic to observe it rather than refreshing again immediately.
- Keep the 15 ms button debounce unless hardware evidence justifies changing it. Add hold-to-repeat only where repeated
  navigation is useful, such as long log lists.
- Bind settings behavior through `settings_button_action_e`, never by a fragile row number. `Enter Bootloader` must remain
  the final General settings item; `USB Drive` must invoke USB storage, not bootloader entry.
- Keep simulator behavior aligned with production HMI behavior. The simulator is not permission to introduce a second,
  independent implementation of a screen.
- Whenever making visual changes, run the relevant simulator scenarios and inspect the rendered screens for clipping,
  overlap, spacing, alignment, and other visual regressions. Do not rely only on automated scenario success.

Do not trade compass/recovery stability for faster navigation. Recovery changes need known-location, dual-link selection,
QR navigation, and live compass checks on hardware.

## Dependencies and build environment

- Dependency versions are pinned in `platformio.ini`; intentional local deltas are documented in the vendoring notes.
  Preserve those deltas when rebasing a vendored library and record the exact upstream tag or commit.
- Do not blanket-format vendored libraries. Preserve upstream source unless a documented CATS compatibility patch is
  necessary.
- Use PlatformIO 6.1.19 with Python 3.11 from the root `requirements.txt`. Upload-only packages are pinned in
  `ground_station/requirements-upload.txt`; never install the unrelated Python package named `serial`.
- On Windows, if `platformio` is not on `PATH`, use
  `& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"` and verify that it reports version 6.1.19. Do not
  create a separate legacy environment just to make the command available.
- Do not mix a legacy `PLATFORMIO_CORE_DIR` with the modern platform packages. Use a clean or branch-specific PlatformIO
  core when changing toolchains or when a package installation looks inconsistent.
- If the pinned Emscripten compiler under `LOCALAPPDATA` reports `permission denied` in a managed environment, retry with
  the required execution permission. Do not reinstall or modify the pinned SDK to work around an execution restriction.
- Judge firmware growth from PlatformIO's flash/RAM report and the `.bin`, not UF2 file size. UF2 block encoding normally
  makes the UF2 roughly twice the binary size.

## Before committing

Run the automated Ground Station checks from this directory:

```powershell
.\gs-precommit.ps1
```

The script formats changed C/C++ files under `src` with clang-format 17, checks the diff, builds the Windows firmware,
generates the compilation database, runs the Linux build and clang-tidy check in WSL, and runs the deterministic and
WebAssembly simulator checks. It also runs the FatFs compatibility test when FatFs files or fixtures changed.

The Windows and WSL environments must both provide PlatformIO 6.1.19. Set up the isolated WSL tools once from the
repository root inside WSL:

```bash
python3 -m venv ~/.cache/cats-gs-precommit/venv
~/.cache/cats-gs-precommit/venv/bin/python -m pip install --requirement requirements.txt clang-format==17.0.6
```

The script verifies these versions but does not install or update system or Python packages.
It also requires `rsync` and keeps an automatically synchronized working copy under
`~/.cache/cats-gs-precommit/worktree` so Linux tools do not repeatedly read the project through `/mnt/c`.

After the script passes, inspect the formatted and staged diffs and stage only the intended files. Do not commit `.pio/`,
generated UF2/bin/elf files, generated `compile_commands.json`, or generated simulator JS/WASM.

Automated checks do not replace visual review. For HMI or browser changes, inspect the affected simulator screens for
clipping, overlap, spacing, alignment, transitions, and stale rendering. Review changed golden framebuffer images
visually, and verify the live browser runtime when browser behavior changed.

## Hardware evidence

A build and simulator pass cannot validate the physical LCD, GPIO timing, sensors, task stacks, USB enumeration, MSC
ownership, DFU/UF2 reset, or bootloader recovery. Clearly say which of these remain hardware-untested.

For USB, storage, startup, task-stack, display-driver, or toolchain changes, test at least:

- immediate behavior after UF2 installation and behavior after a power cycle;
- display output (no snow), buttons, and absence of restart loops;
- Settings > USB Drive, leaving that screen, and the final Enter Bootloader item;
- entering the log list without a restart and seeing both current and legacy CSV names;
- USB connect/eject/reconnect, recording reclaim, and retained configuration/logs;
- both receiver modes, enabled-link LEDs, sensors, and recovery navigation when affected.

Keep a known-good UF2 available. For a regression across several risky changes, produce uniquely named UF2 builds at
individual commit boundaries and bisect on hardware instead of stacking speculative fixes.

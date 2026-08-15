# Ground Station desktop simulator

The simulator uses the injected HMI controller contract and the same 400x240
one-bit canvas adapter as the WebAssembly build.

From the `ground_station` directory, `gs-sim.ps1` is the Windows entrypoint:

Allow locally developed PowerShell scripts for the current Windows user once:

```powershell
Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
```

Then run:

```powershell
.\gs-sim.ps1 setup
.\gs-sim.ps1 build
.\gs-sim.ps1 serve
.\gs-sim.ps1 run .\simulator\scenarios\menu.json
.\gs-sim.ps1 test
.\gs-sim.ps1 fatfs-test
```

The browser plays the four-second startup intro on initial load and whenever
Reset is selected. Pause time and use the millisecond step control to inspect
individual deterministic animation phases.

Settings > Other > Startup Animation defaults to ON. When switched OFF and
saved, Reset shows the legacy static CATS logo for two seconds instead.

`fatfs-test` verifies that FatFs R0.16 can read and update an R0.13c storage
image, create and remount a new volume, and reject corrupted media without
writing to it.

The SDK is pinned to Emscripten `6.0.6` and records the release commit
`9981799f744be74ac67b1c1813ff172f63be0630`. Generated WebAssembly output is
ignored; source scenarios and the dependency-free runner remain checked in.

Scenario operations are `set`, `press`, `release`, `hold`, `advance`, `replay`,
`assert`, and `snapshot`. CSV replay accepts the firmware recorder format
(`link,ts[deciseconds],state,...`) and schedules rows relative to the first
decisecond timestamp. Missing radio metrics use deterministic defaults.

The browser toolbar includes presets for complete/partial and 24-log
catalogs, nominal dual recovery, a missing recovery fix, active Never-mode
recording, zero-coordinate GNSS fixes, USB deletion blocking,
write/delete/finalize failures, and manual finalization. These presets drive
the compiled controller and production `Window` renderer rather than static
images.

USB mass storage is shared automatically whenever USB is connected and the
recorder is idle. The firmware reclaims the filesystem before writing the
first flight sample and shares it again after finalization. Settings > General
> USB Drive shows the current state and can disconnect the drive manually.
Simulate eject by setting `device.usbStorageState` back to `firmware`.

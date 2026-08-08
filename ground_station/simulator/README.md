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
.\gs-sim.ps1 serve
.\gs-sim.ps1 run .\simulator\scenarios\menu.json
.\gs-sim.ps1 test
```

The SDK is pinned to Emscripten `6.0.6` and records the release commit
`833aa203ba2283fc2b6adb504a79a3a0d692df81`. Generated WebAssembly output is
ignored; source scenarios and the dependency-free runner remain checked in.

Scenario operations are `set`, `press`, `release`, `hold`, `advance`, `replay`,
`assert`, and `snapshot`. CSV replay accepts the firmware recorder format
(`link,ts[deciseconds],state,...`) and schedules rows relative to the first
decisecond timestamp. Missing radio metrics use deterministic defaults.

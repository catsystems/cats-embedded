# Ground Station simulator coverage

This matrix is the checklist for simulator changes. “Golden” compares a stable
framebuffer SHA-256, “deterministic” compares two independent runs byte for
byte, and “browser golden” exercises the compiled WebAssembly module in a real
browser and compares its framebuffer.

## Screens

| GS screen or state | Deterministic scenario | Named snapshot | Framebuffer check | Browser/WASM check |
|---|---|---|---|---|
| Animated startup | `startup-intro.json` | `rocket-launch` through `logo-hold` | Golden | `startup-rocket` browser golden |
| Static startup | `startup-static.json` | `static-logo` | Golden | — |
| Main menu | `menu.json` | `menu` | Golden | `menu` browser golden |
| Live telemetry | `live.json` | `live-telemetry` | Deterministic | Physical-controls smoke |
| Recovery | `qr-recovery.json` | `recovery-link-2-qr` | Golden | Physical-controls smoke |
| Testing | `testing-timeout.json` | `timeout` | Golden | — |
| Data and flight statistics | `log-management.json` | `log-management` | Deterministic | Fixture smoke |
| Sensors, compass, and calibration | `sensors.json`, `sensors-orientation.json` | `sensors-calibration-complete`, `orientation-east` | Deterministic | `compass-orientation` browser golden |
| Settings | `settings.json` | `settings` | Golden | USB fixture flow |
| USB Drive | `usb-storage.json` | `usb-storage-controls`, `usb-storage-handoff` | Deterministic | `usb-storage` browser golden |
| Bootloader request | `bootloader.json` | `bootloader-requested` | Deterministic | — |

## Data, recovery, and failure paths

| Path | Deterministic scenario or shared fixture | Named snapshot | Framebuffer check | Browser/WASM check |
|---|---|---|---|---|
| Replay timing | `replay.json` | `replay` | Golden | — |
| Recorded light-mode flight timing and units | `light-demo` fixture | — | Deterministic fixture load | `light-demo` browser golden |
| Recovery QR, dual link | `qr-recovery.json` | `recovery-link-2-qr` | Golden | Physical-controls smoke |
| Data QR, dual link | `qr-data.json` | `data-link-2-qr` | Golden | — |
| Missing recovery fix | `qr-no-fix.json`, `missing-fix` fixture | `no-fix-statistics` | Golden | Fixture smoke |
| Equator or prime-meridian fix | `qr-zero-coordinate.json`, `zero-coordinate` fixture | `zero-coordinate-qr` | Golden | Fixture smoke |
| More than 11 logs | `log-management.json`, `many` fixture | `log-management` | Deterministic | Fixture smoke |
| Legacy CSV names | `legacy-log-names.json` | `legacy-log-names` | Deterministic | — |
| Recording independent of screen | `recording-independent.json` | `screen-independent-recording` | Deterministic | — |
| Automatic and Never-mode finalization | `recording-modes.json`, `never` fixture | `recording-modes` | Deterministic | Fixture smoke |
| Malformed or partial log | `demo` fixture | — | Deterministic fixture load | Fixture smoke |
| Deletion failure | `delete-failure` fixture | — | Deterministic fixture load | Fixture smoke |
| USB blocks deletion | `usb-delete` fixture | — | Deterministic fixture load | Fixture smoke |
| Finalization failure | `finalize-failure` fixture | — | Deterministic fixture load | Fixture smoke |
| Recorder write failure | `recorder-fault` fixture | — | Deterministic fixture load | `recorder-fault` browser golden |
| USB remount failure | — | — | Gap | Gap |

The explicit gaps are intentional: they stay visible here until a scenario and
framebuffer assertion are added, instead of being implied by unrelated tests.

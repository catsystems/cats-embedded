# Ground Station self-test

Open **Settings > System > Self-Test**, rest the Ground Station on the bench with
access to the sky, and press **A: Start**. Results stay on the screen; no reports
or CSV files are saved. Saved settings, flight logs and compass calibration are retained.

## Phase 1 - Ground Station

Runs automatically without test Vegas:

- GNSS: fresh positions and advancing time, with up to two minutes to acquire.
- IMU: plausible, stable acceleration and gyro readings while stationary.
- Magnetometer: detected sensor with valid raw readings.
- Storage: write, reopen, verify and delete a 256-byte temporary file. USB storage
  is reclaimed before accessing files; existing files are never overwritten.

## Phase 2 - Telemetry

Power two test Vegas with phrases **cats_test_1** and **cats_test_2**, then press
**A: Start**. The following checks run automatically:

- Each telemetry chip returns a fresh version reply. Any version is accepted.
- Dual reception, dual reception with phrases swapped, single mode, receiver 1
  alone and receiver 2 alone. Each receiver is scored separately.
- Packet rate, link quality and reception gaps meet the bench limits. A failed
  radio check retries once. Original receiver settings are restored afterward.

The initial limits are at least 5 packets/s, mean link quality at least 50%, and
no gap longer than 1.5 seconds. These provisional limits need checking against
known-good units in the fixture, especially with antennas absent. They do not
measure range. No Vega testing or pyro commands are sent.

## Phase 3 - Manual checks

**Every numbered check waits for a fresh A: Start press.** Waiting has no timeout;
measurements, patterns and test timers begin only after confirmation. Completing
one check opens the next confirmation screen. Holding A cannot start the next check.

| Check | What to do after pressing Start |
| --- | --- |
| 3.1 Sensor movement | Tilt and rotate around all three axes; completion is detected automatically. |
| 3.2 Buttons | Press and release all seven buttons. The Start press does not count. |
| 3.3 Display | Inspect black, white and two checkerboard patterns. Then A = good, B = fault. |
| 3.4 Receiver LEDs | Keep both test Vegas powered. Watch receiver 1, then receiver 2 blink. Then A = good, B = fault. |
| 3.5 Battery | Unplug USB with the power switch on. The GS checks five seconds of battery operation at 3.0-4.35 V. |
| 3.6 USB reconnect | Reconnect USB to the computer. A connection made before Start does not complete this check. |

Receiver settings are restored again after the LED check. USB storage sharing
resumes when the run ends and USB is connected. The power checks do not measure
charge current or battery capacity.

Hold **B for two seconds** to cancel. Available Skip actions leave a check
**NOT TESTED**. The final screen shows **PASS** only if every check passed;
otherwise it shows **FAIL**, with individual results and reasons available using
up/down. Press A to rerun or B to return to Settings.

The simulator uses the production state machine and display renderer with fixture
inputs. Automated tests cover all confirmation gates, held buttons, timeouts,
receiver faults, storage faults, cancellation and USB sequencing. Physical sensors,
radio quality, LCD/LED appearance, battery operation and USB/storage behavior still
require verification on a Ground Station.

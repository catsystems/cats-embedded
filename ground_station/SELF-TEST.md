# Ground Station self-test

Start on battery power with **USB disconnected**. Open **Settings > System > Self-Test**,
rest the Ground Station on the bench with access to the sky, and press **A: Start**.
Leave USB disconnected until the final USB drive check. Results stay on the screen; no reports
or CSV files are saved. Saved settings, flight logs and compass calibration are retained.

## Step 0 - Gyroscope calibration

Keep the Ground Station completely still on the bench. The test averages the three
raw gyro readings over at least five seconds and 200 fresh samples. Acceleration
must remain within 0.8-1.2 g in magnitude, and each raw gyro reading within +/-5 degrees/s.
An acceleration range above 0.1 g on any axis, a gyro range above 0.5 degrees/s,
invalid readings or a sampling gap over 100 ms restart the measurement. It fails
if a stable measurement cannot be collected within 30 seconds.

Successful offsets are saved and read back in the ESP32's NVS under `cats-gyro/bias-v1`.
They are applied to the gyro before compass/orientation processing and the subsequent
IMU checks, and are loaded on future starts. Calibration always uses raw readings,
so rerunning it does not subtract the previous offsets twice. Failed or cancelled
measurements retain the previous offsets. This is a stationary zero-rate correction,
not a temperature compensation or magnetometer calibration.

## Phase 1 - Ground Station

Runs automatically without test Vegas:

- GNSS: fresh positions and advancing time, with up to two minutes to acquire.
- IMU: plausible, stable acceleration and gyro readings while stationary.
- Magnetometer: detected sensor with valid raw readings.
- Battery: five seconds alongside the sensor checks, with every voltage reading
  within 3.0-4.35 V and no active USB bus. No separate manual battery step is needed.
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
- Watch the receiver LEDs during reception; there is no separate manual LED step.

The limits are at least 8 packets/s over the observation window, mean link quality
at least 80%, SNR strictly greater than -5 dB in every received quality report, and
no gap longer than 1 second. Each active receiver needs at least one quality report.
Dual, swapped and single reception are measured for 15 seconds each; each receiver
alone is measured for 5 seconds, after a 2.5-second settling period.
These limits need checking against
known-good units in the fixture, especially with antennas absent. They do not
measure range. No Vega testing or pyro commands are sent.

## Phase 3 - Manual checks

**Every numbered check waits for a fresh A: Start press.** Waiting has no timeout;
measurements, patterns and test timers begin only after confirmation. Completing
one check opens the next confirmation screen. Holding A cannot start the next check.

| Check | What to do after pressing Start |
| --- | --- |
| 3.1 Sensor movement | Tilt and rotate around all three axes; completion is detected automatically. |
| 3.2 Buttons | Press and release all six buttons: Up, Down, Left, Right, A and B. The Start press does not count. |
| 3.3 Display | Inspect black, white and two checkerboard patterns, held for four seconds each. Then A = good, B = fault. |
| 3.4 USB drive | After Start, connect USB and open the CATS GS drive on the computer. The GS shares storage and requires a fresh successful host read within 30 seconds. Connection alone does not pass. |

Receiver settings are restored after telemetry testing. USB storage is shared during
the USB drive check and remains shared afterward. The battery check assumes the
operator starts with USB physically disconnected; bus state alone cannot prove
removal of USB power. Normal storage ownership still treats host suspension
separately from disconnection. The power checks do not measure charge current or
battery capacity.

Hold **B for two seconds** to cancel. Available Skip actions leave a check
**NOT TESTED**. The final screen shows **PASS** only if every check passed;
otherwise it shows **FAIL**, with individual results and reasons available using
up/down. Press A to rerun or B to return to Settings.

The simulator uses the production state machine and display renderer with fixture
inputs. Automated tests cover all confirmation gates, held buttons, timeouts,
receiver faults, storage faults, cancellation and USB sequencing. Physical sensors,
radio quality, LCD/LED appearance, battery operation and USB/storage behavior still
require verification on a Ground Station.

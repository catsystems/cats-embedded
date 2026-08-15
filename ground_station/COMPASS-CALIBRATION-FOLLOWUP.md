# Deferred Compass Calibration Improvements

Status: analysis only; implementation is intentionally deferred.

## Current behavior

The navigation task samples at 50 Hz. QMC5883L initialization requests a
100 Hz output rate, so acquisition speed is not the main reason calibration
feels slow. Calibration currently:

1. records raw magnetometer extrema on every navigation sample;
2. derives per-axis hard-iron offsets and soft-iron scales continuously;
3. uses the accelerometer to mark 64 Fibonacci-sphere orientation points;
4. concludes when the displayed progress reaches 100%; and
5. stores three offsets and three independent axis scales in `config.json`.

The progress calculation multiplies covered sphere points by 130%, so it
actually concludes after 50 of 64 points (78.1% coverage).

## Findings

- `calibrate()` runs before `check_rotation()`. Samples rejected for progress
  can therefore still contaminate the magnetic extrema and final result.
- The slow-motion check only tests whether acceleration magnitude is within
  0.05 g of gravity. It does not use the gyroscope to limit rotation speed.
- A pose contributes only if all three acceleration components fall within
  `0.1` of a predefined sphere point. Useful poses between those windows are
  discarded, which makes progress unnecessarily slow.
- Soft-iron extrema are collected after subtracting an offset that changes as
  new samples arrive. Earlier and later extrema are therefore expressed
  relative to different offsets.
- `resetCalib()` does not reset soft-iron extrema, offsets, scales, or progress.
  A repeated or cancelled calibration can retain state from the previous run.
- There is no final guard against insufficient axis range, division by zero,
  non-finite results, or unreasonable scale factors.
- Calibration completion is published to the HMI before the new calibration
  is copied into the configuration. This creates a narrow ordering race with
  saving `config.json`.
- QMC reads do not report whether all six data bytes were received, so an I2C
  failure cannot currently be rejected explicitly by the calibration code.
- The calibration screen rebuilds its percentage strings at the 50 Hz HMI
  rate even though the Sharp display is presented much less frequently.

## Recommended first patch

Keep the existing configuration schema and runtime transform:

```text
corrected[axis] = (raw[axis] - offset[axis]) * scale[axis]
```

The first patch should remain localized to navigation and calibration-screen
code. It should not require changes to the customized QMC5883Compass library.

1. Fully reset every temporary calibration field and progress when starting.
2. Accept magnetic samples only when acceleration and gyroscope readings show
   stable, reasonably slow movement and all inputs are finite and plausible.
3. Normalize acceleration and assign each valid pose to its nearest
   Fibonacci-sphere point using a dot product. This allows every useful pose
   to contribute while retaining broad orientation coverage.
4. Average samples within orientation bins to reduce sensor noise and prevent
   repeated stationary samples from dominating the result.
5. Accumulate raw extrema from accepted samples only. Calculate offsets and
   scales once at completion so every value uses the same coordinate frame.
6. Require adequate range on all three axes and finite, reasonable correction
   values. Continue calibration or report failure rather than saving bad data.
7. Copy the accepted calibration into the configuration before publishing the
   concluded state to the HMI.
8. Redraw the progress value only when its displayed value changes.

This should make calibration faster because valid intermediate poses are no
longer ignored, and more repeatable because only stable, evenly distributed
samples affect the result.

## Accuracy ceiling and possible second patch

Three offsets plus three scales correct hard-iron effects and axis-aligned
soft-iron distortion. They cannot correct cross-axis coupling or a tilted
ellipsoid. A full solution would fit an ellipsoid and store a 3x3 correction
matrix in addition to the offset vector.

That larger change would require a configuration-schema migration, matrix
application in the navigation loop, and stronger numerical tests. It should be
considered only if hardware measurements after the targeted patch still show
material heading error.

References:

- [NXP AN4246: Calibrating an eCompass in the Presence of Hard and Soft Iron Interference](https://www.nxp.com/docs/en/application-note/AN4246.pdf)
- [ST AN5130: LSM6DS3TR-C always-on 3D application note](https://www.st.com/content/ccc/resource/technical/document/application_note/group0/90/41/09/82/24/d9/4f/29/DM00472670/files/DM00472670.pdf/jcr%3Acontent/translations/en.DM00472670.pdf)

## Validation required before release

- Feed deterministic synthetic sphere samples with known offsets, axis scales,
  noise, and invalid readings; verify recovered calibration and rejection.
- Test successful, cancelled, repeated, and insufficient-motion calibrations.
- Verify no NaN, infinity, zero range, or implausible scale can be persisted.
- Run calibration multiple times on both QMC5883L and QMC5883P hardware and
  compare coefficient repeatability.
- Check corrected magnetic magnitude and heading at known rotations.
- Calibrate away from USB cables, computers, speakers, steel surfaces, and
  other magnetic/current-carrying objects that will not be present in normal
  operation.

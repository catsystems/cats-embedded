# VQF

Unmodified `vqf/cpp/vqf.hpp`, `vqf/cpp/vqf.cpp`, and `LICENSES/MIT.txt`
(stored here as `LICENSE`) from [VQF v2.1.2](https://github.com/dlaidig/vqf/tree/v2.1.2),
commit `86ba56bdd3158b9b05f9f9fe5596866ba326438c`.
`library.json` is CATS packaging for PlatformIO.

The GS uses full online VQF with upstream defaults, including double precision,
rest/motion gyro bias estimation and magnetic disturbance rejection. No upstream
source or algorithm defaults are patched. Units, board mounting and the GS angle
convention are defined in `src/attitude.hpp` and `src/attitude.cpp`.

Saved factory gyro offsets are subtracted before fusion; VQF estimates residual
bias only. Saving new gyro or magnetometer calibration resets the filter on the
navigation task so learned state from the old calibration is discarded.

The existing Madgwick vendor folder is retained for comparison but is no longer
included or linked by the GS. Real-filter integration tests run in
`gs-wsl-check.sh` (also called by `gs-precommit.ps1`). The display simulator injects
GS angles directly and does not simulate VQF.

Before hardware acceptance, check compass heading through a full rotation,
positive pitch when pointing up, tilt compensation, magnetic disturbance and
recovery, calibration, and navigation task timing/stack headroom at 50 Hz.

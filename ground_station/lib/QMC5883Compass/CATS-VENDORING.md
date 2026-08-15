# QMC5883Compass vendoring

This library is rebased on upstream
[`mprograms/QMC5883LCompass` v1.2.3](https://github.com/mprograms/QMC5883LCompass/releases/tag/v1.2.3),
commit `b6ff8bfab322db558e9eeed516ac617eec0b6af5`.

The upstream changes after v1.2.1 are an `int`-to-`long` fix in the blocking
`calibrate()` helper, an author-credit correction, and release metadata. The
CATS fork does not contain the blocking helper: Ground Station calibration is
performed by the navigation task. The applicable upstream metadata and author
fixes are retained without changing runtime behavior.

The CATS delta is intentionally preserved:

- common `QMC5883Compass` base API with raw and calibrated vector reads;
- runtime selection between QMC5883L (`0x0D`, WHO_AM_I `0xFF`) and QMC5883P
  (`0x2C`, WHO_AM_I `0x80`);
- QMC5883P register setup and sample decoding;
- Ground Station I2C pins and 400 kHz shared `Wire` initialization;
- calibration performed by the navigation task;
- no call to `Wire.end()`, because the bus is shared with other sensors.

Do not replace this directory directly with the upstream archive. Future
upgrades must rebase upstream changes onto these interfaces and validate both
sensor variants on hardware.

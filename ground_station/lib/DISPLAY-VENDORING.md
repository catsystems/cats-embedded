# Vendored display libraries

The display stack stays in this repository because the Ground Station
simulator compiles Adafruit GFX directly. The vendored sources are taken from
the following upstream release tags:

- Adafruit GFX Library 1.12.6 (`1.12.6`)
- Adafruit BusIO 1.17.4 (`1.17.4`)
- Adafruit SHARP Memory Display 1.1.4 (`1.1.4`)

Keep these versions aligned with `platformio.ini`. When updating them, build
both the firmware and the WebAssembly simulator and review every changed
framebuffer image before accepting new golden hashes.

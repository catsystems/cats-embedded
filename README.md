<h1 align="center">CATS Embedded Software</h1>

<p align="center">
  <img src="https://github.com/catsystems/cats-docs/blob/main/logo/PNG/logo_with_smile.png" alt="CATS logo" width="200" height="200">
</p>

<p align="center">🐈 <em>Always land on your paws!</em> 🐈‍⬛</p>

Control and Telemetry Systems (CATS) builds open source (hardware and software) flight computers for sounding rockets. Our portfolio includes the _CATS Vega_ flight computer and the _CATS Ground Station_ that can be connected to the flight computer over telemetry. The flight computer can easily be configured through the _CATS Configurator_, a desktop application. The system allows the user to configure up to 8 actions per flight transition, giving full control over the flight and a safe recovery of your rocket.
This repository contains the embedded software for the CATS Vega flight computer, its telemetry module, and the CATS Ground Station.

## Features
* Kalman filter altitude and velocity estimation
* Highly configurable events (up to eight actions per flight state transition)
* Backup timers
* High speed logging (up to 100 Hz)
* Telemetry with a range of at least 10 km
* GNSS logging and transmission to the ground station
* Accelerometric liftoff detection
* Fully open source
* Configuration is done over our application, no need to work with a CLI
* Explore rocket flights recorded with CATS flight computers at [CATS Flights](https://flights.catsystems.io/).

## Quick Links
- [CATS User Manual](./Cats%20User%20Manual.pdf)
- [CATS Configurator downloads](https://github.com/catsystems/cats-configurator/releases)
- [Firmware releases](https://github.com/catsystems/cats-embedded/releases)
- [CATS website](https://www.catsystems.io/)
- [CATS Flights](https://flights.catsystems.io/)
- [Discord community](https://discord.gg/r7ErmSNvsy)


## Repository Layout

| Path | Description |
| --- | --- |
| [`flight_computer`](./flight_computer) | CATS Vega flight-control firmware for the STM32F411 |
| [`telemetry`](./telemetry) | Onboard radio and GNSS telemetry firmware for the STM32G071 |
| [`ground_station`](./ground_station) | CATS Ground Station firmware for the ESP32-S2 |
| [`docs`](./docs) | Typst sources, fonts, and assets for the CATS User Manual |
| [`.github/workflows`](./.github/workflows) | Automated formatting, building, linting, and artifact generation |

## CATS Configurator

The CATS Configurator our desktop application used to configure Vega and work with recorded flight data. It provides access to the flight configuration, events, timers, testing functions, advanced CLI, and flight-log visualization.

Download the latest version from the [CATS Configurator releases page](https://github.com/catsystems/cats-configurator/releases). Instructions are available in the [CATS User Manual](./Cats%20User%20Manual.pdf).

## Building the Firmware

The firmware projects use [PlatformIO](https://platformio.org/). Install the repository’s pinned PlatformIO version with:

```sh
python -m pip install -r requirements.txt
```

Build all three firmware projects from the repository root:

```sh
platformio run -d flight_computer
platformio run -d telemetry
platformio run -d ground_station
```

To build the Flight Computer firmware using its debug environment:

```sh
platformio run -d flight_computer --environment debug
```

Pull requests are automatically checked for formatting, build errors, and static-analysis warnings.

## Contributing

Contributions are welcome. Examples include:

- Implementing firmware features
- Fixing bugs
- Improving documentation
- Reporting hardware or software issues
- Suggesting new features

Before submitting a pull request, make sure the affected firmware project builds successfully. The continuous-integration workflow will run formatting, compilation, and static-analysis checks.

For questions and development discussions, join the [CATS Discord community](https://discord.gg/r7ErmSNvsy).

## License and Attribution

This software is distributed under the [GNU General Public License version 3](./LICENSE.md) and is provided without warranty.

Parts of the Flight Computer CLI and embedded FAT implementation were adapted from [Betaflight](https://github.com/betaflight/betaflight) and [Cleanflight](https://github.com/cleanflight/cleanflight). Thanks to their contributors for making that work available.

## Overview
Here is a quick overview of our system, in the form of a poster that we presented at the [26th ESA PAC Symposium](https://atpi.eventsair.com/26th-esa-pac-symposium/).

<img src="https://github.com/catsystems/cats-docs/blob/main/Poster/cats_poster_26th_esa_pac_symposium.png" alt = "CATS Poster" width="900">

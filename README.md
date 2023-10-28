# CATS Embedded Software

<p align="center">
<img src="https://github.com/catsystems/cats-docs/blob/main/logo/PNG/logo_with_smile.png" alt = "CATS Logo" width="300" height="300">
</p>
<p align="center">Always land on your paws</p>

Control and Telemetry Systems (CATS) builds open source hard- and software flight computers for sounding rockets. Our portfolio includes the flight computer and a ground station which can be connected to the flight computer through telemetry. The flight computer can easily be configured through our configurator, a desktop application. The system allows the user to configure up to 8 action per flight transition, giving full control.

## Features
* Kalman filter altitude and velocity estimation
* Highly configurable events (up to eight actions per flight state transition)
* Backup timers
* High speed logging (up to 100 Hz)
* Telemetry with a range of at least 10 km
* GNSS logging and transmission to the ground station
* Accelerometric liftoff detection
* Fully open source
* Configuration is one over our application, no need to work with a CLI

## Important Links
* [Website](https://www.catsystems.io)
* [Webshop](https://www.catsystems.io/shop)
* [Manual](https://github.com/catsystems/cats-embedded/raw/main/CATS%20User%20Manual.pdf)
* [Configurator Download Link](https://github.com/catsystems/cats-configurator/releases)
* [Hardware Repository](https://github.com/catsystems/cats-hardware)

## Configuration Tool
The configuration tool just had its beta release. Further features will be released in the near future. It can be downloaded [here](https://github.com/catsystems/cats-configurator/releases). A description of the configurator can be found in the [manual](https://github.com/catsystems/cats-embedded/raw/main/CATS%20User%20Manual.pdf).

## Quick Start
Include the one Pager Here.

## Open Source
All CATS code is open source and can be used free of charge without warranty. 

The CATS CLI is forked from [Betaflight](https://github.com/betaflight/betaflight), so thanks goes to all the contributors of Cleanflight and Betaflight. 

## Contribution
Contributions are welcome and encouraged. You can contribute in many ways:

* Implement a new feature in the firmware
* Bug reporting & fixes
* New feature ideas & suggestions

The best way to get started is to join our [Discord](https://discord.gg/H9Caj8XeBj) server.

## Pushing to Remote Repository
In order to ensure consistency and easier diff review between commits,
*clang-format* is used to format all modified C/C++ files.

To download `clang-format` visit this [page](https://releases.llvm.org/download.html).


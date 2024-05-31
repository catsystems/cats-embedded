# CATS Embedded Software

<p align="center">
<img src="https://github.com/catsystems/cats-docs/blob/main/logo/PNG/logo_with_smile.png" alt = "CATS Logo" width="200" height="200">
</p>

<p align="center">🐈<i>Always land on your paws!</i>🐈‍⬛</p>

Control and Telemetry Systems (CATS) builds open source (hardware and software) flight computers for sounding rockets. Our portfolio includes the _CATS Vega_ flight computer and the _CATS Ground Station_ that can be connected to the flight computer over telemetry. The flight computer can easily be configured through the _CATS Configurator_, a desktop application. The system allows the user to configure up to 8 actions per flight transition, giving full control over the flight and a safe recovery of your rocket.

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

## Configuration Tool
The CATS Configurator is in beta. More features will be released in the near future. It can be downloaded [here](https://github.com/catsystems/cats-configurator/releases). A description of the configurator can be found in the [CATS User Manual](https://github.com/catsystems/cats-embedded/raw/main/CATS%20User%20Manual.pdf).
 

## Open Source
All CATS code is open source and available for free use under the [GPL-3.0 License](https://github.com/catsystems/cats-embedded/blob/main/LICENSE.md). Please note that it is provided without any warranty.

The CATS CLI is forked from [Betaflight](https://github.com/betaflight/betaflight), so thanks goes to all the contributors of Cleanflight and Betaflight. 

## Contribution
Contributions are welcome and encouraged. You can contribute in many ways:

* Implement a new feature in the firmware
* Bug reporting & fixes
* New feature ideas & suggestions

The best way to get started is to join our [Discord](https://discord.gg/H9Caj8XeBj) server.

## Overview
Here is a quick overview of our system, in the form of a poster that we presented at the [26th ESA PAC Symposium](https://atpi.eventsair.com/26th-esa-pac-symposium/).

<img src="https://github.com/catsystems/cats-docs/blob/main/Poster/cats_poster_26th_esa_pac_symposium.png" alt = "CATS Poster" width="900">

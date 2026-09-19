#import "../styles.typ": *

= Introduction

Welcome to the CATS User Manual. The following pages explain the CATS System so that you can install, configure, test, fly, and recover a rocket using the CATS Vega flight computer and CATS Ground Station. Each component has a practical operating section, while later chapters provide additional technical detail.#linebreak() If you have feedback, suggestions, or need further help, contact us on our #link("https://discord.gg/r7ErmSNvsy")[Discord server]. The CATS ecosystem is open source; source code and hardware designs are available on our #link("https://github.com/catsystems")[GitHub page].

== Coverage of This Manual

This manual covers the use of the CATS Vega flight computer and its Ground Station. It explains how the flight computer works, how to connect it to a #gls("Power Supply", cap: false) and deployment actuators, and how to configure it for your flight trajectory.#linebreak() This manual does not cover everything that can be done with the CATS System. In particular, it does not explain how to modify the software or hardware, or how the software works in detail. For further information about those topics, contact us on our #link("https://discord.gg/r7ErmSNvsy")[Discord server].

== Module Overview

The CATS System has three main components. #linebreak()#v(-1.8pt) *CATS Vega:* The flight computer installed inside your rocket.#linebreak() *Ground Station:* The receiver that displays and records telemetry, supports recovery, and controls bench testing.#linebreak() *Configurator:* The desktop application used to configure and update the CATS Vega, review a configuration before flight, manage profiles, and analyze completed flights.#linebreak()

== Basic Functionality

This section provides a broad overview of the CATS System for users who are new to rocketry. The following sections explain each topic in greater detail.#linebreak() The CATS Vega is the core of the system. Its main purpose is to deploy the drogue chute at #gls("apogee", cap: false) and the main chute at a user-configurable height above ground level. To do this, it combines barometric and accelerometer data to estimate height above ground level and velocity.#linebreak() The CATS System defines five *events*: _Liftoff_, _Burnout_, _Apogee_, _Main Deployment_, and _Touchdown_. The onboard control system triggers these events automatically, while the user defines the actions performed when each event occurs.#linebreak() Up to eight *actions* can be assigned to each event. Actions include delays, pyro-channel and servo-channel triggers, and low-level #gls("I/O", cap: false) signals. This allows the user to define how the deployment mechanism is actuated. Most commercial recovery mechanisms use pyro or servo channels, and the CATS Vega provides two of each.#linebreak() At the same time, the system transmits important data to the Ground Station, including GNSS data, current height, velocity, and flight state. The data is also recorded in onboard flash memory. #linebreak() The Ground Station is also used for testing mode, in which the user can trigger configured events to test separation mechanisms.

#pagebreak()

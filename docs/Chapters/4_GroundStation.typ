#import "../styles.typ": *

= Ground Station

The Ground Station is the counterpart to the CATS Vega. It receives data from the flight computer and sends commands to it. It displays the rocket's position, velocity, system health, and other important information in real time. This chapter explains how to use the Ground Station and describes its operating principle.

#cats-figure(image("../images/How To Use/Groundstation/Ground_Station.jpg", width: 80%), caption: [Ground Station])

== Hardware

=== Overview

The Ground Station is built around an ESP32-S2 microcontroller and features a transflective display that remains readable in bright sunlight. The onboard flash can store up to 1 MB of data, enough to track over an hour of flight data.

#pagebreak()

=== Specifications

#cats-table(
  table(
  columns: (0.5fr, 0.5fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: none,
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [Microcontroller],
  [ESP32-S2],
  [Flash Memory],
  [1 MB],
  [Battery],
  [Li-Ion 18650],
  [Power Consumption],
  [60mA],
  [Charging Current],
  [500mA],
  [Screen],
  [LS027B7DH01],
  [Radio],
  [2x SX1280],
  [Radio Range],
  [Tested to 10km \@100mW],
  [GNSS],
  [ATGM336H-5N]
),
  caption: [Ground Station Specifications],
  continued: false,
  breakable: false,
) <tab-GSSpecs>

== How to Use

This section covers the basic use of the Ground Station. For more advanced information, refer to the later sections.

=== Explanation of All Menus

#metadata(none) <sec-explanMenus> Use the joystick to move left, right, up, and down. Press the A button to open a menu or select an option, and press the B button to go back.#linebreak()#linebreak()#v(1.8pt)

*Live Data*#linebreak()#v(-1.8pt) The Live Data screen shows all data received from the Vega flight computer, together with information about the data-link quality.

#pagebreak()

#cats-figure(image("../images/How To Use/Groundstation/Ground_Station_Live_Data.jpg", width: 80%), caption: [Live Data of the Ground Station.])

The current flight-computer status appears at the top of the screen. The rocket's altitude, vertical velocity, GNSS coordinates, battery voltage, pyro continuity, and errors appear below it.

At the bottom, information about the telemetry link is displayed:

#list(tight: false,
  [
*AGE* - The packet age in seconds. If no packet is received for 5 seconds, the link disconnects.
],
  [
*SNR* - Signal-to-noise ratio in dB. The link can remain active down to an SNR of -15 dB. A low SNR indicates substantial radio interference at your location.
],
  [
*LQ* - Link quality as a percentage: the ratio of packets received to packets expected during the last 3 seconds.
],
  [
*RSSI* - Received signal-strength indication in dBm. The link can remain active down to an RSSI of -110 dBm. As a rule of thumb, RSSI decreases by 6 dB each time the distance doubles.
]
)

*Recovery*#linebreak()#v(-1.8pt) The Recovery screen helps you locate the rocket after it lands. It uses the rocket's last known GNSS location, or its current location if the connection is still active. The Ground Station's onboard sensors calculate the distance and direction to the rocket. For accurate direction guidance, calibrate the device's compass outdoors near the launch site and away from large metal objects. The calibration procedure will be explained in a later version of this manual. #linebreak() The Recovery screen shows the #gls("GNSS", cap: false) coordinates of the Ground Station and the rocket, as well as the distance to the rocket. Follow the arrow to locate the rocket.#linebreak()#linebreak()#v(1.8pt)

*Testing*#linebreak()#v(-1.8pt) The Testing screen is used to perform manual tests. Refer to Section #xref("sec:Testing") for a detailed explanation of testing mode and how to use it.#linebreak()#linebreak()#v(1.8pt)

#block(breakable: false)[
*Data*#linebreak()#v(-1.8pt) Not yet implemented.#linebreak()#linebreak()#v(1.8pt)
]

*Sensors*#linebreak()#v(-1.8pt) The Sensors screen displays raw data from the onboard IMU, magnetometer, and GNSS module. It also allows you to calibrate the magnetometer.#linebreak() Calibrate the magnetometer if the compass on the Recovery screen does not point north. On the Sensors screen, press A and follow the instructions. Slowly rotate the Ground Station in every direction while the progress appears on the screen. When progress reaches 100%, the calibration is complete and is saved on the Ground Station.#linebreak()#linebreak()#v(1.8pt)

*Settings*#linebreak()#v(-1.8pt) The Ground Station settings are mostly self-explanatory, and tooltips are provided for every option. The current software version supports the following settings:

#list(tight: false,
  [
*Timezone*: Choose your local timezone to display the correct time.
],
  [
*Stop Logging*: Choose whether the Ground Station stops logging after touchdown or continues logging indefinitely.
],
  [
*Version*: Ground Station software version number.
],
  [
*Bootloader*: Start the bootloader for software updates, as described in Section #xref("sec:gs_updates").
],
  [
*Telemetry Mode*: Select Single or Dual mode, as described in Section #xref("sec:telemetrymode").
],
  [
*Link Phrase 1*: Set the phrase that must match the link phrase configured on the CATS Vega.
],
  [
*Link Phrase 2*: Set the phrase that must match the second CATS Vega when tracking two flight computers. This setting is not used in Single mode.
],
  [
*Testing Phrase*: Set the phrase that must match the testing phrase configured on the CATS Vega. This phrase is required to use the testing mode described in Section #xref("sec:Testing").
]
)

=== Telemetry Modes

#metadata(none) <sec-telemetrymode> The Ground Station's telemetry settings include a _mode_ option. Because the Ground Station has two receivers, it supports two modes.

In *Dual mode*, the Ground Station can track two Vega flight computers. This is useful when a section separates from the rocket and you want to track it as well as the main body.

In *Single mode*, the Ground Station tracks one Vega flight computer. Packets from both receivers are combined, allowing more data to be received than with a single receiver. For best diversity performance, use one directional antenna and one omnidirectional antenna.

=== Data Streaming via USB

#metadata(none) <sec-data_streaming> When connected to a computer via USB, the Ground Station continuously streams each newly received telemetry packet through its virtual serial port. Each line identifies the radio link and includes the timestamp, flight state, GPS coordinates, altitude, velocity, and battery voltage; in dual-receiver mode, data from both links is reported.

The serial stream emits one line per newly received telemetry packet. All units are fixed and are not affected by the Ground Station’s unit settings:

#list(tight: false,
  [
*Link*: Receiving radio link number (1 or 2)
],
  [
*Ts*: Flight-computer uptime in seconds, with 0.1 s resolution
],
  [
*State*: Numeric flight state (0–7: Invalid, Calibrating, Ready, Thrusting, Coasting, Drogue, Main, Touchdown)
],
  [
*Lat* / *Lon*: GPS coordinates in decimal degrees
],
  [
*Alt*: Estimated altitude in meters
],
  [
*Vel*: Estimated vertical velocity in meters per second
],
  [
*V*: Flight-computer battery voltage in volts, with 0.1 V resolution
]
)

#cats-figure(image("../images/How To Use/Groundstation/Ground_Station_USB_Streaming.png", width: 95%), caption: [Ground Station telemetry data from both radio links streamed over the USB serial port.])

#pagebreak()

=== Charging

The Ground Station is powered by a Li-ion 18650 battery. A fully charged battery provides more than 8 hours of operation. Charge the battery through the USB port. At a charging current of 500 mA, a full charge can take up to 6 hours. The LED next to the USB port lights while the battery is charging and turns off when charging is complete. To replace the internal battery, remove the battery cover on the back of the Ground Station. If the replacement battery has different specifications, the estimated remaining charge shown on the screen may differ from the actual percentage.

=== How to Get the Data on Your Computer

Like the CATS Vega, the Ground Station is recognized as a mass-storage device when connected to a computer. Open the device folder and drag the recorded logs to your preferred location. Ground Station logs are stored as `.csv` files.

#cats-figure(image("../images/How To Use/Groundstation/Ground_Station_Logs.png", width: 95%), caption: [Ground station data when connecting the ground station to the user computer.])

#pagebreak()

=== Software Updates

#metadata(none) <sec-gs_updates> To update the Ground Station software, enter #gls("DFU", cap: false) mode. First, connect the Ground Station to your computer. In the Settings panel, select Bootloader. A large USB symbol appears on the screen, and a mass-storage device named SAOLA1RBOOT appears on the computer. Drag the firmware file into this folder to overwrite the old file. #linebreak() You can also enter the bootloader using the hardware controls. Remove the Ground Station casing and connect the Ground Station to your computer. Quickly press the reset button, followed by the boot button. The SAOLA1RBOOT mass-storage device will appear on your computer. Drag the firmware file onto the device. #linebreak() Firmware filenames end in `.UF2`. The latest Ground Station firmware can be downloaded from our repository.

#cats-figure(image("../images/How To Use/Groundstation/Ground_Station_DFU.png", width: 95%), caption: [SAOLA1RBOOT mass storage device when successfully changing to the #gls("DFU", cap: false) Mode])

#pagebreak()

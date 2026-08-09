#import "../styles.typ": *

= Ground Station

The ground station is the counterpart to the #gls("CATS", cap: false) Vega. It receives the data from the board and can send commands to it. It allows you to track the position and velocity of your rocket in real time as well as the health and other important information about the rocket. In the following chapters we will explain in detail how to use it and also provide more details on its working principle.

#cats-figure(image("../images/How To Use/Groundstation/Ground_Station.jpg", width: 80%), caption: [Ground Station])

== Hardware

=== Overview

The ground station is based around an ESP32S2 microcontroller and features a transflective display making it very readable even in bright sunlight. The onboard flash can store up to 1 MB of data, enough to track over an hour of flight data.

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
  [4MB],
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

This section should be sufficient for the basic usage of the ground station. For more advanced information refer to the sections further below.

=== Explanation of All Menus

#metadata(none) <sec-explanMenus> To navigate on the display, use the joystick to move left, right, up and down. Use the A button to enter a menu or select an option and the B button the go back.#linebreak()#linebreak()#v(1.8pt)

*Live Data*#linebreak()#v(-1.8pt) The Live data screen shows all the data received from the Vega flight computer as well as information about the data link quality.

#pagebreak()

#cats-figure(image("../images/How To Use/Groundstation/Ground_Station_Live_Data.jpg", width: 80%), caption: [Live Data of the Ground Station.])

On top of the screen the current status of the flight computer is shown. Information about the rocket altitude, vertical velocity, GNSS coordinates, battery voltage, pyro continuity and errors is shown below.

At the bottom, information about the telemetry link is displayed:

#list(tight: false,
  [
*AGE* - The package age in seconds. If no package is received for 5 seconds, the link disconnects.
],
  [
*SNR* - Signal to noise ratio in dB. The link can be kept alive down to a SNR of -15 dB. If the SNR is low, lots of radio interference is present at your location.
],
  [
*LQ* - Link Quality in percent, this is the ratio of packages received vs the expected number of packages over the last 3 seconds.
],
  [
*RSSI* - Received signal strength indication in dBm. The link can be kept alive down to a RSSI of -110 dBm. As a rule of thumb the RSSI reduces by 6 dB with every doubling of distance.
]
)

*Recovery*#linebreak()#v(-1.8pt) The recovery window helps you track down your rocket once it is on the ground. The last known GNSS location of the rocket (or the current if connection is still established) is used to track your rocket. The ground stations onboard sensor suite is used to calculate the distance and direction to the rocket. For the direction to work, make sure you calibrate the devices compass outdoors in the general vicinity of the launch site with no large metallic objects close by. The calibration procedure will be explained in a later version of this manual. #linebreak() The #gls("GNSS", cap: false) coordinates of the ground station and the rocket are shown in the recovery window, as well as the distance to the rocket. To find your rocket, simply follow the arrow!#linebreak()#linebreak()#v(1.8pt)

*Testing*#linebreak()#v(-1.8pt) The testing window is used to do some manual tests. Refer to section #xref("sec:Testing") for a thorough explanation of the testing mode and how to use it.#linebreak()#linebreak()#v(1.8pt)

*Data*#linebreak()#v(-1.8pt) Coming Soon#linebreak()#linebreak()#v(1.8pt)

*Sensors*#linebreak()#v(-1.8pt) The sensors tab shows you the raw data of the onboard IMU, magnetometer and GNSS module. Additionally, it allows you to calibrate the magnetometer.#linebreak() Magnetometer calibration should be done if the compass shown in the recovery tab is not showing north. To calibrate the magnetometer, simply press A in the sensors tab and follow the instructions. The ground station should be rotated slowly in all different directions for the calibration. On the screen, the progress is shown. Once it reaches 100% the magnetometer is calibrated and the calibration is saved on the ground station.#linebreak()#linebreak()#v(1.8pt)

*Settings*#linebreak()#v(-1.8pt) The ground station settings are built in a way that it should be mostly self explanatory. Tool tips are shown for all options. The current Software version support the following settings:

#list(tight: false,
  [
*Timezone*: Chose the timezone which you are operating in for correct time display.
],
  [
*Stop Logging*: Choose if the ground station stops logging after touchdown or never stops logging.
],
  [
*Version*: ground station software version number.
],
  [
*Bootloader*: Start the bootloader for software updates, described in section #xref("sec:gs_updates").
],
  [
*Telemetry Mode*: Single or Dual mode, described in section #xref("sec:telemetrymode").
],
  [
*Link Phrase 1*: Link phrase which needs to match the link phrase configured on the CATS Vega for connection.
],
  [
*Link Phrase 2*: Link phrase which needs to match the link phrase configured on the CATS Vega for connection to track a second CATS Vega. Not used if the telemetry mode is set to single mode.
],
  [
*Testing Phrase*: Test phrase which needs to match the test phrase configured on the CATS Vega. This test phrase is crucial to use the testing mode of the system explained further in section #xref("sec:Testing").
]
)

=== Telemetry Modes

#metadata(none) <sec-telemetrymode> Under the telemetry settings on the ground station you will find an option called _mode_. Since the ground station has two receivers, they can be used in two modes.

In *Dual mode* the ground station can track two Vega computers. This can be useful if you separate a section of your rocket and want to keep track of it as well as the main body.

In *Single mode*, the ground station tracks just one Vega computer. Packages are fused from both antennas making it possible to receive more data than with just one receiver. We recommend that you use a directional as well as an omnidirectional antenna in diversity mode to get the best performance out of it.

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

The ground station is powered by a Li-Ion 18650 battery. With a fully charged battery the system can run for more than 8 hours. Charging the battery can be done through the USB port. With a charging current of 500 mA it can take up to 6 hours for it to fully charge up. While the battery is charging, the LED next to the USB port will light up and turn off once it is fully charged. The internal battery can also be replaced by removing the battery cover on the back of the ground station. If the battery is replaced with one of other specification, the remaining battery estimate displayed on the screen can differ from the actual battery percentage.

=== How to Get the Data on Your Computer

Just like the #gls("CATS", cap: false) Vega, the ground station can be connected to any computer and is recognized as a mass storage device. A folder will open up on your computer and you can just drag and drop the recorded logs to your preferred location. The logs from the ground station are stored in a .csv file format.

#cats-figure(image("../images/How To Use/Groundstation/Ground_Station_Logs.png", width: 95%), caption: [Ground station data when connecting the ground station to the user computer.])

#pagebreak()

=== Software Updates

#metadata(none) <sec-gs_updates> To do software updates on the ground station, #gls("DFU", cap: false) mode needs to be entered. First connect the ground station with your computer. Then select the bootloader setting in the Settings panel. Then a large USB symbol is shown on the screen and on the computer a new mass storage device will appear on your computer with the name SAOLA1RBOOT. Now, simply drag and drop the firmware file into the folder. The old file is then overwritten. #linebreak() Entering the bootloader can also be done with the hardware only. In this case, remove the casing from the ground station. Plug in the ground station to your computer. Then quickly click the reset button, followed by the boot button. If you do this quickly, the mass storage device with SAOLA1RBOOT will appear on your computer. Then you can just drag and drop the firmware to your ground station. #linebreak() Firmware files have the file ending .UF2 and the newest version of the ground station can always be downloaded from our repository.

#cats-figure(image("../images/How To Use/Groundstation/Ground_Station_DFU.png", width: 95%), caption: [SAOLA1RBOOT mass storage device when successfully changing to the #gls("DFU", cap: false) Mode])

#pagebreak()

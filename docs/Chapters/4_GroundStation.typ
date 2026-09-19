#import "../styles.typ": *

= Ground Station

The Ground Station is the counterpart to the CATS Vega. It receives data from the flight computer and sends commands to it. It displays the rocket's position, velocity, system health, and other important information in real time. This chapter explains how to use the Ground Station and describes its operating principle.

#cats-figure(doc-image("How To Use/Groundstation/Ground_Station.jpg", width: 80%), caption: [Ground Station])

== Hardware

=== Overview

The Ground Station is built around an ESP32-S2 microcontroller and features a transflective display that remains readable in bright sunlight. It has 4 MB of internal flash, including a 1 MB FAT data partition for logs and firmware-transfer files.

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
  [4 MB internal; 1 MB data partition],
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

#metadata(none) <sec-explanMenus> Use the joystick to move left, right, up, and down. Press A to open a menu, select an item, or confirm an action. Press B to go back. Arrows and hints at the edges of the display indicate when another page or action is available.

#cats-figure(
  doc-image("How To Use/Groundstation/GS_Main_Menu.png", width: 80%, outline: true, alt: "Ground Station main menu rendered by the simulator"),
  caption: [Ground Station main menu],
)

==== Live

The Live screen shows telemetry from the connected Vega or Vegas. It displays flight state, altitude, vertical velocity, battery voltage, pyro continuity, errors, and radio-link information. Press Left for the GNSS view and Right for the downrange view. The downrange view uses the Ground Station's own GNSS position to show the rocket's relative direction and distance.

The link indicators include:

#list(tight: false,
  [
*AGE* - Time since the last received packet. A link is treated as disconnected after five seconds without a packet.
],
  [
*SNR* - Signal-to-noise ratio in dB. Lower values indicate a noisier radio environment.
],
  [
*LQ* - Percentage of expected packets received during the recent measurement window.
],
  [
*RSSI* - Received signal strength in dBm. More negative values indicate a weaker received signal.
]
)

#cats-figure(
  responsive-split(
    [#subfigure(doc-image("How To Use/Groundstation/GS_Live_Telemetry.png", width: 100%, outline: true, alt: "Ground Station live telemetry screen"), [GNSS telemetry from both receivers in Single mode.], "a")],
    [#subfigure(doc-image("How To Use/Groundstation/GS_Live_Downrange.png", width: 100%, outline: true, alt: "Ground Station live downrange screen"), [Relative downrange distance and direction.], "b")],
    columns: (46%, 1fr, 46%),
  ),
  caption: [Ground Station Live views],
)

==== Recovery

The Recovery screen guides you toward the last valid GNSS position received from the rocket. It shows the selected rocket, distance, relative direction, and whether a usable location is available. Calibrate the Ground Station compass outdoors near the launch site and away from large metal objects before relying on direction guidance.

In Dual receiver mode, press Up or Down to choose Link 1 or Link 2. Press Right to show a QR code for the selected last location; scan it with a phone to open the coordinates in a mapping application. If the other link also has a valid location, press Right again to switch QR-code pages. Press Left to return to direction guidance.

#cats-figure(
  responsive-split(
    [#subfigure(doc-image("How To Use/Groundstation/GS_Recovery.png", width: 100%, outline: true, alt: "Ground Station recovery direction screen"), [Direction and distance to the last received location.], "a")],
    [#subfigure(doc-image("How To Use/Groundstation/GS_Recovery_QR.png", width: 100%, outline: true, alt: "Ground Station recovery location QR code"), [QR code for transferring the selected last location.], "b")],
    columns: (46%, 1fr, 46%),
  ),
  caption: [Ground Station Recovery views],
)

==== Testing

The Testing screen arms testing mode and manually triggers configured flight events. Triggered events execute their assigned actions. Read Section #xref("sec:Testing") completely before using this screen.

==== Data

The Data screen lists logs stored on the Ground Station. An active recording is marked as active. Select a log to view its duration, maximum altitude and velocity, flight state, and recorded locations. When a valid location is available, open its QR-code page to transfer the coordinates to a phone.

The options page can finalize the active log or delete a completed log after confirmation. Disconnect the Ground Station USB drive before deleting a log. Finalizing stops the active recording and closes its files; deleting permanently removes the selected completed log.

#cats-figure(
  responsive-split(
    [#subfigure(doc-image("How To Use/Groundstation/GS_Data_Logs.png", width: 100%, outline: true, alt: "Ground Station list of recorded flight logs"), [Select a stored Ground Station flight log.], "a")],
    [#subfigure(doc-image("How To Use/Groundstation/GS_Data_Details.png", width: 100%, outline: true, alt: "Ground Station flight-log statistics"), [Review recorded statistics and last locations.], "b")],
    columns: (46%, 1fr, 46%),
  ),
  caption: [Ground Station Data views],
)

==== Sensors

The Sensors screen shows raw IMU, magnetometer, and GNSS readings. Press Right or Down to open the Compass / 3D Orientation page, which shows the Ground Station's heading, pitch, and roll. Press Left or Up to return to raw readings.

Press A to start compass calibration and follow the on-screen instructions. Rotate the Ground Station slowly through multiple orientations, away from large metal objects and magnets. When calibration reaches 100%, confirm the result to save it.

#cats-figure(
  responsive-split(
    [#subfigure(doc-image("How To Use/Groundstation/GS_Sensors_Readings.png", width: 100%, outline: true, alt: "Ground Station raw IMU and GNSS sensor readings"), [Raw IMU, magnetometer, and GNSS readings.], "a")],
    [#subfigure(doc-image("How To Use/Groundstation/GS_Sensors_Orientation.png", width: 100%, outline: true, alt: "Ground Station compass and 3D orientation screen"), [Compass heading, pitch, and roll.], "b")],
    columns: (46%, 1fr, 46%),
  ),
  caption: [Ground Station Sensors views],
)

==== Settings

Settings are divided into three pages. Move Left or Right while no setting is selected to change pages.

#cats-table(
  table(
  columns: (0.22fr, 0.3fr, 0.48fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [Page], [Setting], [Purpose],
  [Telemetry], [Receiver Mode], [Single uses both receivers for one Vega; Dual assigns one Vega to each receiver.],
  [Telemetry], [Link Phrase 1], [Phrase used by both receivers in Single mode or the left receiver in Dual mode.],
  [Telemetry], [Link Phrase 2], [Phrase used by the right receiver in Dual mode.],
  [Telemetry], [Test Phrase], [Phrase required to arm Vega testing mode.],
  [Preferences], [Stop Logging], [Stop at landing or continue until manually finalized.],
  [Preferences], [Time Zone], [Local offset from UTC.],
  [Preferences], [Units], [Metric or imperial display units; recorded data remains metric.],
  [Preferences], [Startup Animation], [Animated startup or static CATS logo.],
  [System], [Firmware Versions], [Ground Station and both receiver-firmware versions.],
  [System], [USB Drive], [View or disconnect the shared USB storage.],
  [System], [Self-Test], [Factory-oriented automatic and guided hardware checks.],
  [System], [Update Firmware], [Update the Ground Station application or both radio receivers.],
  table.hline(y: 1, stroke: 0.5pt + black)
),
  caption: [Ground Station settings],
  continued: false,
  breakable: true,
) <tab-GSSettings>

#cats-figure(
  responsive-split(
    [#subfigure(doc-image("How To Use/Groundstation/GS_Settings_Telemetry.png", width: 100%, outline: true, alt: "Ground Station Telemetry settings page"), [Telemetry settings.], "a")],
    [#subfigure(doc-image("How To Use/Groundstation/GS_Settings_Preferences.png", width: 100%, outline: true, alt: "Ground Station Preferences settings page"), [Display and recording preferences.], "b")],
    columns: (46%, 1fr, 46%),
  ),
  caption: [Ground Station Telemetry and Preferences settings],
)

#cats-figure(
  doc-image("How To Use/Groundstation/GS_Settings_System.png", width: 46%, outline: true, alt: "Ground Station System settings page"),
  caption: [Ground Station System settings],
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

#cats-figure(doc-image("How To Use/Groundstation/Ground_Station_USB_Streaming.png", width: 95%), caption: [Ground Station telemetry data from both radio links streamed over the USB serial port.])

#pagebreak()

=== Charging

The Ground Station is powered by a Li-ion 18650 battery. A fully charged battery provides more than 8 hours of operation. Charge the battery through the USB port. At a charging current of 500 mA, a full charge can take up to 6 hours. The LED next to the USB port lights while the battery is charging and turns off when charging is complete. To replace the internal battery, remove the battery cover on the back of the Ground Station. If the replacement battery has different specifications, the estimated remaining charge shown on the screen may differ from the actual percentage.

=== How to Get the Data on Your Computer

When connected by USB and not actively recording, the Ground Station shares its data partition as the `CATS GS` USB drive. Open the drive and copy the required `.csv` logs to the computer. The firmware reclaims the filesystem automatically when recording starts and shares it again after the log is finalized. Close files before recording or updating firmware, and use *Settings* $arrow.r$ *System* $arrow.r$ *USB Drive* when the drive needs to be disconnected manually.

=== Software Updates

#metadata(none) <sec-gs_updates> Use the Configurator's Firmware Updates page for normal Ground Station and radio-receiver updates. See Section #xref("sec:FirmwareUpdates") for the complete preparation, installation, verification, and recovery procedures.

#pagebreak()

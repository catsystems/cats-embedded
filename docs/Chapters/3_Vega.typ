#import "../styles.typ": *

= CATS Vega

This section describes how the flight computer works and how to configure it for a flight. The How to Use section explains its basic features. For more detailed information, refer to Section #xref("sec:AdvancedInfo").

#cats-figure(doc-image("How To Use/Vega/Vega_HW.jpeg", width: 60%), caption: [CATS Vega])

== Hardware

#metadata(none) <sec-VegaHW>

=== Specifications

#cats-table(
  table(
  columns: (0.5fr, 0.5fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: none,
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [Size],
  [100 × 33 × 15 mm (without the antenna)],
  [Weight],
  [33 g],
  [Input Voltage],
  [7 - 24 V],
  [Power Consumption],
  [100 mA],
  [Number of #gls("pyro", cap: true) channels],
  [2],
  [Number of #gls("servo", cap: true) channels],
  [2],
  [Number of IOs],
  [1],
  [Additional IO],
  [#gls("UART", cap: false)],
  [#gls("servo", cap: true) Power],
  [5 V / 3 A max.],
  [Microcontroller],
  [STM32F4],
  [Flash Memory],
  [16MB],
  [#gls("IMU", cap: false)],
  [LSM6DSO32],
  [#gls("barometer", cap: false)],
  [MS5607],
  [Radio Frequency],
  [ISM 2.4GHz],
  [Radio Power],
  [Up to 1W],
  [Radio Range],
  [Tested to 10km \@100mW]
),
  caption: [Vega Specifications],
  continued: false,
  breakable: false,
) <tab-Specs>

#pagebreak()

=== Hardware Overview

This section provides a quick hardware overview and shows the location of each port. The numbered markers in Figure #xref("fig:VegaHWSpecs") correspond to the following list.#linebreak()#v(-1.8pt)

#responsive-split(columns: (63%, 1fr, 33%), [
#enum(tight: false,
  [
*Switch Port*; Connect a manual switch between the two terminals.
],
  [
*Battery Port*; Connect a battery to these terminals and observe the correct polarity.
],
  [
*Buzzer*; Indicates flight-computer readiness and status through beeping patterns, as explained in Section #xref("sec:BeepingPatterns").
],
  [
*Status LEDs*; The POWER LED is illuminated when power is present. The STATUS LED blinks when the system is operating normally.
],
  [
*USB Connector*; The connector is on the other side of the board.
],
  [
*Test Button*; If this button is held during startup, the board enters testing mode if a testing phrase has been configured.
],
  [
*Servo Connector*; This connector fits the standard servo connectors. Two servos can be connected to this connector.
],
  [
*Telemetry LEDs*; The #gls("GNSS", cap: false) LED blinks whenever #gls("GNSS", cap: false) coordinates are received. The LINK LED blinks after a connection to the Ground Station has been established.
],
  [
*Low-Level #gls("I/O", cap: false) and #gls("UART", cap: false) Connector*; Connect external hardware to this port to exchange data with the CATS board.
],
  [
*#gls("pyro", cap: true) LEDs*; These red LEDs are turned on when continuity of the #gls("pyro", cap: false) channel is detected.
],
  [
*#gls("pyro", cap: true) Channel 1*; Connect a pyrotechnic charge or another supported device to this connector.
],
  [
*#gls("pyro", cap: true) Channel 2*; Connect a pyrotechnic charge or another supported device to this connector.
],
  [
*Antenna Connector*; Connect an antenna here so that the CATS Vega can transmit data to the Ground Station.
]
)
], [
#cats-figure(doc-image("How To Use/Vega/VEGA-Hardware-optimized.png", width: 100%), caption: [CATS Vega board hardware specifications]) <fig-VegaHWSpecs>
])

== Working Principle

This section briefly introduces the operating principles needed to understand the Vega flight computer's configuration options.

=== Configurable Actions & Finite State Machine

#metadata(none) <sec-FSM> The finite state machine (#gls("FSM", cap: false)), shown in Figure #xref("fig:FSM"), controls the outputs of the Vega flight computer. When the flight computer is turned on, it starts in the Calibrating state. Every flight follows the sequence of states shown in Figure #xref("fig:FSM"). Whenever a state transition occurs, the associated event is triggered.

#cats-figure(doc-image("Working Principle/Finite_State_Machine.png", width: 13cm), caption: [Finite State Machine controlling the CATS software.]) <fig-FSM>

Each event can trigger one or more actions, such as a #gls("pyro", cap: false) channel, #gls("servo", cap: false) channel, or timer. Use the Configurator to assign actions to events, as described in Section #xref("sec:DescriptionOfConfigurator").

#cats-table(
  table(
  columns: (0.5fr, 0.5fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: none,
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [#gls("Calibrating", cap: false) $arrow.r$ Testing],
  [A telemetry command starts testing mode. This transition is available only if testing mode was enabled through the Configurator.],
  [#gls("Calibrating", cap: false) $arrow.r$ #gls("Ready", cap: false)],
  [#gls("IMU", cap: false) (gyroscope and linear acceleration) readings are constant for 10 seconds.],
  [#gls("Ready", cap: false) $arrow.r$ Thrusting],
  [The measured acceleration in any direction exceeds the user-defined acceleration threshold for 0.1 seconds.],
  [Thrusting $arrow.r$ Coasting],
  [The measured acceleration in the "up" direction is smaller than 0 $m/s^2$ for 0.1 seconds],
  [Coasting $arrow.r$ Drogue],
  [The estimated velocity needs to be smaller than 0 $m/s$ for 0.3 seconds],
  [Drogue $arrow.r$ Main],
  [The estimated height is below the user-defined height for 0.3 seconds],
  [Main $arrow.r$ Touchdown],
  [The estimated velocity is in the bound $[-3, 3] m/s$ for 1 second]
),
  caption: [FSM Transition Specifications],
  continued: false,
  breakable: false,
) <tab-FSMTransitions>

With this setup for state changes, the flight has a strictly controlled order. The Main event can only be thrown after the #gls("apogee", cap: false) event. Events are also *unique*; during a flight only one event can be thrown.#linebreak()

#note[
*Note:* A software safeguard checks the time between #gls("liftoff", cap: false) and #gls("apogee", cap: false). If that time is smaller than 1.5 seconds, the flight computer assumes a faulty #gls("liftoff", cap: false) detection and no further events or actions are activated. The flight computer then jumps instantly to #gls("touchdown", cap: false) without triggering anything.
]

=== Actions

When an event is triggered, the flight computer performs the actions assigned to it. Up to eight actions can be assigned to each event, supporting a wide range of applications. Examples include:

#list(tight: false,
  [
Enabling a camera at #gls("liftoff", cap: false) using a #gls("pyro", cap: false) channel,
],
  [
Actuating a solenoid valve for two seconds using a #gls("pyro", cap: false) channel,
],
  [
Enabling some mechanism at engine burnout,
],
  [
Disabling the camera at touchdown using the #gls("pyro", cap: false) channel,
],
  [
...
]
)

The full range of actions can be found below.

#cats-table(
  table(
  columns: (0.5fr, 0.5fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: none,
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [Action],
  [Parameter],
  [#gls("pyro", cap: true) 1],
  [ON/OFF],
  [#gls("pyro", cap: true) 2],
  [ON/OFF],
  [#gls("servo", cap: true) 1],
  [\[0-1000\]‰],
  [#gls("servo", cap: true) 2],
  [\[0-1000\]‰],
  [Low-Level I/O],
  [ON/OFF],
  [Delay],
  [\[0-15000\] ms],
  [Recorder],
  [ON/OFF/PREFILLING],
  table.hline(y: 2, stroke: 0.5pt + black)
),
  caption: [Exhaustive List of all possible Actions],
  continued: false,
  breakable: false,
) <tab-ActionTable>

#note[
*Note:* The _PREFILLING_ recorder option continuously accumulates log elements in a buffer. Once the recorder transitions into _ON_ mode, the elements in the buffer are written to the flash chip. This information can be used to analyze the initial ignition sequence and thrust build-up.
]

How actions can be configured is shown in section #xref("sec:Examples").

#pagebreak()

== How to Use

Now that the hardware and software have been introduced, this section explains how to configure and mount the flight computer, update its software, and generate plots from flight data.

=== Connection to Your Computer

Before connecting your CATS Vega to your computer, download the Configurator from our #link("https://github.com/catsystems/cats-configurator/releases/")[releases page]. Drivers are usually not required. If your computer does not recognize the device, refer to the troubleshooting steps in our #link("https://github.com/catsystems/cats-embedded/wiki/Installation")[wiki]#source-note("https://github.com/catsystems/cats-embedded/wiki/Installation").

=== Description of the Configurator

#metadata(none) <sec-DescriptionOfConfigurator> The Configurator is the desktop application used to configure the CATS Vega, inspect its status, manage configuration profiles, run a preflight review, update firmware, and analyze flight logs. Download the latest release from the #link("https://github.com/catsystems/cats-configurator/releases/")[Configurator releases page]#source-note("https://github.com/catsystems/cats-configurator/releases").

When exactly one compatible Vega is connected, the Configurator selects it and connects automatically. If no automatic connection is made, use the device selector in the upper-right corner, choose the intended Vega, and select *Connect*. Use the refresh button if a newly connected device is not listed. When several Vegas are connected, always verify the selected device before changing or saving a configuration.

The left navigation provides the following work areas:

#cats-table(
  table(
  columns: (0.28fr, 0.72fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [Configuration],
  [Flight settings, telemetry, testing, recorder settings, live board status, and hardware information.],
  [Events & Timers],
  [Actions assigned to flight events and the four configurable timers.],
  [Profiles],
  [Export the connected board configuration, compare another profile, and apply compatible settings.],
  [Preflight],
  [A read-only review of the configuration, warnings, event sequence, timers, and outputs.],
  [#gls("CLI", cap: false)],
  [Direct access to advanced flight-computer commands.],
  [Flight Logs],
  [Open local or onboard `.cfl` logs, generate plots, export data, or open a log in CATS Flights.],
  [Firmware Updates],
  [Detect supported devices, retrieve official releases, and install or prepare firmware.],
  [Flights],
  [Open CATS Flights in the default browser.]
),
  caption: [Configurator navigation],
  continued: false,
  breakable: false,
) <tab-ConfiguratorNavigation>

==== Configuration

The Configuration page groups flight settings into General, Telemetry, Testing, Recording, Info, and Hardware Info panels. The General panel contains the main-deployment altitude, liftoff threshold, and initial servo positions. For most flights, use a liftoff threshold of 40 $m/s^2$ unless testing with the complete rocket shows that another value is required. The Telemetry and Testing panels contain the enable controls and phrases used by the Ground Station.

Changes shown in the Configurator are not persistent until *Save* is selected. Use *Refresh* to discard unsaved edits and reload the values stored on the Vega. *Reset Config* restores the default configuration after confirmation.

==== Events & Timers

The Events & Timers page shows all flight events and their assigned actions, followed by the four timers. Select *Add Action* to add an output, recorder command, or delay to an event. Existing actions can be edited or removed. Up to eight actions can be assigned to each event. Select *Save* after changing events or timers.

Each timer has a start event, a duration, and an event to trigger when the duration expires. A timer can provide a backup event or trigger Custom 1 or Custom 2 for a payload sequence. Triggering an event through a timer does not force the flight-state estimator into the corresponding state.

#note[
*Note:* Flight events are unique during normal operation. If both the estimator and a timer could trigger the same event, only the first occurrence is processed.
]

#cats-figure(
  figure-stack(
    spacing: 12pt,
    breakable: true,
    subfigure(
      doc-image("How To Use/Configurator/Configurator_Configuration.png", width: 100%, alt: "Configurator Configuration page connected to a CATS Vega"),
      [Review and edit the connected Vega configuration.],
      "a",
    ),
    subfigure(
      doc-image("How To Use/Configurator/Configurator_Events_Timers.png", width: 100%, alt: "Configurator Events and Timers page"),
      [Review event actions and timer configuration.],
      "b",
    ),
  ),
  caption: [Configurator Configuration and Events & Timers pages],
  breakable: true,
)

==== Profiles

#metadata(none) <sec-Profiles> The Profiles page can export the complete connected-board configuration as a JSON profile. Opening another profile shows a setting-by-setting comparison with the connected Vega, including differences in configuration, events, timers, and recorder settings. Review compatibility warnings before applying a profile. You can apply the full compatible profile or individual differing settings, then save the resulting configuration to the board.

==== Preflight

#metadata(none) <sec-Preflight> The Preflight page reads the saved board configuration and produces a report without changing it. Run Preflight after saving the final configuration. Review every error and warning, especially testing mode, telemetry and recording status, deployment actions, liftoff threshold, timer cycles, and event ordering. The event timeline summarizes which actions run at each flight event and which active timers may trigger additional events.

Preflight is an additional configuration review, not a substitute for continuity checks, deployment-system testing, range procedures, or a redundant recovery system.

#cats-figure(
  figure-stack(
    spacing: 12pt,
    breakable: true,
    subfigure(
      doc-image("How To Use/Configurator/Configurator_Profiles.png", width: 100%, alt: "Configurator Profiles page showing connected-board settings"),
      [Inspect the connected-board profile or compare another profile before applying it.],
      "a",
    ),
    subfigure(
      doc-image("How To Use/Configurator/Configurator_Preflight.png", width: 100%, alt: "Configurator Preflight page showing a successful read-only review"),
      [Run the read-only Preflight review after saving the final configuration.],
      "b",
    ),
  ),
  caption: [Configurator Profiles and Preflight pages],
  breakable: true,
)

==== CLI

The CLI page sends commands directly to the connected Vega. It is intended for advanced inspection and troubleshooting; normal configuration should be performed through the other pages. Common commands are listed in Section #xref("sec:CLI").

==== Flight Logs

#metadata(none) <sec-FlightLogs> The Flight Logs page accepts one Vega `.cfl` file selected through the file picker or dropped anywhere in the Configurator window. When a Vega USB drive is mounted, the page also lists its onboard logs. An onboard log can be viewed locally, saved as a copy, deleted from the Vega, or opened in CATS Flights.

After opening a log, use *Export CSV* for tabular data or *Export HTML* for interactive plots. *Open in Flights* hands the selected log to CATS Flights through the local browser; the log remains on the computer unless it is explicitly saved or shared there.

#cats-figure(
  doc-image("How To Use/Configurator/Configurator_Flight_Logs.png", width: 100%, alt: "Configurator Flight Logs page listing an onboard Vega log"),
  caption: [Configurator Flight Logs page with a connected Vega],
)

=== Mounting

#responsive-split(columns: (60%, 1fr, 33%), [
The CATS Vega *does not* require a specific mounting orientation. The system automatically detects the gravity vector for internal state estimation, so you can mount the board in any orientation.#linebreak() The board has a length of 100 mm, a width of 33 mm and a total height of 15 mm. Three mounting holes secure the system to the rocket. The mounting holes are spaced 60 mm by 27 mm and are designed for M3 screws. Use spacers to prevent the electronics from touching the rocket. Download the system's 3D files from our #link("https://github.com/catsystems/cats-hardware/tree/main/CATS-Vega/3D")[GitHub repository]#source-note("https://github.com/catsystems/cats-hardware/tree/main/CATS-Vega/3D").#linebreak() For reliable radio reception during flight, pay close attention to the area surrounding each antenna. Install the CATS flight computer in a radio-transparent section of the rocket, such as fiberglass or cardboard. Do not install it in a carbon-fiber section, which blocks #gls("RF", cap: false) signals. Ensure that the onboard #gls("patch antenna", cap: false) has a clear view of the sky for optimal #gls("GNSS", cap: false) reception, and keep the telemetry antenna away from metal objects.

After power-up, the system detects the up direction once it is stable. A beeping pattern and the Ground Station indicate when the flight computer enters the #gls("Ready", cap: false) state. In this state, the flight computer is armed and waiting for #gls("liftoff", cap: false). Do not move the rocket, and follow all safety guidelines. At this stage, the flight computer can be disarmed only by switching it off. For more information about calibration, refer to Sections #xref("sec:EstAlg") and #xref("sec:FSM").#linebreak()
], [
#cats-figure(doc-image("How To Use/Vega/Mounting_Img.png", width: 80%), caption: [CATS Vega board with the mounting hole and dimensions.])
])

#warning[
*Warning:* Power up the flight computer only after the rocket is upright on the launch pad. Calibration is performed once, as soon as no motion is detected after startup.
]

=== Battery, Switch and Actuators

The CATS Vega has one battery port, one switch port, two #gls("pyro", cap: false) channels, two #gls("servo", cap: false) channels, and one low-level #gls("I/O", cap: false). The following sections briefly explain each port. Table #xref("tab:HowToAct") summarizes the most important parameters. For more information about the board hardware, refer to Section #xref("sec:VegaHW") and the labeled board diagram in Figure #xref("fig:VegaHWSpecs").#linebreak()#linebreak()#v(1.8pt)

*Battery Port*#linebreak()#v(-1.8pt) The battery port supports voltages between 7 and 25 volts. For LiPo and Li-ion batteries, this corresponds to 2- to 6-cell battery packs. The battery port is protected against reverse polarity.#linebreak()#linebreak()#v(1.8pt)

*Switch Port*#linebreak()#v(-1.8pt) The switch port allows the user to add a mechanical switch to the system. If this switch is turned off, the system is fully disconnected from power.#linebreak()

#note[
*Note:* The battery current is routed through the switch. Make sure that the wires and the switch are rated for the currents required.
]

*Pyro Channels*#linebreak()#v(-1.8pt) The #gls("pyro", cap: false) channels apply the battery voltage to the connected circuitry with a voltage drop of approximately 1 V. An electric match is normally connected to a channel to ignite a black-powder charge. The channels can also power other devices. For example, they can actuate solenoid valves (with an external flyback diode), power cameras, or power other electronic circuits. By default, the maximum continuous current is approximately 1 A. The channels are short-circuit protected by a resettable PTC fuse. This current is more than sufficient to ignite electric matches before the fuse reduces it. If the connected load requires more current, the fuse can be bypassed with a solder jumper on the back of the board. In this configuration, stay below 5 A continuous or 20 A burst. Exercise extreme caution: a short circuit on the channel can damage the board.#linebreak()

#warning[
*Warning:* The pyro channels are short-circuit protected by a resettable PTC fuse. If the connected circuit requires more than 1 A, close the solder bridge on the back of the board.
]

*#gls("servo", cap: true) Channels*#linebreak()#v(-1.8pt) The #gls("servo", cap: false) channels can actuate #gls("PWM", cap: false) #gls("servo", cap: false)s. An onboard voltage regulator reduces the battery voltage to 5 V to power the servos. The microcontroller's power rail is completely separate from the 5 V #gls("Power Supply", cap: false); therefore, a short circuit on the servo power rail does not affect the system. A maximum current of 3 A can be drawn. A #gls("PWM", cap: false) signal is always applied to each #gls("servo", cap: false) channel, and the endpoints can be changed in the Configurator.#linebreak()#linebreak()#v(1.8pt)

*Low-Level #gls("I/O", cap: false)*#linebreak()#v(-1.8pt) The low-level #gls("I/O", cap: false) can send a signal to another system. The voltage level is 3.3 V, and the pin is connected directly to the microcontroller. Therefore, the #gls("I/O", cap: false) should be used *only* for signal transmission, not to actuate a recovery mechanism.

#pagebreak()

*#gls("I/O", cap: false) Specification*

#cats-table(
  table(
  columns: (0.2fr, 0.5fr, 0.3fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [#gls("I/O", cap: false)],
  [Description],
  [Limits],
  [Battery Port],
  [Connect battery],
  [7-24 V],
  [Switch Port],
  [Connect mechanical switch],
  [n.a.],
  [#gls("pyro", cap: false) Channels],
  [Connect up to two pyrotechnic charges or other devices],
  [Battery voltage / 5 A],
  [#gls("servo", cap: true) Channels],
  [Used for #gls("servo", cap: false) actuation, up to two #gls("servo", cap: false)s],
  [5 V / 3 A],
  [Low-Level #gls("I/O", cap: false)],
  [Use only for signal transmission, not actuation],
  [3.3 V / 10 mA],
  table.hline(y: 2, stroke: 0.5pt + black)
),
  caption: [Overview of the I/Os],
  continued: false,
  breakable: false,
) <tab-HowToAct>

#pagebreak()

=== Setting up the Minimal Flight Configuration

For nominal flight performance, several parameters must be configured before every flight. In particular, the user must know:#linebreak()

#list(tight: false,
  [
Expected maximum acceleration
],
  [
Recovery mechanism for the #gls("drogue chute", cap: false)
],
  [
Recovery mechanism for the #gls("main chute", cap: false)
],
  [
Time until #gls("apogee", cap: false) (optional)
],
  [
Desired deployment altitude of the #gls("main chute", cap: false)
],
  [
Time until #gls("main chute", cap: false) deployment (optional)
]
)

With this information, the user can configure the flight computer. Timers are optional and should be used only as a backup.

#enum(start: 1, tight: false,
  [
Connect the flight computer to your computer.
],
  [
Open the Configurator and connect to the board as described in Section #xref("sec:DescriptionOfConfigurator").
],
  [
In the Configuration tab, set the #gls("liftoff", cap: false) threshold. We recommend using a #gls("liftoff", cap: false) acceleration threshold of $40 m/s^2$, but make sure that it is around $20 m/s^2$ lower than your maximum expected acceleration.
],
  [
In the Configuration tab, set the main altitude to your desired height. This is the height above ground level where the #gls("main chute", cap: false) will be deployed.
],
  [
If you use a #gls("servo", cap: false) channel in either of your recovery mechanisms, it is now also the time to set the initial #gls("servo", cap: false) position.
],
  [
In the Configurator's Configuration tab, set the link phrase for your CATS Vega.
],
  [
Make sure that the Testing Mode is disabled.
],
  [
Save the settings.
],
  [
Go to the Events tab.
],
  [
For the apogee event, set your deployment mechanism as described in #xref("sec:DescriptionOfConfigurator").
],
  [
For the main deployment event, set your deployment mechanism as described in #xref("sec:DescriptionOfConfigurator").
],
  [
Save the settings.
],
  [
(Optional) Go to the Timers tab.
],
  [
(Optional) Set the Timer One start event to #gls("liftoff", cap: false) and the Timer One end event to apogee. Set the time until apogee with 1-2 seconds margin.
],
  [
(Optional) Set the Timer Two start event to #gls("liftoff", cap: false) and the Timer Two end event to main deployment. Set the time until main deployment with 10-60 seconds margin, depending on the flight time.
],
  [
Save the settings.
],
  [
Set the same link phrase on your Ground Station. Navigate to Settings, select Link Phrase, and enter the same phrase.
]
)

The flight computer is now ready to be installed in the rocket. For this flight configuration, complete the following steps:

#enum(tight: false,
  [
Mount the flight computer to your rocket.
],
  [
Connect the switch to the switch port.
],
  [
Connect the battery to the battery port.
],
  [
Connect the recovery mechanism for the apogee event.
],
  [
Connect the recovery mechanism for the main event.
],
  [
Place the rocket on the launch pad.
],
  [
Turn on the flight computer with the switch.
],
  [
The Ground Station will begin receiving data.
],
  [
Wait for the flight computer to finish calibrating and show READY on the Ground Station.
],
  [
The flight computer is now armed. Every 6 seconds, the flight computer beeps twice to indicate that it is in the #gls("Ready", cap: false) state.
],
  [
Launch your rocket!
]
)

=== How to Get the Data on Your Computer

After the flight, connect the board to a computer with a USB-C cable. The flight computer appears as a USB drive containing `.cfl` flight logs and their associated files. Copy the required files before disconnecting the board, or use the Configurator's Flight Logs page to browse, view, save, or delete onboard logs.

#cats-figure(doc-image("How To Use/Vega/StorageVega.png", width: 100%), caption: [USB drive when the CATS Vega is plugged into the user computer.])

=== Visualizing the Flight Data

#metadata(none) <sec-GeneratePlots> Open the Configurator's Flight Logs page and choose or drop a `.cfl` flight log. The Configurator plots altitude, velocity, acceleration, angular velocity, linear acceleration, pressure, state changes, and actions. Export the data as `.csv` files for further processing or as an `.html` file with interactive plots. You can also open the log in CATS Flights. A legacy Python plotting tool is described in Section #xref("sec:GeneratePlotsPython") for users who need a customizable local workflow.

=== Software Updates

#metadata(none) <sec-softwareupdates> Use the Configurator's Firmware Updates page for normal Vega updates. The complete procedure, including Ground Station and radio-receiver updates, is described in Section #xref("sec:FirmwareUpdates").

#pagebreak()

== Beeping Patterns

#metadata(none) <sec-BeepingPatterns> The CATS Vega flight computer uses beeping patterns to indicate its current state or a potential error. The tables below list the available patterns.

#cats-table(
  table(
  columns: (0.15fr, 0.25fr, 0.5fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [*State*],
  [*Description*],
  [*Pattern*],
  [Bootup],
  [The flight computer has booted up.],
  [#doc-image("How To Use/Beeping_Patterns/Beep-Bootup.png", width: 50%, alt: "Bootup beeping pattern")],
  [#gls("Calibrating", cap: false) $arrow.r$ #gls("Ready", cap: false)],
  [The flight computer has switched from the #gls("Calibrating", cap: false) state to the #gls("Ready", cap: false) state.],
  [#doc-image("How To Use/Beeping_Patterns/Beep-Moving-Ready.png", width: 50%, alt: "Calibrating to ready beeping pattern")],
  [#gls("Ready", cap: false)],
  [Calibration was successful, and the flight computer is in the #gls("Ready", cap: false) state.],
  [#doc-image("How To Use/Beeping_Patterns/Beep-Ready.png", width: 50%, alt: "Ready beeping pattern")],
  [Testing],
  [The flight computer is in testing mode. This pattern sounds only after the computer is rebooted.],
  [#doc-image("How To Use/Beeping_Patterns/Beep-Testing.png", width: 50%, alt: "Testing beeping pattern")],
  [Testing Armed],
  [The flight computer is in the armed testing state.],
  [#doc-image("How To Use/Beeping_Patterns/Beep-Testing-Armed.png", width: 50%, alt: "Testing armed beeping pattern")],
  table.hline(y: 2, stroke: 0.5pt + black)
),
  caption: [Overview of state beeping patterns.],
  continued: false,
  breakable: false,
) <tab-BeepingPatternsStates>

#note[
*Note:* The pitch axis shows only relative, unitless changes.
]

#pagebreak()

#cats-table(
  table(
  columns: (0.15fr, 0.6fr, 0.15fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [*Error*],
  [*Description*],
  [*Pattern*],
  [Filter Error],
  [A #gls("barometer", cap: false) or accelerometer error is present. If this error persists, *do not fly; the flight computer's functionality is compromised.*],
  [TBD],
  [Pyro Error],
  [A configured pyrotechnic charge is not detected. If this error persists, *do not fly; the configured recovery mechanism will not work.*],
  [TBD],
  [Log Full],
  [The flash chip is full. If you fly, the flight will not be recorded.],
  [TBD],
  [Telemetry Hot],
  [The telemetry chip has reached 60 $degree$C and may be damaged.],
  [TBD],
  [Calibration Error],
  [The calibration is faulty. *Do not fly!* Return to the rocket and reboot the flight computer to restart calibration.],
  [TBD]
),
  caption: [Overview of error beeping patterns.],
  continued: false,
  breakable: false,
) <tab-BeepingPatternsErrors>

#note[
*Note:* Bad calibrations usually happen when the flight computer is turned on and then rotated. Only turn the flight computer on once the rocket is in launch configuration and upright on the launch pad.
]

#note[
*Note:* Audible error codes are not currently implemented. Errors are shown only through telemetry on the Ground Station. Audible error codes will be added in a future update.
]

#pagebreak()

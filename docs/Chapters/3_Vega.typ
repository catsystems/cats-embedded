#import "../styles.typ": *

= CATS Vega

This section describes how the flight computer works and how to configure it for a flight. The How to Use section explains its basic features. For more detailed information, refer to Section #xref("sec:AdvancedInfo").

#cats-figure(image("../images/How To Use/Vega/Vega_HW.jpeg", width: 60%), caption: [CATS Vega])

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

#grid(columns: (63%, 1fr, 33%), [
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
], [], [
#cats-figure(image("../images/How To Use/Vega/VEGA-Hardware-optimized.png", width: 100%), caption: [CATS Vega board hardware specifications]) <fig-VegaHWSpecs>
])

== Working Principle

This section briefly introduces the operating principles needed to understand the Vega flight computer's configuration options.

=== Configurable Actions & Finite State Machine

#metadata(none) <sec-FSM> The finite state machine (#gls("FSM", cap: false)), shown in Figure #xref("fig:FSM"), controls the outputs of the Vega flight computer. When the flight computer is turned on, it starts in the Calibrating state. Every flight follows the sequence of states shown in Figure #xref("fig:FSM"). Whenever a state transition occurs, the associated event is triggered.

#cats-figure(image("../images/Working Principle/Finite_State_Machine.png", width: 13cm), caption: [Finite State Machine controlling the CATS software.]) <fig-FSM>

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

Before connecting your CATS Vega to your computer, download the Configurator from our #link("https://github.com/catsystems/cats-configurator/releases/")[releases page]. Drivers are usually not required. If your computer does not recognize the device, refer to the troubleshooting steps in our #link("https://github.com/catsystems/cats-embedded/wiki/Installation")[wiki]#footnote[#link("https://github.com/catsystems/cats-embedded/wiki/Installation")[https://github.com/catsystems/cats-embedded/wiki/Installation]].

=== Description of the Configurator

#metadata(none) <sec-DescriptionOfConfigurator> The Configurator allows you to configure the CATS System from your computer. Download the latest stable version from the #link("https://github.com/catsystems/cats-configurator/releases/")[releases page]#footnote[#link("https://github.com/catsystems/cats-configurator/releases/")[https://github.com/catsystems/cats-configurator/releases]]. Release candidates are marked accordingly and should be used only to test new features.#linebreak()#v(-1.8pt) *Home Tab*#linebreak()#v(-1.8pt) The Home tab appears when the Configurator starts. If the flight computer is already connected to your computer, select the correct communication port (label 2 in Figure #xref("fig:GUIHome")) and select Connect. If the connection times out, confirm that you selected the correct device. If several flight computers are connected, make sure that you are configuring the intended one. Until a flight computer is connected, the other tabs remain disabled (label 6), and the Configurator indicates that no board is connected (labels 5 and 7).#linebreak() If the required communication port does not appear in the dropdown menu, select Refresh (label 1). If the port still does not appear, refer to our #link("https://github.com/catsystems/cats-embedded/wiki/Installation")[wiki] for troubleshooting steps.#linebreak()

#cats-table(
  table(
  columns: (0.3fr, 0.7fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [1: Refresh Button],
  [Refreshes the communication ports shown under label 2],
  [2: Com Port Selection],
  [Selects the communication port used to connect to the CATS Vega],
  [3: Connect Button],
  [Connects to the CATS Vega],
  [4: Home Screen],
  [Displays general information about the Configurator],
  [5: Connection Status],
  [Indicates that the flight computer is not currently connected],
  [6: Navigation],
  [Remains disabled until a flight computer is connected, then provides navigation between tabs],
  [7: Connection Status],
  [Shows the connection status],
  [8: Flight Log Graph],
  [Loads a CATS flight log (.cfg) file for plotting. See Section #xref("sec:GeneratePlots") for more information]
),
  caption: [Overview of the different Settings for the Home Tab],
  continued: false,
  breakable: false,
) <tab-HomeTabOverview>

#cats-figure(image("../images/How To Use/Configurator/GUI_Home.jpg", width: 100%), caption: [Home Menu.]) <fig-GUIHome>

*Configuration Tab*#linebreak()#v(-1.8pt) This tab displays the flight computer's status and allows you to configure its parameters. Figure #xref("fig:GUIConfig") shows the Configuration tab, and the following table explains each label.

#cats-figure(image("../images/How To Use/Configurator/GUI_Config.png", width: 100%), caption: [Configuration Menu.]) <fig-GUIConfig>

#cats-table(
  table(
  columns: (0.2fr, 0.23fr, 0.57fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  table.cell(rowspan: 3, fill: luma(94%))[#strong[1: Communication Port]],
  [Refresh Button],
  [Used to refresh the communication port list],
  [Com Port],
  [Shows the communication port selected],
  [Connection Button],
  [Shows the connection status and allows you to connect or disconnect],
  table.cell(rowspan: 4, fill: luma(94%))[#strong[2: Navigation]],
  [Configuration],
  [The current tab],
  [Events],
  [Event tab, described on the next page],
  [Timers],
  [Timer tab, described on the next page],
  [#gls("CLI", cap: false)],
  [Access to the #gls("CLI", cap: false) for advanced commands, described on the next page],
  table.cell(rowspan: 7, fill: luma(94%))[#strong[3: General]],
  [Main Altitude],
  [Sets the desired height above ground level at which the #gls("main chute", cap: false) is deployed],
  [#gls("liftoff", cap: true) Detection Acceleration],
  [User-defined acceleration threshold above which liftoff is detected. We recommend 40 $m/s^2$ for most flights],
  [Initial Position #gls("servo", cap: true) 1],
  [Initial angle of the #gls("servo", cap: false) connected to #gls("servo", cap: false) port 1. If no #gls("servo", cap: false) is connected, no value is required. At startup, the flight computer drives the #gls("servo", cap: false) to this position.],
  [Initial Position #gls("servo", cap: true) 2],
  [Same as for #gls("servo", cap: false) 1 but for the second channel.],
  [Backup Config],
  [Saves a configuration file to your computer. Select Load Config to load this file onto the flight computer.],
  [Load Config],
  [Loads a previously backed-up configuration from your computer onto the flight computer.],
  [Reset Settings],
  [Set the default parameters of the flight computer.],
  table.hline(y: 1, stroke: 0.5pt + black),
  table.hline(y: 4, stroke: 0.5pt + black),
  table.hline(y: 8, stroke: 0.5pt + black)
),
  caption: [Overview of the Configuration Tab],
  continued: false,
  breakable: false,
)

#pagebreak()

#cats-table(
  table(
  columns: (0.2fr, 0.23fr, 0.57fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  table.cell(rowspan: 4, fill: luma(94%))[#strong[4: Info]],
  [System Time],
  [The elapsed system time since startup, in milliseconds],
  [State],
  [The current flight state of the board],
  [Voltage],
  [The battery voltage (if no battery is connected, a value below 1 V is shown)],
  [State Estimation],
  [The currently estimated height above ground level, velocity and acceleration],
  table.cell(rowspan: 2, fill: luma(94%))[#strong[5: Telemetry]],
  [Link Phrase],
  [A link phrase containing 4 to 16 characters. It must match the link phrase configured on the Ground Station.],
  [Enable Telemetry],
  [Set to ON or OFF to enable or disable telemetry.],
  table.cell(rowspan: 2, fill: luma(94%))[#strong[6: Testing]],
  [Enable Testing Mode],
  [If this is set to ON, the flight computer enters testing mode after its next reboot. It must then be armed through telemetry using the Ground Station. Read Section #xref("sec:Testing") thoroughly before using testing mode.],
  [Testing Phrase],
  [A testing phrase containing 4 to 16 characters. It must match the testing phrase configured on the Ground Station. Read Section #xref("sec:Testing") thoroughly before using testing mode.],
  table.cell(rowspan: 4, fill: luma(94%))[#strong[7: Hardware Info]],
  [Status],
  [Connection status of the board],
  [Board],
  [CATS Board Name],
  [Code Version],
  [Current code version],
  [Telemetry Code Version],
  [Current code version on the telemetry chip],
  table.cell(rowspan: 2, fill: luma(94%))[#strong[8: Save Settings]],
  [Save],
  [Save the current settings to the board. *Attention:* if Save is not pressed, the values are not saved to the board!],
  [Refresh],
  [Refresh the displayed values. When this button is pressed, the saved values from the flight computer are fetched and shown.],
  table.hline(y: 1, stroke: 0.5pt + black),
  table.hline(y: 5, stroke: 0.5pt + black),
  table.hline(y: 7, stroke: 0.5pt + black),
  table.hline(y: 9, stroke: 0.5pt + black),
  table.hline(y: 13, stroke: 0.5pt + black)
),
  caption: [Overview of the Configuration Tab (Cont.)],
  continued: true,
  breakable: false,
)

#metadata(none) <tab-ConfigurationTabOverview>

#pagebreak()

*Event Tab*#linebreak()#v(-1.8pt) This tab shows the actions configured for each event. Section #xref("sec:FSM") describes the events in detail. Up to eight actions can be assigned to each event: Liftoff, Burnout, Apogee, Main Deployment, Touchdown, Custom 1, and Custom 2. Select Add Action under an event to assign a new action. To avoid missing flight data, we recommend assigning Recorder On to Liftoff and Recorder Off to Touchdown.#linebreak() In the example shown in Figure #xref("fig:GUIEvents"), Liftoff, Apogee, Main Deployment, and Touchdown have mapped actions. At Liftoff, the recorder is enabled. At #gls("apogee", cap: false), #gls("pyro", cap: false) channel 1 is activated. At Main Deployment, #gls("pyro", cap: false) channel 2 is activated. At Touchdown, the recorder is disabled.#linebreak() Figure #xref("fig:GUIEventSel") shows the pop-up menu opened by selecting Add Action. Select the action type in the upper section, then configure its behavior in the lower section.#linebreak() To remove an action, select the cross beside it. Select the gear icon to reconfigure an action. Select *Save* after making changes.#linebreak()

#note[
*Note:* The custom events can only be triggered as described below in the Timers section.
]

#cats-figure(image("../images/How To Use/Configurator/GUI_Events.jpg", width: 100%), caption: [Event Menu.]) <fig-GUIEvents>

#pagebreak()

#cats-figure(image("../images/How To Use/Configurator/GUI_EventSel.jpg", width: 100%), caption: [Configuring an Event in the Event Menu.]) <fig-GUIEventSel>

*Timers*#linebreak()#v(-1.8pt) Use this tab to configure the four available timers. Enable or disable each timer with the yellow button in the upper-right corner of its panel. #linebreak() After enabling a timer (Timer 1 in Figure #xref("fig:GUITimers")), configure its start event, duration in milliseconds, and end event. In the example, the start event is #gls("liftoff", cap: true), the duration is 10000 ms, and the end event is #gls("apogee", cap: true). At liftoff, a 10-second timer starts and triggers the #gls("apogee", cap: false) event when it expires. This triggers the event without placing the flight computer in the Apogee state.#linebreak()

#note[
*Note:* Events are unique, meaning that if a timer is used to trigger apogee, only one apogee event will be thrown by the flight computer. It will either be the timer or the event from the estimation, whichever is thrown first.
]

*Custom Events*#linebreak()#v(-1.8pt) Two custom events are available. To activate one, configure a timer whose end event is the desired custom event. This allows you to execute actions at an arbitrary time.#linebreak() This is particularly useful for payload experiments. For example, to actuate a device 10 seconds after apogee, configure a timer that starts at apogee, runs for 10 seconds, and triggers Custom 1. Then assign the desired actions to that event.

#pagebreak()

#cats-figure(image("../images/How To Use/Configurator/GUI_Timers.jpg", width: 100%), caption: [Timer Menu.]) <fig-GUITimers>

*CLI*#linebreak()#v(-1.8pt) The CLI tab allows the user to send commands directly to the CATS board. Section #xref("sec:CLI") explains all supported commands. Figure #xref("fig:GUICLI") shows the CLI.

#cats-figure(image("../images/How To Use/Configurator/GUI_CLI.jpg", width: 100%), caption: [Command Line Interface.]) <fig-GUICLI>

=== Mounting

#grid(columns: (60%, 1fr, 33%), [
The CATS Vega *does not* require a specific mounting orientation. The system automatically detects the gravity vector for internal state estimation, so you can mount the board in any orientation.#linebreak() The board has a length of 100 mm, a width of 33 mm and a total height of 15 mm. Three mounting holes secure the system to the rocket. The mounting holes are spaced 60 mm by 27 mm and are designed for M3 screws. Use spacers to prevent the electronics from touching the rocket. Download the system's 3D files from our #link("https://github.com/catsystems/cats-hardware/tree/main/CATS-Vega/3D")[GitHub repository]#footnote[#link("https://github.com/catsystems/cats-hardware/tree/main/CATS-Vega/3D")[https://github.com/catsystems/cats-hardware/tree/main/CATS-Vega/3D]].#linebreak() For reliable radio reception during flight, pay close attention to the area surrounding each antenna. Install the CATS flight computer in a radio-transparent section of the rocket, such as fiberglass or cardboard. Do not install it in a carbon-fiber section, which blocks #gls("RF", cap: false) signals. Ensure that the onboard #gls("patch antenna", cap: false) has a clear view of the sky for optimal #gls("GNSS", cap: false) reception, and keep the telemetry antenna away from metal objects.

After power-up, the system detects the up direction once it is stable. A beeping pattern and the Ground Station indicate when the flight computer enters the #gls("Ready", cap: false) state. In this state, the flight computer is armed and waiting for #gls("liftoff", cap: false). Do not move the rocket, and follow all safety guidelines. At this stage, the flight computer can be disarmed only by switching it off. For more information about calibration, refer to Sections #xref("sec:EstAlg") and #xref("sec:FSM").#linebreak()
], [], [
#cats-figure(image("../images/How To Use/Vega/Mounting_Img.png", width: 80%), caption: [CATS Vega board with the mounting hole and dimensions.])
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

After the flight, connect the board to a computer with a USB-C cable. The flight computer appears as a USB drive, allowing you to drag and drop the flight data onto your desktop. You can also drag the flight log directly into the Configurator to plot it.#linebreak()

#cats-figure(image("../images/How To Use/Vega/StorageVega.png", width: 100%), caption: [USB drive when the CATS Vega is plugged into the user computer.])

=== Visualizing the Flight Data

#metadata(none) <sec-GeneratePlots> To visualize flight data, open the Configurator and drag and drop the CATS flight log (`.cfg`). The Configurator immediately plots the important data: altitude, velocity, acceleration, angular velocity (x, y, and z), linear acceleration (x, y, and z), pressure, state changes, and actions.#linebreak() You can export the plots as `.html` files, which retain their zoom functionality when opened in a browser, or as `.csv` files for further processing.#linebreak()#v(-1.8pt) A legacy Python plotting tool is also available as a reference for custom implementations. It is described briefly in Section #xref("sec:GeneratePlotsPython").

=== Software Updates

#metadata(none) <sec-softwareupdates> The software is continuously improved, so install each new update when it is released. Updates are announced on our #link("https://discord.gg/r7ErmSNvsy")[Discord server]#footnote[#link("https://discord.gg/r7ErmSNvsy")[https://discord.gg/r7ErmSNvsy]]. To update the software, follow these steps:

#enum(tight: false,
  [
Download and install the #link("https://www.st.com/en/development-tools/stm32cubeprog.html")[STM Programmer]#footnote[#link("https://www.st.com/en/development-tools/stm32cubeprog.html")[https://www.st.com/en/development-tools/stm32cubeprog.html]] (STM32CubeProg).
],
  [
Connect the CATS Vega with a USB-C cable.
],
  [
Start the Configurator, select the correct COM port, select Connect, and open the #gls("CLI", cap: false) tab described in Section #xref("sec:DescriptionOfConfigurator").
],
  [
In the #gls("CLI", cap: false), enter #text(font: "DejaVu Sans Mono", size: 0.9em)[bl] and send the command.
],
  [
The CATS Vega will disconnect. Close the Configurator.
],
  [
Start the STM32 Programmer.
],
  [
In the upper-right corner of the programmer, select USB (Figure #xref("fig:SWUpdateInit")).
],
  [
On the right, select the appropriate USB port (Figure #xref("fig:SWUpdateUSB")).
],
  [
Select Connect (Figure #xref("fig:SWUpdateUSB")).
],
  [
Confirm that the upper-right corner shows that the programmer is connected to the board (Figure #xref("fig:SWUpdateConnected")).
],
  [
In the left navigation panel, select Erasing & Programming (Figure #xref("fig:SWUpdateProgram")).
],
  [
In the File path field, select the firmware file to flash (the filename ends in `.bin`). The latest release of the CATS software is available #link("https://github.com/catsystems/cats-embedded/releases")[here]#footnote[#link("https://github.com/catsystems/cats-embedded/releases")[https://github.com/catsystems/cats-embedded/releases]] (Figure #xref("fig:SWUpdateProgram")).
],
  [
Select Start Programming (Figure #xref("fig:SWUpdateProgram")).
],
  [
Wait for the "File Download Complete" pop-up (Figure #xref("fig:SWUpdateProgramFinished")).
],
  [
Disconnect and reconnect the board. Start the Configurator and verify that the version number has been updated.
],
  [
You've successfully updated the software!
]
)

#warning[
*Warning:* The steps above update only the Vega's flight-control software (#text(font: "DejaVu Sans Mono", size: 0.9em)[flight\_computer.bin]). The telemetry code (#text(font: "DejaVu Sans Mono", size: 0.9em)[telemetry.bin]) resides on another chip and cannot be updated via USB. Updating the telemetry software requires an STLINK-V3MINI debugger and a TC2030-IDC-NL 6-pin connector.
]

#cats-figure(
  stack(dir: ttb, spacing: 8pt,
    [#align(center)[#block(width: 100%)[#subfigure(image("../images/How To Use/SoftwareUpdate/Programmer-Init.jpg", width: 100%), [Open the dropdown menu and select USB. The panel shown in the next image opens on the right.], "a") <fig-SWUpdateInit>]]],
    [#align(center)[#block(width: 100%)[#subfigure(image("../images/How To Use/SoftwareUpdate/Programmer-USB.png", width: 100%), [Select the displayed USB port in the Port field, then select Connect.], "b") <fig-SWUpdateUSB>]]]
  ),
  caption: [Flashing new software to the board.],
  continued: false,
)

#cats-figure(
  stack(dir: ttb, spacing: 8pt,
    [#align(center)[#block(width: 100%)[#subfigure(image("../images/How To Use/SoftwareUpdate/Programmer-Connected.png", width: 100%), [Confirm that the Connected status appears in the upper-right corner.], "c") <fig-SWUpdateConnected>]]],
    [#align(center)[#block(width: 100%)[#subfigure(image("../images/How To Use/SoftwareUpdate/Programmer-Erase-Programm.png", width: 100%), [Select Erasing & Programming on the left. Use Browse to select the appropriate `.bin` file, then select Start Programming.], "d") <fig-SWUpdateProgram>]]]
  ),
  caption: [Flashing new software to the board (cont.).],
  continued: true,
)

#cats-figure(
  stack(dir: ttb, spacing: 8pt,
    [#align(center)[#block(width: 100%)[#subfigure(image("../images/How To Use/SoftwareUpdate/Programmer_Finished.JPG", width: 100%), [This message appears when the firmware has been flashed successfully.], "e") <fig-SWUpdateProgramFinished>]]]
  ),
  caption: [Flashing new software to the board (cont.).],
  continued: true,
) <fig-SWUpdate>

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
  [#image("../images/How To Use/Beeping_Patterns/Beep-Bootup.png", width: 50%)],
  [#gls("Calibrating", cap: false) $arrow.r$ #gls("Ready", cap: false)],
  [The flight computer has switched from the #gls("Calibrating", cap: false) state to the #gls("Ready", cap: false) state.],
  [#image("../images/How To Use/Beeping_Patterns/Beep-Moving-Ready.png", width: 50%)],
  [#gls("Ready", cap: false)],
  [Calibration was successful, and the flight computer is in the #gls("Ready", cap: false) state.],
  [#image("../images/How To Use/Beeping_Patterns/Beep-Ready.png", width: 50%)],
  [Testing],
  [The flight computer is in testing mode. This pattern sounds only after the computer is rebooted.],
  [#image("../images/How To Use/Beeping_Patterns/Beep-Testing.png", width: 50%)],
  [Testing Armed],
  [The flight computer is in the armed testing state.],
  [#image("../images/How To Use/Beeping_Patterns/Beep-Testing-Armed.png", width: 50%)],
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

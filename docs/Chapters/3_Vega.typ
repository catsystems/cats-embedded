#import "../styles.typ": *

= CATS Vega

This section describes how the flight computer works and how you can set it up for your flight. The How to Use section should give you all the information about the basic features of the flight computer. In case you want more detailed information, Section #xref("sec:AdvancedInfo") will give you more insights.

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
  [100 x 33 x 21 mm (without the antenna)],
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

Here, we give a quick overview of the hardware and shows the location of all the ports. These ports are marked in Figure #xref("fig:VegaHWSpecs"), matching the numbers in the list.#linebreak()#v(-1.8pt)

#grid(columns: (63%, 1fr, 33%), [
#enum(tight: false,
  [
*Switch port*; A manual switch needs to be connected between the two connectors.
],
  [
*Battery port*; A battery needs to be connected between these two ports. Pay attention to the right polarity.
],
  [
*Buzzer*; Beeps the readiness of the flight computer. The beeping patterns are explained in section #xref("sec:BeepingPatterns").
],
  [
*Status LEDs*; If there is power, the POWER LED is enabled. The STATUS LED blinks when everything is going well.
],
  [
*USB Connector*; The connector is on the other side of the board.
],
  [
*Test Button*; If this button is pressed while booting up the board, the board will enter test mode if a testing passphrase is set.
],
  [
*Servo Connector*; This connector fits the standard servo connectors. Two servos can be connected to this connector.
],
  [
*Telemetry LEDs*; The #gls("GNSS", cap: false) LED blinks every time #gls("GNSS", cap: false) Coordinates have been received. The LINK LED blinks when a connection to the Ground station was established.
],
  [
*Low Level #gls("I/O", cap: false) and #gls("UART", cap: false) connector*; External hardware can be connected to the #gls("CATS", cap: false) board to transmit data over this connector.
],
  [
*#gls("pyro", cap: true) LEDs*; These red LEDs are turned on when continuity of the #gls("pyro", cap: false) channel is detected.
],
  [
*#gls("pyro", cap: true) Channel 1*; A #gls("pyro", cap: false) charge can be connected to this connector.
],
  [
*#gls("pyro", cap: true) Channel 2*; A #gls("pyro", cap: false) charge can be connected to this connector.
],
  [
*Antenna Connector*; An antenna needs to be connected to this connector to send the data to the ground station from the #gls("CATS", cap: false) Vega.
]
)
], [], [
#cats-figure(image("../images/How To Use/Vega/VEGA-Hardware-optimized.png", width: 100%), caption: [CATS Vega board hardware specifications]) <fig-VegaHWSpecs>
])

== Working Principle

In this section the working principle is briefly introduced as some understanding of it is essential to properly understand the configuration options of the Vega flight computer.

=== Configurable Actions & Finite State Machine

#metadata(none) <sec-FSM> The system that controls all the different actuations of the VEGA flight computer is the finite state machine (#gls("FSM", cap: false)), shown in Figure #xref("fig:FSM"). When the flight computer is turned on, it is initialized to the Calibration state. Every rocket flight follows the flight states outlined in Figure #xref("fig:FSM"). Whenever a flight transition occurs, the associated event is thrown.

#cats-figure(image("../images/Working Principle/Finite_State_Machine.png", width: 13cm), caption: [Finite State Machine controlling the CATS software.]) <fig-FSM>

Each event can be used to trigger some actions (i.e., a #gls("pyro", cap: false) channel, a #gls("servo", cap: false) channel, a timer, etc.). The user configures which actions are triggered during which event using the configurator. Configuration of actions is described in section #xref("sec:DescriptionOfConfigurator").

#cats-table(
  table(
  columns: (0.5fr, 0.5fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: none,
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [#gls("Calibrating", cap: false) $arrow.r$ Testing],
  [Telemetry input to put it into testing mode. Only works, if the testing mode was activated over the configurator.],
  [#gls("Calibrating", cap: false) $arrow.r$ #gls("Ready", cap: false)],
  [#gls("IMU", cap: false) (gyroscope and linear acceleration) readings are constant for 10 seconds.],
  [#gls("Ready", cap: false) $arrow.r$ Thrusting],
  [The measured acceleration in any direction is larger than the user defined acceleration value for 0.1 seconds.],
  [Thrusting $arrow.r$ Coasting],
  [The measured acceleration in the "up" direction is smaller than 0 $m/s^2$ for 0.1 seconds],
  [Coasting $arrow.r$ Drogue],
  [The estimated velocity needs to be smaller than 0 $m/s$ for 0.3 seconds],
  [Drogue $arrow.r$ Main],
  [The estimated height is smaller than the user defined height for 0.3 seconds],
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

Based on the thrown events, actions can be mapped onto those events. For each event, a total of 8 actions can be mapped. This gives you the option to do many different things for any different application. Examples include:

#list(tight: false,
  [
Enabling a camera at #gls("liftoff", cap: false) using a #gls("pyro", cap: false) channel,
],
  [
Actuating a solenoid valve for two seconds only using a #gls("pyro", cap: false) channel,
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
  [Low Level I/O],
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
*Note:* The _PREFILLING_ recorder option continuously accumulates log elements in a buffer. Once the recorder transitions into _ON_ mode, the elements in the buffer will be written to the flash chip. This info can be used to analyze the initial ignition sequence and thrust build-up.
]

How actions can be configured is shown in section #xref("sec:Examples").

#pagebreak()

== How to Use

Now that an overview of the hardware and software was given, we explain here how to set up your flight computer, how to mount it in your rocket, how to do software updates, how to generate plots of your flight data and much more.

=== Connection to Your Computer

Before you connect your #gls("CATS", cap: false) Vega to your computer, we recommend you download the configurator from our #link("https://github.com/catsystems/cats-configurator/releases/")[website]. Usually, no drivers are required to get started but if the device is not recognized by your computer please refer to our #link("https://github.com/catsystems/cats-embedded/wiki/Installation")[wiki]#footnote[#link("https://github.com/catsystems/cats-embedded/wiki/Installation")[https://github.com/catsystems/cats-embedded/wiki/Installation]] where we describe some troubleshooting steps.

=== Description of the Configurator

#metadata(none) <sec-DescriptionOfConfigurator> The configurator allows you to configure the #gls("CATS", cap: false) system from your computer. The configurator can be downloaded #link("https://github.com/catsystems/cats-configurator/releases/")[here]#footnote[#link("https://github.com/catsystems/cats-configurator/releases/")[https://github.com/catsystems/cats-configurator/releases]]. Make sure you download the latest stable version. Release candidates are marked as such and should only be used for testing new features.#linebreak()#v(-1.8pt) *Home Tab*#linebreak()#v(-1.8pt) Once the configurator starts, the home tab is shown. If the flight computer is already plugged into your computer just select the correct communication port (label 2 in the Figure #xref("fig:GUIHome")) and press connect. If the connection times out, double check that you are attempting to connect with the correct device. If you have multiple flight computers connected make sure you are configuring the desired one. As long as no flight computer is connected, the other tabs are disabled (label 6), and the configurator shows that no board is connected (label 5 and 7).#linebreak() If no communication port / the wrong communication port is shown in the drop down menu, press refresh (label 1). If your communication port is still not showing up, refer to our #link("https://github.com/catsystems/cats-embedded/wiki/Installation")[wiki] for some troubleshooting steps.#linebreak()

#cats-table(
  table(
  columns: (0.3fr, 0.7fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [1: Refresh Button],
  [Used to refresh the shown communication ports in label 2],
  [2: Com Port Selection],
  [Selection of the communication port to connect to the #gls("CATS", cap: false) Vega],
  [3: Connect Button],
  [Press to connect to the #gls("CATS", cap: false) Vega],
  [4: Home Screen],
  [Some general information about the Configurator],
  [5: Connection Status],
  [Information box, tells you that the flight computer is currently not connected],
  [6: Navigation],
  [Greyed out as long as no flight computer is connected. Once it is connected, used to navigate between the tabs.],
  [7: Connection Status],
  [Shows the connection status],
  [8: Flight Log Graph],
  [Load a cats flight log (.cfg) flie to plot your flight. More information in Section #xref("sec:GeneratePlots")]
),
  caption: [Overview of the different Settings for the Home Tab],
  continued: false,
  breakable: false,
) <tab-HomeTabOverview>

#cats-figure(image("../images/How To Use/Configurator/GUI_Home.jpg", width: 100%), caption: [Home Menu.]) <fig-GUIHome>

*Configuration Tab*#linebreak()#v(-1.8pt) This tab shows you the status of the flight computer and lets you configure various parameters. Figure #xref("fig:GUIConfig") shows the Configuration tab. In the following Table, the labels are explained in more detail.

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
  [Shows the connection status and allows to connect/disconnect],
  table.cell(rowspan: 4, fill: luma(94%))[#strong[2: Navigation]],
  [Configuration],
  [The current tab],
  [Events],
  [Event tab, described on next page],
  [Timers],
  [Timer tab, described on next page],
  [#gls("CLI", cap: false)],
  [Access to the #gls("CLI", cap: false), for advanced commands, described on next page],
  table.cell(rowspan: 7, fill: luma(94%))[#strong[3: General]],
  [Main Altitude],
  [Set the desired height above ground level where the #gls("main chute", cap: false) should be deployed],
  [#gls("liftoff", cap: true) Detection Acceleration],
  [Acceleration Threshold above which Liftoff should be detected. We recommend a value of 40 $m/s^2$ for most flights],
  [Initial Position #gls("servo", cap: true) 1],
  [Initial #gls("servo", cap: false) angle of the #gls("servo", cap: false) connected to #gls("servo", cap: false) port 1. If you do not use #gls("servo", cap: false)s, no settings need to be set here. The flight computer will actively steer to that position at start-up.],
  [Initial Position #gls("servo", cap: true) 2],
  [Same as for #gls("servo", cap: false) 1 but for the second channel.],
  [Backup Config],
  [Used to save a configuration file  to your computer. This file can then be set on the flight computer by pressing load config.],
  [Load Config],
  [Load a configuration, previously backed up on your computer, to the flight computer.],
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
  [The current system time since boot up in ms],
  [State],
  [The current flight state of the board],
  [Voltage],
  [The battery voltage (if no battery is applied, a value \< 1 is shown],
  [State Estimation],
  [The currently estimated height above ground level, velocity and acceleration],
  table.cell(rowspan: 2, fill: luma(94%))[#strong[5: Telemetry]],
  [Link Phrase],
  [4 - 16 character link phrase which needs to be set and needs to match the link phrase set on the ground station for a connection.],
  [Enable Telemetry],
  [Set to ON or OFF to enable or disable telemetry.],
  table.cell(rowspan: 2, fill: luma(94%))[#strong[6: Testing]],
  [Enable Testing Mode],
  [If this is set to ON, the next time the flight computer reboots, it enters testing mode. It then needs to armed by telemetry using the ground station. Make sure to read section #xref("sec:Testing") thoroughly for more information about testing.],
  [Testing Phrase],
  [4 - 16 character testing phrase which should be the same than the one configured on the ground station. Make sure to read Section #xref("sec:Testing") thoroughly for more information about testing.],
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

*Event Tab*#linebreak()#v(-1.8pt) This tab shows you the configuration of the individual event. Each event is described more closely in Section #xref("sec:FSM"). For each event (Liftoff, Burnout, Apogee, Main Deployment, Touchdown, Custom 1 and Custom 2) up to eight actions can be mapped. On each Event, add Action can be pressed to add an action to the flight computer. We recommend keeping the recorder log and recorder off event at Liftoff and Touchdown to not miss any data.#linebreak() In the example (Figure #xref("fig:GUIEvents")), the events Liftoff, Apogee, Main and Touchdown have mapped events, meaning that the flight computer only acts when those events are triggered. At Liftoff, the recorder is enabled. At #gls("apogee", cap: false), #gls("pyro", cap: false) channel 1 is turned on. At Main deployment #gls("pyro", cap: false) 2 is enabled. At Touchdown, the recorder is disabled.#linebreak() Figure #xref("fig:GUIEventSel") shows the pop up menu which is opened when "Add Action" is pressed. The user then can click on the upper part, which describes what action should be taken and then the lower part, which describes what that particular action should do.#linebreak() To remove events again, press the little cross right of the event you want to delete. Press on the little gear on the left to reconfigure that event. Don't forget to hit *save* once you've done some changes!#linebreak()

#note[
*Note:* The custom events can only be triggered as described below in the Timers section.
]

#cats-figure(image("../images/How To Use/Configurator/GUI_Events.jpg", width: 100%), caption: [Event Menu.]) <fig-GUIEvents>

#pagebreak()

#cats-figure(image("../images/How To Use/Configurator/GUI_EventSel.jpg", width: 100%), caption: [Configuring an Event in the Event Menu.]) <fig-GUIEventSel>

*Timers*#linebreak()#v(-1.8pt) This tab is used to configure the individual timers. There are a total of four timers and each timer can be enabled and disabled through the little yellow button on the top right of each timer field. #linebreak() Once a timer was enabled (Figure #xref("fig:GUITimers"), Timer 1), the start event can be defined, the duration in ms and the end event which shall be triggered. In the example picture the start event was chosen to be #gls("liftoff", cap: true), the duration 10000 ms and the end event #gls("apogee", cap: true). This means that at liftoff, a timer of 10 second is started which triggers the #gls("apogee", cap: false) event at its end. However, this does not put the flight computer in the apogee phase! It only triggers that event.#linebreak()

#note[
*Note:* Events are unique, meaning that if a timer is used to trigger apogee, only one apogee event will be thrown by the flight computer. It will either be the timer or the event from the estimation, whichever is thrown first.
]

*Custom Events*#linebreak()#v(-1.8pt) As events are unique, two custom events are provided to the user. To activate these custom events, a timer must be started with the end event being the custom event. This allows to apply any actions at an arbitrary point in time.#linebreak() This can be in particular interesting for payload experiments. If you would like to actuate something 10 seconds after apogee, you can set a timer starting from apogee, running for 10 seconds and activating custom event 1. Then you can map the desired actions to that event.

#pagebreak()

#cats-figure(image("../images/How To Use/Configurator/GUI_Timers.jpg", width: 100%), caption: [Timer Menu.]) <fig-GUITimers>

*CLI*#linebreak()#v(-1.8pt) The CLI tab allows the user to send manual commands to the CATS board. All supported commands are explained in detail in section #xref("sec:CLI"). A screenshot of the CLI is shown in Figure #xref("fig:GUICLI").

#cats-figure(image("../images/How To Use/Configurator/GUI_CLI.jpg", width: 100%), caption: [Command Line Interface.]) <fig-GUICLI>

=== Mounting

#grid(columns: (60%, 1fr, 33%), [
The #gls("CATS", cap: false) Vega *does not* require a specific mounting orientation. The gravity vector is automatically detected and used for the internal state estimation. Therefore you can mount it in any way you see fit.#linebreak() The board has a length of 100 mm, a width of 33 mm and a total height of 15 mm. Three mounting holes are present on the board to secure the system safely to your rocket. The mounting holes are 60 mm by 27 mm apart. The holes are designed for M3 screws and some additional spacers are recommended to keep the electronics from touching your rack. The 3D files of the system can be downloaded from our #link("https://github.com/catsystems/cats-hardware/tree/main/CATS-Vega/3D")[github]#footnote[#link("https://github.com/catsystems/cats-hardware/tree/main/CATS-Vega/3D")[https://github.com/catsystems/cats-hardware/tree/main/CATS-Vega/3D]].#linebreak() In order for you to get a good radio reception during the flight, pay close attention to the surroundings of your antennas. Make sure the #gls("CATS", cap: false) flight computer is in a radio-transparent section of your rocket (e.g., glass fiber or cardboard). Do not mount your system in a carbon fiber section as it will block all #gls("RF", cap: false) signals. Make sure the #gls("patch antenna", cap: false) on the board has a view of the sky for optimal #gls("GNSS", cap: false) reception. Keep your telemetry antenna away from any metallic objects.

When the system is powered up, the up direction is automatically detected as soon as the system is stable. It will then beep that it is in the #gls("Ready", cap: false) state and also show it on the ground station. Once the flight computer is in #gls("Ready", cap: false) mode, it is armed and waiting for #gls("liftoff", cap: false). Do not move the rocket and make sure you follow safety guidelines. Disarming the flight computer at that stage can only be done by turning it off. For more information regarding calibration refer to sections #xref("sec:EstAlg") and #xref("sec:FSM").#linebreak()
], [], [
#cats-figure(image("../images/How To Use/Vega/Mounting_Img.png", width: 80%), caption: [CATS Vega board with the mounting hole and dimensions.])
])

#warning[
*Warning:* The flight computer should only be powered up once the rocket is upright on the launch pad. The calibration is done only once, as soon as no motion is detected after boot up.
]

=== Battery, Switch and Actuators

The #gls("CATS", cap: false) Vega has one battery port, one switch port, two #gls("pyro", cap: false) channels, two #gls("servo", cap: false) channels and one low level #gls("I/O", cap: false). In the following, each port is briefly explained. Table #xref("tab:HowToAct") shows an overview of the most important parameters. For further information about the board hardware refer to section #xref("sec:VegaHW"). There, also a picture is shown where each #gls("I/O", cap: false) is marked on the board itself (Figure #xref("fig:VegaHWSpecs"))#linebreak()#linebreak()#v(1.8pt)

*Battery Port*#linebreak()#v(-1.8pt) The battery port supports voltages between 7 and 25 Volts. For LiPo and Li-ion batteries this translates to 2 to 6 cell battery packs. The battery port is reverse-polarity protected.#linebreak()#linebreak()#v(1.8pt)

*Switch Port*#linebreak()#v(-1.8pt) The switch port allows the user to add a mechanical switch to the system. If this switch is turned off, the system is fully disconnected from power.#linebreak()

#note[
*Note:* The battery current is routed through the switch. Make sure that the wires and the switch are rated for the currents required.
]

*Pyro Channels*#linebreak()#v(-1.8pt) The #gls("pyro", cap: false) channels apply the battery voltage with around 1 V of dropout to the connected circuitry, meaning that the pyro channel will have 1 V less than the applied battery voltage. Usually an electric match is connected to the channel to ignite a black powder charge. However, the channels can also be used to power other devices. For example the #gls("pyro", cap: false) channels can be used to actuate solenoid valves (with an external flyback diode), power cameras or other electronic circuits. By default the maximum current which can be drawn is around 1 A continuous. The reason for that is that the channels are short circuit protected via a resettable PTC fuse. This is more than enough to ignite electric matches before the fuse reduces the current. If the load connected requires more current, the fuse can be bypassed with a solder jumper on the back of the board. In this configuration it is recommended to stay below 5 A continuous or 20 A burst. Be very cautious what you do from this point forward as a short on the channel is going to damage the board!#linebreak()

#warning[
*Warning:* The pyro channels are short circuit protected with a resettable PTC fuse. If your connected circuit requires more than 1 A a solder bridge on the back of the board needs to be closed.
]

*#gls("servo", cap: true) Channels*#linebreak()#v(-1.8pt) The #gls("servo", cap: false) channels can be used to actuate #gls("PWM", cap: false) #gls("servo", cap: false)s. An onboard voltage regulator reduces the battery voltage to 5 V which is used to power your servos. The power rails for the microcontroller is kept completely separate from the 5 V #gls("Power Supply", cap: false), therefore, even a short circuit on that power rail has no impact on the system. A maximum current of 3 A can be drawn. A #gls("PWM", cap: false) signal is always applied to the #gls("servo", cap: false) channel and the end points can be changed in the configurator.#linebreak()#linebreak()#v(1.8pt)

*Low Level #gls("I/O", cap: false)*#linebreak()#v(-1.8pt) The low level #gls("I/O", cap: false) can be used to propagate a signal to another system. The voltage level is 3.3 V and the pin is directly connected to the microcontroller therefore the #gls("I/O", cap: false) should *only* be used for signal transmission and not to actuate any recovery mechanism!

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
  [Connect up to two #gls("pyro", cap: false)s or other devices],
  [Battery voltage / 5 A],
  [#gls("servo", cap: true) Channels],
  [Used for #gls("servo", cap: false) actuation, up to two #gls("servo", cap: false)s],
  [5 V / 3 A],
  [Low Level #gls("I/O", cap: false)],
  [Only used for signal transmission, no actuation!],
  [3.3 V / 10 mA],
  table.hline(y: 2, stroke: 0.5pt + black)
),
  caption: [Overview of the I/Os],
  continued: false,
  breakable: false,
) <tab-HowToAct>

#pagebreak()

=== Setting up the Minimal Flight Configuration

For every flight a couple of parameters need to be set for nominal flight performance. For this, some preparation from the user is needed. In particular, the user needs to know:#linebreak()

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
Desired deployment height of the #gls("main chute", cap: false) deployment
],
  [
Time until #gls("main chute", cap: false) deployment (optional)
]
)

With this in hand, the user can configure their flight computer. The timers are optional, as they should only be used as a backup.

#enum(start: 1, tight: false,
  [
Connect the flight computer to the user computer.
],
  [
Open the configurator and connect to the board as described in section #xref("sec:DescriptionOfConfigurator").
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
Set the link phrase of your CATS Vega by navigating to the configuration tab in the Configurator and setting your link phrase.
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
Set the same link phrase on your ground station (Navigate to Settings, link phrase and type in the same password).
]
)

At this point you are ready to set up the flight computer in your rocket. With this particular flight configuration you would need to do the following steps:

#enum(tight: false,
  [
Mount the flight computer to your rocket.
],
  [
Plug in the switch into the switch port.
],
  [
Plug in the battery into the battery port.
],
  [
Plug in the recovery mechanism for the apogee event.
],
  [
Plug in the recovery mechanism for the main event.
],
  [
Place the rocket on the launch pad.
],
  [
Turn on the flight computer via the switch.
],
  [
At this point, you will get data to your ground station.
],
  [
The flight computer is calibrating, wait until it shows READY on the ground station.
],
  [
The flight computer is now armed. Every 6 seconds, the flight computer beeps twice to indicate that it is in the #gls("Ready", cap: false) state.
],
  [
Launch your rocket!
]
)

=== How to Get the Data on Your Computer

After your flight, just plug your board to a computer with a USB-C cable. The flight computer should act just like a USB drive and you can simply drag and drop your flight data onto your desktop. The flight log, you can also directly drag into the configurator for it to be plotted.#linebreak()

#cats-figure(image("../images/How To Use/Vega/StorageVega.png", width: 100%), caption: [USB drive when the CATS Vega is plugged into the user computer.])

=== Visualizing the Flight Data

#metadata(none) <sec-GeneratePlots> To visualize your flight data, just open up the Configurator and drag and drop the CATS flight log (.cfg). All important information is then instantly plotted in the Configurator (height, velocity, acceleration, angular velocity (x,y,z), linear accelerations (x,y,z), pressure, state changes and actions).#linebreak() You can then also export the plots as .html files, which can conveniently be opened in any browser, keeping their zoom-in functions or as .csv files to do some further processing.#linebreak()#v(-1.8pt) There is also a python plotting tool which you can have a look at as a reference if you want to implement some custom things. This tool is explained briefly in section #xref("sec:GeneratePlotsPython").

=== Software Updates

#metadata(none) <sec-softwareupdates> As the software is continuously improved, software updates are a crucial part and should be done whenever a new update is released. Updates are always announced on our #link("https://discord.gg/r7ErmSNvsy")[Discord server]#footnote[#link("https://discord.gg/r7ErmSNvsy")[https://discord.gg/r7ErmSNvsy]]. To update the software follow these steps:

#enum(tight: false,
  [
Download and install the #link("https://www.st.com/en/development-tools/stm32cubeprog.html")[STM Programmer]#footnote[#link("https://www.st.com/en/development-tools/stm32cubeprog.html")[https://www.st.com/en/development-tools/stm32cubeprog.html]] (STM32CubeProg).
],
  [
Plug in the #gls("CATS", cap: false) Vega using a USB-C cable.
],
  [
Start the configurator, connect to the board by selecting the right com port, press the connect button and go to the #gls("CLI", cap: false) tab (described in section #xref("sec:DescriptionOfConfigurator")).
],
  [
In the #gls("CLI", cap: false) type #text(font: "DejaVu Sans Mono", size: 0.9em)[bl] and send the command.
],
  [
The #gls("CATS", cap: false) will disconnect. Close the configurator.
],
  [
Start the STM32 Programmer.
],
  [
In the programmer, on the top right, select USB (Figure #xref("fig:SWUpdateInit")).
],
  [
On the right, select a valid USB port (Figure #xref("fig:SWUpdateUSB")).
],
  [
Click connect (Figure #xref("fig:SWUpdateUSB")).
],
  [
Now, on the top right it should be shown that the programmer is connected to the board (Figure #xref("fig:SWUpdateConnected")).
],
  [
Select Erasing & Programming on the left in the navigation (Figure #xref("fig:SWUpdateProgram")).
],
  [
On the file path, select the program which you want to flash (the file ends with .bin). The latest release of the #gls("CATS", cap: false) software is found #link("https://github.com/catsystems/cats-embedded/releases")[here]#footnote[#link("https://github.com/catsystems/cats-embedded/releases")[https://github.com/catsystems/cats-embedded/releases]] (Figure #xref("fig:SWUpdateProgram")).
],
  [
Click start program (Figure #xref("fig:SWUpdateProgram")).
],
  [
Wait until you see the pop up "File Download Complete" (Figure #xref("fig:SWUpdateProgramFinished")).
],
  [
Unplug and plug the board back in. Start the configurator and verify that the the version number updated.
],
  [
You've successfully updated the software!
]
)

#warning[
*Warning:* The steps listed above can be used to only update the Vega's flight control software (#text(font: "DejaVu Sans Mono", size: 0.9em)[flight\_computer.bin]). The telemetry code (#text(font: "DejaVu Sans Mono", size: 0.9em)[telemetry.bin]) resides on another chip and cannot be updated via USB. Updating telemetry software requires a STLINK-V3MINI debugger and a TC2030-IDC-NL 6-pin connector.
]

#cats-figure(
  stack(dir: ttb, spacing: 8pt,
    [#align(center)[#block(width: 100%)[#subfigure(image("../images/How To Use/SoftwareUpdate/Programmer-Init.jpg", width: 100%), [Click on the drop down menu and select USB. This opens the right tab as shown on the picture right.], "a") <fig-SWUpdateInit>]]],
    [#align(center)[#block(width: 100%)[#subfigure(image("../images/How To Use/SoftwareUpdate/Programmer-USB.png", width: 100%), [Select the shown USB port in the "port" setting and click connect.], "b") <fig-SWUpdateUSB>]]]
  ),
  caption: [Flashing new software to the board.],
  continued: false,
)

#cats-figure(
  stack(dir: ttb, spacing: 8pt,
    [#align(center)[#block(width: 100%)[#subfigure(image("../images/How To Use/SoftwareUpdate/Programmer-Connected.png", width: 100%), [Make sure the connected tag is shown (top right).], "c") <fig-SWUpdateConnected>]]],
    [#align(center)[#block(width: 100%)[#subfigure(image("../images/How To Use/SoftwareUpdate/Programmer-Erase-Programm.png", width: 100%), [Press on the left on Erasing & Programming. Then select the appropriate .bin file using the browse button. Once chosen, program the board by pressing "Start Program".], "d") <fig-SWUpdateProgram>]]]
  ),
  caption: [Flashing new software to the board (cont.).],
  continued: true,
)

#cats-figure(
  stack(dir: ttb, spacing: 8pt,
    [#align(center)[#block(width: 100%)[#subfigure(image("../images/How To Use/SoftwareUpdate/Programmer_Finished.JPG", width: 100%), [This is what you should see, if the program was flashed properly.], "e") <fig-SWUpdateProgramFinished>]]]
  ),
  caption: [Flashing new software to the board (cont.).],
  continued: true,
) <fig-SWUpdate>

#pagebreak()

== Beeping Patterns

#metadata(none) <sec-BeepingPatterns> The #gls("CATS", cap: false) Vega flight computer has many beeping patterns that inform the user about the current state of the system or about potential errors. The exhaustive list below should be enough to know if an error is present or in what state the flight computer is in.

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
  [Flight computer has booted up.],
  [#image("../images/How To Use/Beeping_Patterns/Beep-Bootup.png", width: 50%)],
  [#gls("Calibrating", cap: false) $arrow.r$ #gls("Ready", cap: false)],
  [Flight computer switched from #gls("Calibrating", cap: false) state to #gls("Ready", cap: false) state.],
  [#image("../images/How To Use/Beeping_Patterns/Beep-Moving-Ready.png", width: 50%)],
  [#gls("Ready", cap: false)],
  [Calibration was successful and flight computer is in #gls("Ready", cap: false) state.],
  [#image("../images/How To Use/Beeping_Patterns/Beep-Ready.png", width: 50%)],
  [Testing],
  [Flight computer is in testing mode. Only beeped after the computer is rebooted.],
  [#image("../images/How To Use/Beeping_Patterns/Beep-Testing.png", width: 50%)],
  [Testing Armed],
  [Flight computer is in armed testing state.],
  [#image("../images/How To Use/Beeping_Patterns/Beep-Testing-Armed.png", width: 50%)],
  table.hline(y: 2, stroke: 0.5pt + black)
),
  caption: [Overview of state beeping patterns.],
  continued: false,
  breakable: false,
) <tab-BeepingPatternsStates>

#note[
*Note:* The pitch axis is only showing relative changes and is unitless.
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
  [A #gls("barometer", cap: false) or accelerometer error is present. If this error persists, *do not fly as the functionality of the flight computer is compromised!!!*],
  [TBD],
  [Pyro Error],
  [A pyro that is configured to trigger is not detected. If this error persists, *do not fly as the recovery mechanism you configured will not work!!!*],
  [TBD],
  [Log Full],
  [The flash chip is full. The flight will not be recorded if you decide to still fly.],
  [TBD],
  [Telemetry Hot],
  [The telemetry chip reached a temperature of 60 $degree$ Celsius. Damage to the telemetry could happen.],
  [TBD],
  [Calibration Error],
  [The calibration is faulty. *Do not fly!* Go back to the rocket, and reboot the flight computer to restart the calibration.],
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
*Note:* Errors are currently not beeped out. They are only shown over telemetry on the ground station. Beeping of errors will be added in a future patch.
]

#pagebreak()

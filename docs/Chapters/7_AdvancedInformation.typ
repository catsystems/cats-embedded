#import "../styles.typ": *

= Advanced Information

#metadata(none) <sec-AdvancedInfo> This section explains the core components of the CATS Vega board in greater detail. It is intended for advanced users.

== Software Overview

This section provides a brief overview of the software architecture. It is intended for advanced users with some programming experience; understanding it is not required to use the flight computer.#linebreak() The software is implemented in C++ and uses #gls("FreeRTOS", cap: false) as its foundation. The hardware is initialized first, after which the tasks are started. Figure #xref("fig:SoftwareOverview") shows the running tasks.

#cats-figure(image("../images/Working Principle/Software_Overview_Vega.png", width: 100%), caption: [Illustration of the different FreeRTOS tasks interacting with each other and the hardware. The black circle 'Settings' is just a memory region that is being accessed by different tasks.]) <fig-SoftwareOverview>

The following list briefly describes each task.

#list(tight: false,
  [
*Sensor Read* reads the #gls("barometer", cap: false) and #gls("IMU", cap: false) data and provides it to the Preprocessing task.
],
  [
*Preprocessing* converts the raw barometric pressure to altitude above ground level and the linear acceleration to acceleration in the "up" direction. It provides the processed data to the FSM and State Estimation tasks.
],
  [
*State Estimation* estimates the current height and velocity based on the Preprocessing task's output and the FSM state.
],
  [
*FSM* computes the current flight phase from the State Estimation and Preprocessing outputs.
],
  [
*Transceiver* handles communication between the main chip and the telemetry chip.
],
  [
*USB Communicator* is a group of three tasks that handle the interface between the flight computer and the user's computer. These tasks do not start when USB is not connected.
],
  [
*Peripherals* triggers all user-defined events.
],
  [
*Health Monitor* checks values like the buzzer and the battery voltage.
],
  [
*Recorder* uses a queue to record all data to the flash chip.
],
  [
*Buzzer* actuates the buzzer.
]
)

== Telemetry

The telemetry system uses 2.4 GHz LoRa and #gls("fhss", cap: false) (Frequency-Hopping Spread Spectrum). #gls("fhss", cap: false) makes transmissions more resistant to interference and more difficult to intercept. It also allows more devices to use the same frequency band with little or no effect on link quality. #linebreak()#linebreak()#v(1.8pt)

*Hopping Pattern*#linebreak()#v(-1.8pt) The link phrase defines the hopping pattern. It is hashed with a #gls("crc", cap: false)-32 algorithm, and the resulting value seeds a pseudo-random number generator. The generator runs 20 times to define the hopping pattern. As a result, a given link phrase always produces the same pattern. The transmitter and receiver must use the same link phrase to communicate.#linebreak()#linebreak()#v(1.8pt)

#cats-figure(image("../images/Working Principle/fhss.png", width: 100%), caption: [#gls("fhss", cap: false) transmission example]) <fig-fhss>

*Synchronization*#linebreak()#v(-1.8pt) The receiver waits on the first frequency until it receives a synchronization packet. This packet contains the link #gls("crc", cap: false), which identifies the transmission source. If the remote #gls("crc", cap: false) matches the local value, the receiver hops to the next frequency and waits for data. Each data packet contains a checksum for validating its contents. The receiver measures the interval between packets and hops to the next frequency when a packet is not received within the estimated interval. It can perform 30 hops without receiving a packet before synchronization is lost. If the connection is lost, the receiver returns to the first frequency.

#pagebreak()

== Estimation Algorithms

#metadata(none) <sec-EstAlg> State estimation calculates the rocket's velocity and altitude from barometric pressure and linear acceleration in the $z$ direction.#linebreak()

*Calibration of Sensors*#linebreak()#v(-1.8pt) Linear acceleration is calibrated when the system enters the #gls("Ready", cap: false) state. This allows the flight computer to be mounted in any orientation. The gravity vector is used to calculate the up direction, which is then used throughout the flight.

#warning[
*Warning:* Power up the flight computer only after the rocket is upright on the launch pad. To prevent repeated transitions into and out of the #gls("Ready", cap: false) state, calibration is performed only once, as soon as no motion is detected after startup.
]

During #gls("Calibrating", cap: false) and #gls("Ready", cap: false), the current altitude above sea level is continuously estimated. Altitude above ground level, the value used during flight, is calculated from the altitude above sea level. This calculation assumes that barometric pressure changes very slowly. When #gls("liftoff", cap: false) is detected, the altitude above sea level is fixed, and only the altitude above ground level is updated. *Kalman Filter*#linebreak()#v(-1.8pt) A #gls("Kalman Filter", cap: false) estimates altitude and velocity from the calibrated values. Its derivation is described below. We define the state and noise as

$ x(t) = mat(h(t); v(t); a_(o)(t)) quad v = mat(v_1; v_2) $

where $h$ is the altitude above ground level, $v$ is the vertical velocity, and $a_0$ is the estimated offset of the measured vertical acceleration. $v_1$ is the noise applied to the vertical acceleration, and $v_2$ is the noise used to estimate the linear-acceleration offset.#linebreak() The system input is $u(t)$, the linear acceleration measured in the $z$ direction.

$ dot(x)(t) = mat(v(t); a(t); dot(a_0)(t)) = A dot x(t) + B dot u(t) + G dot v(t) = mat(0, 1, 0; 0, 0, 1; 0, 0, 0) dot mat(h(t); v(t); a_(o)(t)) + mat(0; 1; 0) dot u(t) + mat(0, 0; 1, 0; 0, 1) dot mat(v_1(t); v_2(t)) $

This is discretized using a first-order approximation:

$ A_d = e^(A T_s) quad B_d = integral_(0)^(T s) e^(A_d t) delta t dot B quad G_d = integral_(0)^(T s) e^(A_d t) delta t dot G $

which gives the system

$ x(k+1) = A_d dot x(k) + B_d dot u(k) + G_d dot v(k) $

The process measurement noise matrix becomes

$ Q(k) = mat(Q_("acc")(k), 0; 0, Q_("acc_0")(k)) $

The measurement step assumes that altitude has already been calculated from barometric pressure using the standard barometric formula.

$ z(k) = h_("meas")(k) + omega(k) = (frac(p(k), p_0)^(frac(1, 5.257)) - 1) dot frac(T_0+ 273.15, L) - h_0 + omega(k) $

where $L = -0.0065$, $T_0 = 15$ C, $p_0 = 101250$ Pa, $h_0$ is the calibrated altitude above sea level, and $omega$ is the measurement noise.

The measurement function is

$ z(k) = H dot x(k) = mat(1, 0, 0) dot mat(h(k); v(k); a_(o)(k)) $

The measurement-noise matrix becomes a scalar:

$ R(k) = R_("height") $

The standard Kalman-filter equations can then propagate the state. #linebreak()#v(-1.8pt) *Gain Scheduling* #linebreak()#v(-1.8pt) Gain scheduling reduces reliance on the barometer during high-velocity flight, because barometric measurements can behave unpredictably in the transonic regime. #linebreak() At #gls("liftoff", cap: false) and while the rocket is moving quickly, the accelerometer is weighted more heavily when estimating altitude and velocity. At lower velocities, barometric pressure is weighted more heavily because accelerometer drift affects the estimate. As the rocket arcs over, the quality of the accelerometer measurement also decreases.#linebreak() Two variables control the relative trust in the sensors: $Q_("acc")$ and $R_("height")$. In the algorithm, $Q_("acc")$ remains constant, while $R_("height")$ changes during flight.#linebreak() The conditions for changing $R_("height")$ are shown below.

$ R_("height") = cases(R_("initial"), & "for state = MOVING or IDLE", R_("max"), & "for state = LIFTOFF", R_("max") dot f(v), & "for state = COASTING", R_("initial"), & "otherwise") $

where $f(v)$ depends on the current velocity.#linebreak() This gain scheduling effectively filters unexpected barometric measurements at high velocities.

#pagebreak()

== Using the Python Plotting Tool

#metadata(none) <sec-GeneratePlotsPython> For more control over flight data, use the legacy Python plotting tool. It provides the Configurator's plotting functionality in a form that is easier to customize.#linebreak() To visualize logs recorded by the Vega flight computer, clone the #link("https://github.com/catsystems/cats-logs")[cats-logs] repository and run the log\_visualizer.py script. #link("https://git-scm.com/")[Git] and #link("https://www.python.org/")[Python 3] must be installed to download and run the script.

To visualize your logs, follow these steps:

```bash
git clone https://github.com/catsystems/cats-logs.git
cd cats-logs/log_parsing
pip install -r requirements.txt
python log_visualizer.py -i <path to input log> -o <path to output directory>
```

The script parses the log file and generates a self-contained HTML file containing all plots. It also generates raw and processed CSV files for each type of recorded value.

For more information about the visualizer script, run:

```bash
./log_visualizer.py --help
```

#pagebreak()

== Description of the CLI

#metadata(none) <sec-CLI> This section describes all commands available in the CLI. To access the CLI, connect the board to your computer, connect through the Configurator, and open the CLI tab. #linebreak() In the list below, square brackets #text(font: "DejaVu Sans Mono", size: 0.9em)[\[ \]] indicate an optional argument, while angle brackets \< \> identify a parameter name.#linebreak() After changing the configuration through the CLI, verify it with the #text(font: "DejaVu Sans Mono", size: 0.9em)[config] command.

#cats-table(
  table(
  columns: (0.3fr, 0.35fr, 0.35fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[bl]],
  [Put the board into DFU mode],
  [Needed for software updates (refer to Section #xref("sec:softwareupdates"))],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[cd]],
  [Change the current working directory],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[config]],
  [Print the flight config in a human-readable format],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[defaults \[--no-outputs\]]],
  [Reset to default settings],
  [The default configuration triggers pyro channels. If #text(font: "DejaVu Sans Mono", size: 0.9em)[--no-outputs] is passed, pyro triggering is not configured.],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[dump]],
  [Print configurable settings in a readable format],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[flash\_erase]],
  [Erase everything on the flash chip; this might take a while],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[flash\_test]],
  [Test writing to and reading from the flash],
  [For testing purposes only; do not use],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[flash\_start\_write]],
  [Start writing to flash],
  [For testing purposes only; do not use],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[flash\_stop\_write]],
  [Stop writing to flash],
  [For testing purposes only; do not use],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[flight\_dump \< flight\_id\>]],
  [Print a specific flight in binary format],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[flight\_parse \< flight\_id\>]],
  [Print a specific flight in a human-readable format],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[get \[\< variable\>\]]],
  [Get a variable value, described in Table #xref("tab:CLICommandsSetGet")],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[help \[\< command name\>\]]],
  [Display all commands with a description],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[lfs\_format]],
  [Reformat the flash filesystem],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[log\_enable]],
  [Enable log output on the terminal],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ls \[\< path\>\]]],
  [List all files in the current working directory],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[reboot]],
  [Reboot the flight computer],
  [This command does not save the changed settings by itself],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[rec\_info]],
  [Get information about flash usage],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[rm \[\< path\>\]]],
  [Remove a file],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[save]],
  [Save flight configuration],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[set \[\< variable\> =\< value\>\]]],
  [Set a variable, described in Table #xref("tab:CLICommandsSetGet")],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[stats \< flight\_id\>]],
  [Print flight statistics],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[status]],
  [Show current sensor data, flight phase, and other important information],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[version]],
  [Show the firmware version],
  []
),
  caption: [Exhaustive List of #gls("CLI", cap: false) Commands],
  continued: false,
  breakable: true,
) <tab-CLICommands>

#pagebreak()

#heading(level: 3, outlined: false)[Get and Set Commands]

The variables below can be read with the #text(font: "DejaVu Sans Mono", size: 0.9em)[get] command or changed with the #text(font: "DejaVu Sans Mono", size: 0.9em)[set] command. Changes are saved to the flight computer's configuration only after the #text(font: "DejaVu Sans Mono", size: 0.9em)[save] command is run.

#cats-table(
  table(
  columns: (0.3fr, 0.35fr, 0.35fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[acc\_threshold]],
  [Acceleration threshold above which liftoff is detected],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[main\_altitude]],
  [Altitude above ground level under which main deployment is triggered],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[timer1\_start]],
  [Event which triggers timer 1],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[timer1\_trigger]],
  [Event which is triggered once timer 1 elapses],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[timer1\_duration]],
  [Timer 1 duration],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[timer2\_start]],
  [Event which triggers timer 2],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[timer2\_trigger]],
  [Event which is triggered once timer 2 elapses],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[timer2\_duration]],
  [Timer 2 duration],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[timer3\_start]],
  [Event which triggers timer 3],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[timer3\_trigger]],
  [Event which is triggered once timer 3 elapses],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[timer3\_duration]],
  [Timer 3 duration],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[timer4\_start]],
  [Event which triggers timer 4],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[timer4\_trigger]],
  [Event which is triggered once timer 4 elapses],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[timer4\_duration]],
  [Timer 4 duration],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_calibrate]],
  [Set the actions associated with the calibration event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_ready]],
  [Set the actions associated with the ready event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_burnout]],
  [Set the actions associated with the burnout event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_apogee]],
  [Set the actions associated with the apogee event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_main\_deployment]],
  [Set the actions associated with the main deployment event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_touchdown]],
  [Set the actions associated with the touchdown event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_custom1]],
  [Set the actions associated with the custom 1 event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_custom2]],
  [Set the actions associated with the custom 2 event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[servo1\_init\_pos]],
  [Set the initial position of servo 1],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[servo2\_init\_pos]],
  [Set the initial position of servo 2],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[tele\_link\_phrase]],
  [Set the telemetry link phrase],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[tele\_test\_phrase]],
  [Set the testing phrase],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[tele\_power\_level]],
  [Set the telemetry power level],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[tele\_adaptive\_power]],
  [Enable or disable adaptive power for the telemetry power level],
  [Adaptive power mode boosts output power to maximum when the flight computer is in _THRUSTING_ mode and returns it to the user-set value when _TOUCHDOWN_ is registered.],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[buzzer\_volume]],
  [Set the buzzer volume],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[battery\_type]],
  [Set the battery type used with the CATS Vega],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[rec\_elements]],
  [Set the desired recorded elements],
  [A bit mask corresponding to the #link("https://github.com/catsystems/cats-embedded/blob/674192f757e7b1cd11fc023cafc6ea9dcf132f5f/flight_computer/src/flash/recorder.hpp#L35")[#text(font: "DejaVu Sans Mono", size: 0.9em)[rec\_entry\_type\_e]] enum],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[rec\_speed]],
  [Set the desired sampling period for recording],
  []
),
  caption: [Exhaustive List of parameters used in the #text(font: "DejaVu Sans Mono", size: 0.9em)[get] and #text(font: "DejaVu Sans Mono", size: 0.9em)[set] Commands],
  continued: false,
  breakable: true,
) <tab-CLICommandsSetGet>

#pagebreak()

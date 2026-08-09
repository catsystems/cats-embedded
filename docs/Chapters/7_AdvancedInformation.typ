#import "../styles.typ": *

= Advanced Information

#metadata(none) <sec-AdvancedInfo> In this section, the core pieces of the CATS Vega board are explained in more detail. This part of the manual is written for advanced users.

== Software Overview

In this section we briefly touch on how the software is set up. This section is for advanced users with some coding experience and in no way does the user need to understand this section to be able to use the flight computer.#linebreak() The software is implemented in C++. The backbone of our implementation is #gls("FreeRTOS", cap: false). All hardware is first initialized and at the end of the initialization the tasks are started. An overview of the running tasks is shown in Figure #xref("fig:SoftwareOverview").

#cats-figure(image("../images/Working Principle/Software_Overview_Vega.png", width: 100%), caption: [Illustration of the different FreeRTOS tasks interacting with each other and the hardware. The black circle 'Settings' is just a memory region that is being accessed by different tasks.]) <fig-SoftwareOverview>

A brief explanation of every task is given here.

#list(tight: false,
  [
*Sensor Read* reads out the #gls("barometer", cap: false) and the #gls("IMU", cap: false) and provides them to the preprocessing task.
],
  [
*Preprocessing* takes this raw data and transforms barometric pressure to height above ground level and linear acceleration to acceleration in the 'up' direction. It then provides this data to the FSM and state estimation task.
],
  [
*State Estimation* estimates the current height and velocity based on the Preprocessing task's output and the FSM state.
],
  [
*FSM* computes the current flight phase based on the estimate from the State Estimation and Preprocessing tasks.
],
  [
*Transceiver* handles the communication between the main chip and the telemetry chip.
],
  [
*USB Communicator* is a collection of three tasks, handling the interface between the flight computer and the user computer. If no USB is connected, this task is not started.
],
  [
*Peripherals* triggers all user-defined events.
],
  [
*Health Monitor* checks values like the buzzer and the battery voltage.
],
  [
*Recorder* records all the data using a queue to the flash chip.
],
  [
*Buzzer* actuates the buzzer.
]
)

== Telemetry

The telemetry system is based on 2.4 GHz LoRa and utilizes #gls("fhss", cap: false) (Frequency-Hopping Spread Spectrum) for transmission. A key benefit of #gls("fhss", cap: false) is that it makes transmissions much more resistant to interference and more difficult to intercept. Additionally, it allows more devices on the same frequency band with little or no impact on the link quality. #linebreak()#linebreak()#v(1.8pt)

*Hopping Pattern*#linebreak()#v(-1.8pt) The hopping pattern is defined with a link phrase. This link phrase is hashed using a #gls("crc", cap: false)32 algorithm, and the resulting hash value is then used as the seed for a pseudo-random number generator. The generator is run 20 times, defining the hopping pattern. With this method, a link phrase always generates the same hopping pattern. Both the transmitter and receiver require the same link phrase for the transmission to work.#linebreak()#linebreak()#v(1.8pt)

#cats-figure(image("../images/Working Principle/fhss.png", width: 100%), caption: [#gls("fhss", cap: false) transmission example]) <fig-fhss>

*Synchronization*#linebreak()#v(-1.8pt) The receiver waits on the first frequency until a sync packet is received. The sync packet contains the link #gls("crc", cap: false) to identify the transmission source. If the remote #gls("crc", cap: false) matches the local #gls("crc", cap: false), the receiver hops to the subsequent frequency, waiting for data. Each data package contains a checksum to validate the contents. The time between packages is measured and used to jump to the next frequency when no package was received in the estimated time frame. A total of 30 hops can be performed without receiving a package before the synchronization is lost. On connection loss, the receiver returns to the first frequency.

#pagebreak()

== Estimation Algorithms

#metadata(none) <sec-EstAlg> The state estimation is used to estimate the velocity and the height of the rocket. For this, it uses the barometric pressure data and the linear acceleration in the $z$ direction.#linebreak()

*Calibration of Sensors*#linebreak()#v(-1.8pt) As soon as the system enters the #gls("Ready", cap: false) state, the linear acceleration is calibrated. This allows you to mount the flight computer in any direction. What is being done here, is that based on the gravity vector, the up direction is computed which is then used throughout the flight.

#warning[
*Warning:* The flight computer should only be powered up once the rocket is upright on the launch pad. Since we want to prevent the flight computer going in and out of the #gls("Ready", cap: false) state, the calibration is done only once, as soon as no motion is detected after boot up.
]

During #gls("Calibrating", cap: false) or #gls("Ready", cap: false), the current height above sea level is always estimated. The height above ground level, which is the actual important value is always calculated based on the height above sea level. To calculate the height above sea level we assume that the barometric pressure is very slowly varying. At the moment when #gls("liftoff", cap: false) is detected, the height above sea level is locked in place and only the height above ground level is updated. *Kalman Filter*#linebreak()#v(-1.8pt) To estimate the height and velocity based on the calibrated values a #gls("Kalman Filter", cap: false) is implemented. Here the derivation of the filter is described. We assume the state and the noise to be

$ x(t) = mat(h(t); v(t); a_(o)(t)) quad v = mat(v_1; v_2) $

where $h$ is the height above ground level, $v$ is the vertical velocity and $a_0$ is the estimated offset of the measured vertical acceleration. $v_1$ is the noise applied on the vertical acceleration and $v_2$ is the noise variable used to estimate the linear acceleration offset.#linebreak() Additionally, we need to define the input to the system, which, in this derivation is $u(t)$ being the linear acceleration measured in $z$ direction.

$ dot(x)(t) = mat(v(t); a(t); dot(a_0)(t)) = A dot x(t) + B dot u(t) + G dot v(t) = mat(0, 1, 0; 0, 0, 1; 0, 0, 0) dot mat(h(t); v(t); a_(o)(t)) + mat(0; 1; 0) dot u(t) + mat(0, 0; 1, 0; 0, 1) dot mat(v_1(t); v_2(t)) $

this is discretized using a first order approximation

$ A_d = e^(A T_s) quad B_d = integral_(0)^(T s) e^(A_d t) delta t dot B quad G_d = integral_(0)^(T s) e^(A_d t) delta t dot G $

giving the system

$ x(k+1) = A_d dot x(k) + B_d dot u(k) + G_d dot v(k) $

The process measurement noise matrix becomes

$ Q(k) = mat(Q_("acc")(k), 0; 0, Q_("acc_0")(k)) $

The measurement step assumes, that the height is already calculated from the barometric pressure. For this we use the standard barometric pressure formula.

$ z(k) = h_("meas")(k) + omega(k) = (frac(p(k), p_0)^(frac(1, 5.257)) - 1) dot frac(T_0+ 273.15, L) - h_0 + omega(k) $

with $L = -0.0065$, $T_0 = 15$ C, $p_0 = 101250$ Pa, $h_0 =$ calibrated altitude above sea level and $omega$ being the measurement noise.

the measurement function becomes, very easily

$ z(k) = H dot x(k) = mat(1, 0, 0) dot mat(h(k); v(k); a_(o)(k)) $

the measurement noise matrix becomes a scalar.

$ R(k) = R_("height") $

And then, the standard Kalman filter equations can be used to propagate the state. #linebreak()#v(-1.8pt) *Gain-Scheduling* #linebreak()#v(-1.8pt) The idea behind gain scheduling this algorithm is that we want to reduce the trust in the barometer during high velocity phases. We do this because in the transonic regime the barometer behaves unpredictably. With the gain scheduling, this unpredictable behaviour is ignored during high velocity phases. #linebreak() What we want is that at #gls("liftoff", cap: false) and while the speed of the rocket is large, the accelerometer is given much more weight in the estimation of the height and the velocity. Once we get back to smaller velocities, we want to trust the barometric pressure more as accelerometer drift impacts the estimate. Additionally, the rocket arcs over, further decreasing the quality of accelerometer measurement.#linebreak() We have two variables that affect the trust of the two sensors: $Q_("acc")$ and $R_("height")$. In the algorithm, $Q_("acc")$ is kept constant and only $R_("height")$ is changed during flight.#linebreak() The conditions for the changes in $R_("height")$ are given below.

$ R_("height") = cases(R_("initial"), & "for state = MOVING or IDLE", R_("max"), & "for state = LIFTOFF", R_("max") dot f(v), & "for state = COASTING", R_("initial"), & "otherwise") $

Where $f(v)$ is a function dependent on the current velocity.#linebreak() With this gain scheduling, unexpected barometric measurements during high velocities are effectively filtered out.

#pagebreak()

== Using the Python Plotting Tool

#metadata(none) <sec-GeneratePlotsPython> If you would like to have some more control over the flight data and access it easily, you can have a look at our old python plotting tool. With this you can do everything that the Configurator can do, but it should be easier for you to do changes.#linebreak() In order to visualize the logs recorded on the Vega flight computer, you need to clone the #link("https://github.com/catsystems/cats-logs")[cats-logs] repository and run the log\_visualizer.py script. Note that you need to have #link("https://git-scm.com/")[git] and #link("https://www.python.org/")[Python 3] installed on your computer in order to be able to download and run this script.

To visualize your logs, follow these steps:

```bash
git clone https://github.com/catsystems/cats-logs.git
cd cats-logs/log_parsing
pip install -r requirements.txt
python log_visualizer.py -i <path to input log> -o <path to output directory>
```

The script will parse the log file and generate a self-contained HTML file that contains all the plots. Additionally, the script will also generate raw and processed CSV files for every type of recorded value.

You can get more information on the visualizer script by calling:

```bash
./log_visualizer.py --help
```

#pagebreak()

== Description of the CLI

#metadata(none) <sec-CLI> This section describes all commands available on the CLI. To access the CLI, plug in the board, connect to the configurator and navigate to the CLI tab. #linebreak() In the list below, square brackets #text(font: "DejaVu Sans Mono", size: 0.9em)[\[ \]] mean that the argument is optional and angle brackets \< \> specify the parameter name.#linebreak() Whenever changes to the configuration are done using the CLI, we heavily encourage to recheck the configuration using the #text(font: "DejaVu Sans Mono", size: 0.9em)[config] command.

#cats-table(
  table(
  columns: (0.3fr, 0.35fr, 0.35fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[bl]],
  [Put the board into DFU mode],
  [needed for software updates (refer to section #xref("sec:softwareupdates"))],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[cd]],
  [Change the current working directory],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[config]],
  [Print the flight config in a human-readable format],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[defaults \[--no-outputs\]]],
  [Reset to default settings],
  [The default configuration triggers pyro channels. If #text(font: "DejaVu Sans Mono", size: 0.9em)[--no-outputs] is passed the pyro triggering will not be set.],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[dump]],
  [Print configurable settings in a readable format],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[flash\_erase]],
  [Erase everything on the flash chip; this might take a while],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[flash\_test]],
  [Test writing and reading from the flash],
  [Only for testing purposes, should not be used!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[flash\_start\_write]],
  [Start writing to flash],
  [Only for testing purposes, should not be used!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[flash\_stop\_write]],
  [Stop writing to flash],
  [Only for testing purposes, should not be used!],
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
  [Reformat flash file system],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[log\_enable]],
  [Enable log output on the terminal],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ls \[\< path\>\]]],
  [List all files in current working directory],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[reboot]],
  [Reboot the computer],
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
  [Print flight stats],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[status]],
  [Show current sensor data, flight phase and other important information],
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

The variables described here can be read by the get command or written by the set command. Those variables are then saved to the configuration on the flight computer only if save is also commanded.

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
  [Set the actions associated to the calibration event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_ready]],
  [Set the actions associated to the ready event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_burnout]],
  [Set the actions associated to the burnout event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_apogee]],
  [Set the actions associated to the apogee event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_main\_deployment]],
  [Set the actions associated to the main deployment event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_touchdown]],
  [Set the actions associated to the touchdown event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_custom1]],
  [Set the actions associated to the custom 1 event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[ev\_custom2]],
  [Set the actions associated to the custom 2 event],
  [Do not use!],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[servo1\_init\_pos]],
  [Set the initial value of the servo 1],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[servo2\_init\_pos]],
  [Set the initial value of the servo 2],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[tele\_link\_phrase]],
  [Set the telemetry link phrase],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[tele\_test\_phrase]],
  [Set the testing pass phrase],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[tele\_power\_level]],
  [Set the telemetry power level],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[tele\_adaptive\_power]],
  [Enable or disable adaptive power for the telemetry power level],
  [Adaptive power mode boosts output power to maximum when FC is in _THRUSTING_ mode, and returns it back to the user-set value when _TOUCHDOWN_ is registered.],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[buzzer\_volume]],
  [Set the buzzer volume],
  [],
  [#text(font: "DejaVu Sans Mono", size: 0.9em)[battery\_type]],
  [Set the battery type used with the #gls("CATS", cap: false) Vega],
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

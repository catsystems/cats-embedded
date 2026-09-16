#import "../styles.typ": *

= Example Configurations

#metadata(none) <sec-Examples> These examples show how hardware, flight events, actions, and timers fit together. Adapt all thresholds, deployment altitudes, delays, and output settings to the actual rocket and recovery system.

#warning[
*Warning:* The values below are examples, not flight-ready recommendations. Verify the complete configuration in *Preflight* and test every recovery mechanism safely before flight.
]

== Simple Dual-Deployment Example

This example uses a pyrotechnic charge on Pyro Channel 1 for the drogue parachute and a second charge on Pyro Channel 2 for the main parachute.

#cats-table(
  table(
  columns: (0.24fr, 0.28fr, 0.48fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [Area], [Example value], [Purpose],
  [Configuration], [Liftoff threshold: 40 $m/s^2$], [Detect liftoff after the configured acceleration is sustained.],
  [Events & Timers], [Apogee → Pyro 1 ON], [Deploy the drogue parachute at detected apogee.],
  [Events & Timers], [Main Deployment → Pyro 2 ON], [Deploy the main parachute below the configured main altitude.],
  [Events & Timers], [Timer: Liftoff → Apogee after 15 s], [Provide a time-based backup for the apogee event.],
  table.hline(y: 1, stroke: 0.5pt + black)
),
  caption: [Simple dual-deployment example],
  continued: false,
  breakable: true,
) <tab-ExampleSimple>

Connect the battery and switch, then connect the deployment circuits to Pyro Channels 1 and 2 as shown below. Keep the charges disconnected while checking continuity and event behavior.

#cats-figure(doc-image("Examples/HWSimpleExample.png", width: 100%), caption: [CATS Vega with a battery, switch, Pyro 1 and Pyro 2 connected.]) <fig-HWsimpleExample>

In *Preflight*, verify the main-deployment altitude, liftoff threshold, timer, event sequence, and both pyro actions. Use the procedure in Chapter #xref("sec:Testing") to test the configuration with safe substitutes before connecting energetic devices.

== Advanced Recovery Example

This example adds timed actions, a servo-deployed main parachute, a camera, and a low-level output. It illustrates the available building blocks; it is not a recommended configuration for a specific vehicle.

#cats-table(
  table(
  columns: (0.25fr, 0.32fr, 0.43fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [Area], [Example value], [Purpose],
  [Configuration], [Liftoff threshold: 40 $m/s^2$], [Detect liftoff.],
  [Configuration], [Main altitude: 350 m], [Trigger the Main Deployment event below 350 m during descent.],
  [Configuration], [Servo 1 initial position: mechanism closed], [Hold the main-deployment mechanism in its safe initial position. Determine the actual value on the bench.],
  [Events & Timers], [Apogee → Pyro 1 ON; delay 2 s; Pyro 1 OFF], [Operate a solenoid valve for two seconds.],
  [Events & Timers], [Main Deployment → Servo 1 deployed position], [Open the main-deployment mechanism. Determine the deployed value on the bench.],
  [Events & Timers], [Liftoff → Pyro 2 ON; Touchdown → Pyro 2 OFF], [Control an externally powered camera through a suitable interface.],
  [Events & Timers], [Burnout → I/O ON], [Assert the low-level output at burnout.],
  [Events & Timers], [Timer: Liftoff → Apogee after 45 s], [Provide a time-based backup for the apogee event.],
  table.hline(y: 1, stroke: 0.5pt + black)
),
  caption: [Advanced recovery example],
  continued: false,
  breakable: true,
) <tab-ExampleAdvanced>

Connect the battery and switch, the solenoid-valve interface to Pyro Channel 1, the camera interface to Pyro Channel 2, and the main-deployment mechanism to Servo Channel 1 as shown below. Observe the electrical limits in Section #xref("sec:VegaHW") and use external drivers where the load requires them.

#cats-figure(doc-image("Examples/HWAdvancedExample.png", width: 100%), caption: [CATS Vega with a battery, switch, solenoid valve, camera, and servo connected.]) <fig-HWadvancedExample>

Before flight, use *Preflight* to inspect the complete timeline, including delayed actions and timer-triggered events. Test each event individually in testing mode, then disable testing mode, reboot the Vega, and run Preflight again.

#pagebreak()

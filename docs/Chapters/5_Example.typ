#import "../styles.typ": *

= Example Configuration of the CATS Vega + Ground Station

#metadata(none) <sec-Examples> This chapter presents two example configurations, showing both the relevant Configurator settings and the corresponding hardware setup. #linebreak() The first example is a simple configuration with a drogue chute and a main chute. The second example is intended for more advanced rockets and includes custom events.

== Simple Example Configuration

For the simple example, assume the following configuration:

#enum(tight: false,
  [
The drogue chute is deployed by a pyrotechnic charge connected to #gls("pyro", cap: false) Channel 1, which must be turned on (Fig. #xref("fig:ExampleSimpleEvents")).
],
  [
The expected time to #gls("apogee", cap: false) is 15 seconds and is used for a backup timer (Fig. #xref("fig:ExampleSimpleTimers")).
],
  [
The main chute is deployed by a pyrotechnic charge connected to #gls("pyro", cap: false) Channel 2, which must be turned on (Fig. #xref("fig:ExampleSimpleEvents")). No timer is configured for main-chute deployment.
],
  [
The liftoff acceleration is simulated to be 70 $m/s^2$ (Fig. #xref("fig:ExampleSimpleConfig")).
]
)

Configure these settings on the flight computer, then connect the battery, switch, and pyrotechnic charges to Pyro Channels 1 and 2. The completed CATS Vega setup should match Fig. #xref("fig:HWsimpleExample").

#cats-figure(image("../images/Examples/HWSimpleExample.png", width: 100%), caption: [CATS Vega with a battery, switch, Pyro 1 and Pyro 2 connected.]) <fig-HWsimpleExample>

#cats-figure(
  stack(dir: ttb, spacing: 8pt,
    [#align(center)[#block(width: 75%)[#subfigure(image("../images/Examples/Config_simple_Config.png", width: 100%), [Configuration Tab (simple example).], "a") <fig-ExampleSimpleConfig>]]],
    [#align(center)[#block(width: 75%)[#subfigure(image("../images/Examples/Config_simple_Events.jpg", width: 100%), [Event Tab (simple example).], "b") <fig-ExampleSimpleEvents>]]],
    [#align(center)[#block(width: 75%)[#subfigure(image("../images/Examples/Config_simple_Timers.jpg", width: 100%), [Timer Tab (simple example).], "c") <fig-ExampleSimpleTimers>]]]
  ),
  caption: [Configurator screenshots for the simple example configuration.],
  continued: false,
) <fig-ExampleSimple>

#pagebreak()

== Advanced Example Configuration

For the advanced example, assume the following configuration:

#enum(tight: false,
  [
The drogue chute is deployed by a solenoid valve connected to #gls("pyro", cap: false) Channel 1. The channel must be turned on for two seconds and then turned off (Fig. #xref("fig:ExampleAdvancedEvents")).
],
  [
The expected time to #gls("apogee", cap: false) is 45 seconds and is used for a backup timer (Fig. #xref("fig:ExampleAdvancedTimers")).
],
  [
The main parachute is deployed by Servo Channel 1. Its initial position is 15 degrees, and the deployment position is 90 degrees (Fig. #xref("fig:ExampleAdvancedConfig"), Fig. #xref("fig:ExampleAdvancedEvents")).
],
  [
No main parachute backup timer is used (Fig. #xref("fig:ExampleAdvancedTimers")).
],
  [
The main parachute should be opened at 350 $m$ (Fig. #xref("fig:ExampleAdvancedConfig")).
],
  [
The liftoff acceleration is simulated to be 60 $m/s^2$ (Fig. #xref("fig:ExampleAdvancedConfig")).
],
  [
A camera is connected to #gls("pyro", cap: false) Channel 2. It turns on at liftoff and off at touchdown (Fig. #xref("fig:ExampleAdvancedEvents")).
],
  [
The low-level #gls("I/O", cap: false) is turned on at burnout (Fig. #xref("fig:ExampleAdvancedEvents")).
]
)

Configure these settings on the flight computer, then connect the battery, switch, solenoid valve to #gls("pyro", cap: false) Channel 1, camera to #gls("pyro", cap: false) Channel 2, and #gls("servo", cap: false) to #gls("servo", cap: true) Channel 1. The completed CATS Vega setup should match Fig. #xref("fig:HWadvancedExample").

#cats-figure(image("../images/Examples/HWAdvancedExample.png", width: 100%), caption: [CATS Vega with a battery, switch, solenoid valve, camera and servo connected.]) <fig-HWadvancedExample>

#cats-figure(
  stack(dir: ttb, spacing: 8pt,
    [#align(center)[#block(width: 75%)[#subfigure(image("../images/Examples/Config_advanced_config.png", width: 100%), [Configuration Tab (advanced example).], "a") <fig-ExampleAdvancedConfig>]]],
    [#align(center)[#block(width: 75%)[#subfigure(image("../images/Examples/Config_advanced_events.jpg", width: 100%), [Event Tab (advanced example).], "b") <fig-ExampleAdvancedEvents>]]],
    [#align(center)[#block(width: 75%)[#subfigure(image("../images/Examples/Config_advanced_timers.jpg", width: 100%), [Timer Tab (advanced example).], "c") <fig-ExampleAdvancedTimers>]]]
  ),
  caption: [Configurator screenshots for the advanced example configuration.],
  continued: false,
) <fig-ExampleAdvanced>

#pagebreak()

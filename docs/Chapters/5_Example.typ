#import "../styles.typ": *

= Example Configuration of the CATS Vega + Ground Station

#metadata(none) <sec-Examples> In this chapter, two example configurations are shown with the relevant configuration in the GUI and the actual setup on the hardware. #linebreak() The first example configuration is simple with only a drogue and main chute. The second example is for more advanced rockets with also some customized events.

== Simple Example Configuration

For the simple configuration let us assume the following desired configuration:

#enum(tight: false,
  [
The drogue is deployed by a pyro, connected to #gls("pyro", cap: false) Channel 1 that needs to be turned on (Fig. #xref("fig:ExampleSimpleEvents")).
],
  [
Time to #gls("apogee", cap: false) is 15 seconds and should be used as a backup timer (Fig. #xref("fig:ExampleSimpleTimers")).
],
  [
The main is deployed by a pyro, connected to #gls("pyro", cap: false) Channel 2 that needs to be turned on (Fig. #xref("fig:ExampleSimpleEvents")). No timer is configured for the main.
],
  [
The liftoff acceleration is simulated to be 70 $m/s^2$ (Fig. #xref("fig:ExampleSimpleConfig")).
]
)

This needs to be configured to the flight computer. Then the accessories need to be added to the CATS Vega. This includes the battery, switch and Pyro Channel 1 and 2. Once those are added to the system, the #gls("CATS", cap: false) should look as shown in Fig. #xref("fig:HWsimpleExample").

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

For the advanced configuration, let us assume the following:

#enum(tight: false,
  [
The drogue is deployed by a solenoid valve, connected to #gls("pyro", cap: false) Channel 1 that needs to be on for two seconds and then turned off again (Fig. #xref("fig:ExampleAdvancedEvents")).
],
  [
Time to #gls("apogee", cap: false) is 45 seconds and should be used as a backup timer (Fig. #xref("fig:ExampleAdvancedTimers")).
],
  [
The main parachute is deployed by Servo Channel 1 that needs to be at 15 degrees initially, and deploys when it is at 90 degrees (Fig. #xref("fig:ExampleAdvancedConfig"), Fig. #xref("fig:ExampleAdvancedEvents")).
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
A camera is connected to #gls("pyro", cap: false) Channel 2 which should turn on at liftoff and turn off at touchdown (Fig. #xref("fig:ExampleAdvancedEvents")).
],
  [
The low level #gls("I/O", cap: false) should be turned on at burnout (Fig. #xref("fig:ExampleAdvancedEvents")).
]
)

This needs to be configured to the flight computer. Then the accessories need to be added to the #gls("CATS", cap: false) Vega. This includes the battery, switch, solenoid valve on #gls("pyro", cap: false) Channel 1, camera on #gls("pyro", cap: false) Channel 2 and the #gls("servo", cap: false) on #gls("servo", cap: true) Channel 1. Once those are added to the system, the #gls("CATS", cap: false) should look as shown in Fig. #xref("fig:HWadvancedExample").

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

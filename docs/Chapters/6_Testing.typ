#import "../styles.typ": *

#set par(spacing: 0.65em)

= Testing

#metadata(none) <sec-Testing> Testing mode lets you trigger the Vega's configured events from a Ground Station while the system is secured on the bench.

== What Testing Mode Does

Testing mode stops normal flight-state processing. After the Vega and Ground Station enter armed testing mode, the Ground Station can trigger any configured event. Every action assigned to that event is executed, including pyrotechnic, servo, low-level I/O, recorder, and delayed actions.

#warning[
*Warning:* Treat every configured output as live. Remove motors and energetic charges until you intentionally test that output, secure the hardware, and keep people clear of all deployment mechanisms. Never fly in testing mode.
]

#note[
*Note:* Unlike during flight, an event can be triggered repeatedly without rebooting. The Vega leaves armed testing mode if the telemetry link is lost.
]

== Before You Start

#list(tight: false,
  [Secure the Vega, battery, Ground Station, and every connected mechanism on a suitable test bench.],
  [Disconnect pyrotechnic charges, motors, and other energetic devices unless testing them is the specific purpose of the procedure.],
  [In the Configurator, review *Events & Timers* and confirm every action that will run for each event.],
  [Open *Preflight* and resolve every reported configuration or connection problem.],
  [Confirm that the Vega and Ground Station use the same receiver mode, link phrase, and testing phrase.]
)

== Enabling and Using Testing Mode

#enum(tight: false,
  [Connect the Vega to the computer and open the Configurator. The Configurator connects automatically when exactly one compatible Vega is available; otherwise, select it manually.],
  [Open *Configuration*, enable *Testing Mode*, and set a testing phrase. Save the configuration.],
  [Reboot the Vega. Testing mode does not become active until after the reboot.],
  [Confirm that the Vega emits the *Testing* beeping pattern described in Section #xref("sec:BeepingPatterns").],
  [Turn on the Ground Station. In *Settings* → *Telemetry*, select the matching receiver mode and set *Link Phrase 1* and *Test Phrase* to the values configured on the Vega.],
  [Open *Testing* on the Ground Station. Read the safety notice, press the A button to continue, and wait for the Ground Station to arm the Vega.],
  [Confirm that the Vega emits the *Armed Testing* beeping pattern.],
  [Select an event, review the confirmation screen, and confirm only when the test area is clear. Observe every configured output and delayed action.],
  [Repeat only the checks required for the test. If the telemetry link is lost, return to the Testing menu and arm the system again.]
)

#cats-figure(
  figure-stack(
    spacing: 14pt,
    responsive-split(
      [#subfigure(doc-image("Testing/GS_Testing_Safety.png", width: 100%, outline: true, alt: "Ground Station testing-mode safety notice"), [Read the complete safety notice.], "a")],
      [#subfigure(doc-image("Testing/GS_Testing_ReadyToStart.png", width: 100%, outline: true, alt: "Ground Station testing-mode arm confirmation"), [Continue only when the connected Vega is in testing mode.], "b")],
      columns: (45%, 1fr, 45%),
    ),
    responsive-split(
      [#subfigure(doc-image("Testing/GS_Testing_Starting.png", width: 100%, outline: true, alt: "Ground Station waiting for testing mode to start"), [Wait while the Ground Station arms the Vega.], "c")],
      [#subfigure(doc-image("Testing/GS_Testing_Events.png", width: 100%, outline: true, alt: "Ground Station testing event selection screen"), [Select the configured event to test.], "d")],
      columns: (45%, 1fr, 45%),
    ),
  ),
  caption: [Entering testing mode and selecting an event],
)

#cats-figure(
  doc-image("Testing/GS_Testing_Confirm.png", width: 60%, outline: true, alt: "Ground Station confirmation before triggering a test event"),
  caption: [Final confirmation before triggering the selected event],
)

== Returning to Flight Configuration

#enum(tight: false,
  [Exit *Testing* on the Ground Station.],
  [Reconnect the Vega to the Configurator, disable *Testing Mode*, and save the configuration.],
  [Reboot the Vega and confirm that the Testing beep pattern is no longer emitted.],
  [Run *Preflight* again before installing energetic devices or preparing the rocket for flight.]
)

#warning[
*Warning:* Testing mode manually executes the actions assigned to the selected event. CATS GmbH is not responsible for injuries or material damage caused by unsafe manual operation of the CATS System.
]

#pagebreak()

#import "../styles.typ": *

#set par(spacing: 0.65em)

= Testing

#metadata(none) <sec-Testing> This section explains how to run tests with the CATS System.

== Working Principle

The testing mode of the Vega flight computer works only with a ground station. Once the flight computer is in this mode all processing of information is disabled. This means that if the flight computer is flown while in testing mode, make sure you bring a shovel with you since you will need it to recover your rocket.#linebreak() Once the flight computer is in testing mode, a beeping sound is emitted, signaling that the computer is indeed in testing. A command from the ground station needs then to be set to arm the flight computer. When the flight computer is armed, events can be executed from the ground station.#linebreak()

#note[
*Note:* Unlike during flight, events can be triggered several times without needing to reboot the system.
]

By design, the testing mode only enables triggering events and not actions so that the user can verify that they added the correct configuration to the flight computer.

== Enabling the Testing Mode

To enable the testing mode, follow these steps:

#enum(tight: false,
  [
Connect your CATS Vega to your computer, start the Configurator, and connect to the flight computer.
],
  [
On the Configurator's Home screen, enable Testing Mode.
],
  [
Set a testing phrase.
],
  [
Reboot the flight computer. If the flight computer is not restarted, the testing mode is not activated.
],
  [
The flight computer should emit the "Testing" beeping pattern. See Section #xref("sec:BeepingPatterns") for more information.
],
  [
Turn on your Ground Station.
],
  [
Make sure that the link phrase and testing phrase match those configured on the flight computer.
],
  [
Open the Testing menu on the Ground Station.

#cats-figure(image("../images/Testing/GroundStation_MainMenu.jpg", width: 70%), caption: [Ground Station Main Menu.]) <fig-GSTestingMainMenu>
],
  [
Read the disclaimer carefully and arm testing mode. *Attention:* After this step, executing events will trigger the connected mechanisms. Follow all safety guidelines.

#cats-figure(image("../images/Testing/GroundStation_Arm_Testing_Mode.jpg", width: 70%), caption: [Arming the test mode.]) <fig-GSTestingArming>
],
  [
A pop-up indicates that testing mode is being activated. Wait until it disappears.

#cats-figure(image("../images/Testing/GroundStation_Wait_Testing_Mode.jpg", width: 70%), caption: [Waiting for testing mode to activate.]) <fig-GSTestingWaitingArming>
],
  [
The flight computer should emit the "Armed Testing" beeping pattern. See Section #xref("sec:BeepingPatterns") for more information.
],
  [
Select the event that you want to trigger.

#cats-figure(image("../images/Testing/GroundStation_Event_Menu.jpg", width: 70%), caption: [Selection of the Event to be triggered.]) <fig-GSTestingEventMenu>
],
  [
Select the event and confirm.

#cats-figure(image("../images/Testing/GroundStation_Trigger_Event.jpg", width: 70%), caption: [Confirming to trigger the desired Event.]) <fig-GSTestingEventTriggering>
]
)

#warning[
*Warning:* Testing mode enables manual triggering of events and corresponding actions associated with them. This feature should only be used for testing purposes and never during flight or other non-controlled activities. CATS GmbH is not responsible for any potential injuries or material damage caused by manual operation of the CATS System.
]

#pagebreak()

#import "../styles.typ": *



#heading(level: 1, numbering: none, outlined: true)[Glossary]

#glossary-table((

  ("apogee", [Apogee #metadata(none) <gls-apogee>], [The highest point of the rocket during a parabolic flight]),

  ("barometer", [Barometer #metadata(none) <gls-barometer>], [A sensor which can measure the barometric pressure of its surroundings]),

  ("Calibrating", [Calibrating #metadata(none) <gls-Calibrating>], [A flight state; when the flight computer is this state, it is in safe mode; no actions are taken by the flight computer]),

  ("CATS", [CATS #metadata(none) <gls-CATS>], [Control and Telemetry Systems]),

  ("CLI", [CLI #metadata(none) <gls-CLI>], [Command Line Interface, used to set and get values from the flight computer when not using the configurator]),

  ("Coasting", [Coasting #metadata(none) <gls-Coasting>], [A flight state; when in this state, burnout of the motor was detected and apogee is approaching]),

  ("crc", [CRC #metadata(none) <gls-crc>], [Cyclic Redundancy Check, used to verify that received data is not corrupted]),

  ("DFU", [DFU #metadata(none) <gls-DFU>], [Device Firmware Update, a mode used to flash new firmware to an embedded system]),

  ("drogue chute", [Drogue Chute #metadata(none) <gls-drogue-chute>], [Parachute which is deployed at apogee. Usually a rather small parachute only decelerating the rocket for a controlled descent]),

  ("fhss", [FHSS #metadata(none) <gls-fhss>], [Frequency Hopping Spread Spectrum, a protocol for sending data over telemetry by using more than one carrier frequency]),

  ("FreeRTOS", [FreeRTOS #metadata(none) <gls-FreeRTOS>], [An operating system for embedded CPUs, the backbone of the CATS Software]),

  ("FSM", [FSM #metadata(none) <gls-FSM>], [Finite State Machine, a term used to describe the different flight phases and how the changes in the flight phase happens / which changes are allowed to happen]),

  ("GNSS", [GNSS #metadata(none) <gls-GNSS>], [Global Navigation Satellite System, for example GPS / Galileo]),

  ("I/O", [I/O #metadata(none) <gls-I-O>], [Input/Output, used to group all inputs and outputs of the system in one word]),

  ("IMU", [IMU #metadata(none) <gls-IMU>], [Inertial Measurement Unit, can measure the linear acceleration (i.e. how fast you accelerate straight) and the angular velocity (i.e., how fast you turn)]),

  ("Kalman Filter", [Kalman Filter #metadata(none) <gls-Kalman-Filter>], [A filtering technique used to estimate states according to physical laws and measurements]),

  ("liftoff", [Liftoff #metadata(none) <gls-liftoff>], [An event during flight, when this event happens, the motor of the rocket starts to accelerate and the rocket takes off]),

  ("main chute", [Main Chute #metadata(none) <gls-main-chute>], [Parachute which is deployed at a certain height above ground level. Usually a bigger parachute, slowing the rocket down enough to prevent damage once it hits the ground]),

  ("patch antenna", [Patch Antenna #metadata(none) <gls-patch-antenna>], [A flat antenna with high directivity. The CATS Vega board has a patch antenna, which is used to get the GNSS signal]),

  ("Power Supply", [Power Supply #metadata(none) <gls-Power-Supply>], [Needed to power the system, in rockets usually some battery]),

  ("PWM", [PWM #metadata(none) <gls-PWM>], [Pulse Width Modulation, protocol used to actuate most modern servos]),

  ("pyro", [Pyro #metadata(none) <gls-pyro>], [A charge with two wires which can be ignited with a high current applied]),

  ("quaternion", [Quaternion #metadata(none) <gls-quaternion>], [A non-singular representation of the orientation in 3D space. Similar to Euler angles]),

  ("Ready", [Ready #metadata(none) <gls-Ready>], [A flight state; when in this state, liftoff can be detected]),

  ("RF", [RF #metadata(none) <gls-RF>], [Radio Frequency]),

))

#pagebreak()

#glossary-table((

  ("servo", [Servo #metadata(none) <gls-servo>], [A small electrical motor where the position of the motor can be set by applying a PWM signal]),

  ("Thrusting", [Thrusting #metadata(none) <gls-Thrusting>], [A flight state; when in this state, liftoff was detected and the motor is accelerating the rocket]),

  ("touchdown", [Touchdown #metadata(none) <gls-touchdown>], [Final phase of the flight, when the rocket lands on the ground]),

  ("UART", [UART #metadata(none) <gls-UART>], [A communication protocol to send data from one location to another over three wires]),

), row-gap: 18pt)

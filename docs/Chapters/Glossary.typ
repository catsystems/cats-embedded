#import "../styles.typ": *



#heading(level: 1, numbering: none, outlined: true)[Glossary]

#glossary-table((

  ("apogee", [Apogee #metadata(none) <gls-apogee>], [The highest point of the rocket during a parabolic flight]),

  ("barometer", [Barometer #metadata(none) <gls-barometer>], [A sensor that measures ambient barometric pressure]),

  ("Calibrating", [Calibrating #metadata(none) <gls-Calibrating>], [A flight state in which the flight computer is in safe mode and performs no actions]),

  ("CATS", [CATS #metadata(none) <gls-CATS>], [Control and Telemetry Systems]),

  ("CLI", [CLI #metadata(none) <gls-CLI>], [Command Line Interface, used to read and change flight-computer values without the Configurator]),

  ("Coasting", [Coasting #metadata(none) <gls-Coasting>], [A flight state entered after motor burnout is detected and while the rocket approaches apogee]),

  ("crc", [CRC #metadata(none) <gls-crc>], [Cyclic Redundancy Check, used to verify that received data is not corrupted]),

  ("DFU", [DFU #metadata(none) <gls-DFU>], [Device Firmware Update, a mode used to flash new firmware to an embedded system]),

  ("drogue chute", [Drogue Chute #metadata(none) <gls-drogue-chute>], [A relatively small parachute deployed at apogee to stabilize and slow the rocket during descent]),

  ("fhss", [FHSS #metadata(none) <gls-fhss>], [Frequency Hopping Spread Spectrum, a protocol that transmits telemetry data across multiple carrier frequencies]),

  ("FreeRTOS", [FreeRTOS #metadata(none) <gls-FreeRTOS>], [An operating system for embedded CPUs and the backbone of the CATS software]),

  ("FSM", [FSM #metadata(none) <gls-FSM>], [Finite State Machine, a model that describes the flight phases, how transitions between them occur, and which transitions are permitted]),

  ("GNSS", [GNSS #metadata(none) <gls-GNSS>], [Global Navigation Satellite System, such as GPS or Galileo]),

  ("I/O", [I/O #metadata(none) <gls-I-O>], [Input/Output, a collective term for the system's inputs and outputs]),

  ("IMU", [IMU #metadata(none) <gls-IMU>], [Inertial Measurement Unit, a sensor that measures linear acceleration and angular velocity]),

  ("Kalman Filter", [Kalman Filter #metadata(none) <gls-Kalman-Filter>], [A filtering technique used to estimate states according to physical laws and measurements]),

  ("liftoff", [Liftoff #metadata(none) <gls-liftoff>], [The event that marks the start of powered ascent]),

  ("main chute", [Main Chute #metadata(none) <gls-main-chute>], [The larger parachute deployed at a specified height above ground level to slow the rocket before landing]),

  ("patch antenna", [Patch Antenna #metadata(none) <gls-patch-antenna>], [A flat, highly directional antenna used by the CATS Vega to receive GNSS signals]),

  ("Power Supply", [Power Supply #metadata(none) <gls-Power-Supply>], [A source of electrical power, typically a battery in a rocket]),

  ("PWM", [PWM #metadata(none) <gls-PWM>], [Pulse Width Modulation, a method used to control most modern servos]),

  ("pyro", [Pyro #metadata(none) <gls-pyro>], [A pyrotechnic charge initiated by applying current through two electrical leads]),

  ("quaternion", [Quaternion #metadata(none) <gls-quaternion>], [A non-singular representation of orientation in three-dimensional space, serving a purpose similar to Euler angles]),

  ("Ready", [Ready #metadata(none) <gls-Ready>], [A flight state; when in this state, liftoff can be detected]),

  ("RF", [RF #metadata(none) <gls-RF>], [Radio Frequency]),

))

#pagebreak()

#glossary-table((

  ("servo", [Servo #metadata(none) <gls-servo>], [A small electric actuator whose position is controlled by a PWM signal]),

  ("Thrusting", [Thrusting #metadata(none) <gls-Thrusting>], [A flight state entered after liftoff is detected while the motor is accelerating the rocket]),

  ("touchdown", [Touchdown #metadata(none) <gls-touchdown>], [The final flight phase, entered when the rocket lands]),

  ("UART", [UART #metadata(none) <gls-UART>], [A serial communication interface used to exchange data between devices]),

), row-gap: 18pt)

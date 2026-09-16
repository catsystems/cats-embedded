#import "../styles.typ": *

= FAQ and Troubleshooting

#metadata(none) <sec-FAQ>

*The Configurator does not find my Vega*#linebreak()#v(-1.8pt) Use a USB data cable and connect the Vega directly to the computer. If more than one compatible device is connected, select the intended device manually. Use the refresh button in the device selector after reconnecting the cable. On Windows, also check whether another application has opened the same serial port.#linebreak()#linebreak()#v(1.8pt)

*My configuration changes disappear after reconnecting*#linebreak()#v(-1.8pt) Select *Save* before disconnecting or rebooting the Vega. After saving, use *Refresh* or reconnect and confirm that the stored values match the intended configuration. Run *Preflight* before flight.#linebreak()#linebreak()#v(1.8pt)

*The Ground Station does not receive telemetry*#linebreak()#v(-1.8pt) Confirm that telemetry is enabled on the Vega, both devices have antennas attached, and the Ground Station receiver mode and link phrase match the Vega configuration. In Single mode, Link Phrase 1 is used by both receivers; in Dual mode, each receiver has its own phrase. Check the Ground Station's *Firmware Versions* page if only one receiver behaves unexpectedly.#linebreak()#linebreak()#v(1.8pt)

*Testing mode does not arm*#linebreak()#v(-1.8pt) Enable testing mode and set a valid testing phrase in the Configurator, save the configuration, and reboot the Vega. Set the same Test Phrase and receiver mode on the Ground Station. The Vega must emit the Testing beep pattern before the Ground Station can arm it. See Chapter #xref("sec:Testing") for the complete safety procedure.#linebreak()#linebreak()#v(1.8pt)

*A flight or Ground Station log is missing*#linebreak()#v(-1.8pt) Vega flight logs can be opened from the Configurator's *Flight Logs* page, including logs still stored onboard. Ground Station logs are exposed on the *CATS GS* USB drive; finalize an active log before copying it, and use *Settings* → *System* → *USB Drive* before disconnecting the cable.#linebreak()#linebreak()#v(1.8pt)

*A firmware update was interrupted*#linebreak()#v(-1.8pt) Reconnect the device and start the update again using the recovery procedure in Chapter #xref("sec:FirmwareUpdates"). After any update, reconnect and verify the reported version before using the device. Normal Vega firmware installation from the Configurator is supported on Windows.#linebreak()#linebreak()#v(1.8pt)

*GNSS position is unavailable or inaccurate*#linebreak()#v(-1.8pt) Move the Vega and Ground Station outdoors with a clear view of the sky, keep antennas away from large conductive objects, and allow time for a fix. Do not rely on the last displayed coordinates until the Ground Station shows a current fix.#linebreak()#linebreak()#v(1.8pt)

If the problem remains, collect the device and firmware versions, describe the exact steps taken, and contact us on our #link("https://discord.gg/r7ErmSNvsy")[Discord server] or open an issue in the relevant #link("https://github.com/catsystems")[CATS repository].

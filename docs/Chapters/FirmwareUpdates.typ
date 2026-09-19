#import "../styles.typ": *

= Firmware Updates

#metadata(none) <sec-FirmwareUpdates> The Configurator installs official stable firmware for the CATS Vega and Ground Station. It also prepares the firmware file that the Ground Station uses to update both of its radio receivers. Use the Firmware Updates page instead of downloading files and selecting programming tools manually.

Normal firmware installation is supported on Windows. On Linux and macOS, the Configurator can check connected devices and releases, but the standard release keeps flashing disabled until those USB workflows have completed hardware acceptance.

== Before Updating

#enum(tight: false,
  [
Install the latest Configurator from the #link("https://github.com/catsystems/cats-configurator/releases/")[Configurator releases page]#source-note("https://github.com/catsystems/cats-configurator/releases"). The Firmware Updates page updates connected CATS devices; it does not update the Configurator application itself.
],
  [
Save or discard any unsaved configuration changes. Firmware installation is disabled while the Configurator contains unsaved changes.
],
  [
Connect only the device that you intend to update. Disconnect deployment charges and place the Vega safely on the bench. Stop Ground Station tracking and recording, and close any files opened from its USB drive.
],
  [
Keep the device powered and connected until the Configurator or Ground Station reports that verification has completed.
]
)

#warning[
*Warning:* Never update a Vega with deployment charges or other energetic outputs connected. Do not reset, unplug, or power off a device while firmware is being written or verified.
]

== Updating the Configurator

Download the current installer from the Configurator releases page, close the running Configurator, and run the installer. After installation, open the Configurator and confirm the expected *App version* in the footer.

#cats-figure(
  doc-image("How To Use/Configurator/Configurator_Firmware_Updates.png", width: 100%, alt: "Configurator Firmware Updates page after checking connected devices and official releases"),
  caption: [Configurator Firmware Updates page after a successful device and release check],
)

== Updating the CATS Vega

#enum(tight: false,
  [
Connect the Vega by USB, open *Firmware Updates*, and select *Check devices & releases*.
],
  [
In the Vega panel, verify the detected device, installed version, and available version. Select *Update Vega*.
],
  [
Confirm that deployment charges are disconnected. If the installed version is unknown or the same version is being reinstalled, acknowledge the additional confirmation shown by the Configurator.
],
  [
Select *Start update*. The Configurator downloads and validates the official image, moves the Vega into its native USB DFU bootloader, erases and writes the application, reads it back for verification, and starts the new firmware.
],
  [
Leave USB connected while the Vega reconnects. The update is complete only after the Configurator verifies the running firmware version.
]
)

The Configurator prevents cancellation after the device starts transitioning into its bootloader. If the application reports a failed verification or reconnect step, leave the Vega connected and use the offered retry action. STM32CubeProgrammer and an ST-Link debugger are service and recovery tools, not part of the normal update procedure.

== Updating the Ground Station

#enum(tight: false,
  [
Connect the Ground Station through its normal USB port. Stop tracking and recording, and close all files on the `CATS GS` drive.
],
  [
Open *Firmware Updates*, select *Check devices & releases*, and verify the Ground Station detected from its `version.json` information.
],
  [
Select *Update Ground Station*, confirm the safety prompt, and start the update. Keep the Ground Station connected while the Configurator enters TinyUF2, copies the official `.uf2` image, and waits for the application to return.
],
  [
The update is complete only when the Configurator reads a fresh `version.json` and reports the installed version.
]
)

If automatic bootloader entry is unavailable, use *Settings* $arrow.r$ *System* $arrow.r$ *Update Firmware* $arrow.r$ *Ground Station* on the device. Then return to the Configurator, check devices again, and start or retry the update. Do not use a 1200-baud reset; on this hardware it enters the ESP32 ROM bootloader rather than the supported TinyUF2 update path.

== Updating the Ground Station Radio Receivers

The Configurator validates and copies the radio image, while the Ground Station performs the installation on both receiver modules.

#enum(tight: false,
  [
Connect the normal `CATS GS` USB drive, open *Firmware Updates*, and select *Check devices & releases*.
],
  [
In the Ground Station radios panel, verify the detected drive and available version. Select *Prepare radio firmware* and confirm that tracking, recording, and file access have stopped.
],
  [
After preparation succeeds, close all files and safely eject the `CATS GS` drive from the computer.
],
  [
On the Ground Station, open *Settings* $arrow.r$ *System* $arrow.r$ *Update Firmware* $arrow.r$ *Radio Receivers*.
],
  [
Select the prepared `.bin` file from the `telemetry_firmware` directory and confirm the update. Keep the Ground Station powered while it updates and verifies both radios.
],
  [
Do not leave the update screen until it reports *Both radios verified* and shows the expected version for Link 1 and Link 2.
],
  [
Power-cycle the Ground Station, reconnect its normal USB drive, and select *Check devices & releases* in the Configurator to refresh all reported versions.
]
)

#note[
*Note:* A completed file-copy progress bar means only that the radio image was prepared. It does not mean that either receiver was updated. Installation and verification take place on the Ground Station.
]

#warning[
*Compatibility:* Telemetry receiver firmware 1.2.0 is the first version that can enter this updater. Receivers running version 1.1.3 or earlier require a one-time ST-Link installation of version 1.2.0 or newer. If a receiver cannot enter the updater, the Ground Station stops before erasing or writing it. Restart the Ground Station before another update attempt.
]

The following Ground Station screens show the complete on-device sequence. Filenames, versions, sizes, and checksums are examples; use the file and version prepared by your Configurator.

#cats-figure(
  figure-stack(
    spacing: 16pt,
    responsive-split(
      [#subfigure(doc-image("How To Use/Software Update/Radio_Settings.png", width: 100%, outline: true), [Open Update Firmware from the System settings page.], "a")],
      [#subfigure(doc-image("How To Use/Software Update/Radio_Receivers.png", width: 100%, outline: true), [Select Radio Receivers.], "b")],
      columns: (45%, 1fr, 45%),
    ),
    responsive-split(
      [#subfigure(doc-image("How To Use/Software Update/Radio_File.png", width: 100%, outline: true), [Select the prepared radio image.], "c")],
      [#subfigure(doc-image("How To Use/Software Update/Radio_Confirm.png", width: 100%, outline: true), [Confirm the selected image and keep power connected.], "d")],
      columns: (45%, 1fr, 45%),
    ),
    responsive-split(
      [#subfigure(doc-image("How To Use/Software Update/Radio_Progress.png", width: 100%, outline: true), [Wait while each receiver is written and verified.], "e")],
      [#subfigure(doc-image("How To Use/Software Update/Radio_Complete.png", width: 100%, outline: true), [Confirm that both radios were verified.], "f")],
      columns: (45%, 1fr, 45%),
    ),
  ),
  caption: [Ground Station radio-receiver update sequence],
  continued: false,
) <fig-RadioUpdateSequence>

== Recovering an Interrupted Update

Keep the device connected and follow the retry action shown by the Configurator. A Vega that remains in native DFU can be written and verified again. A Ground Station already in TinyUF2 can be checked again before retrying the validated update. If a radio update was interrupted, reopen the Ground Station's Radio Receivers update screen and run the prepared file again, then verify both reported versions.

If a device cannot enter its supported bootloader or cannot be detected after the documented retries, disconnect all outputs and contact CATS support. Direct programming through STM32CubeProgrammer or an ST-Link debugger is reserved for service recovery and factory provisioning.

#pagebreak()

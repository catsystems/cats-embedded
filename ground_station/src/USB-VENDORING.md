# USB core override

`USB.cpp` is rebased on the Arduino-ESP32 3.3.8 implementation supplied by
Tasmota platform 2026.05.50 (`framework-arduinoespressif32` source revision
`f2a3fa2b`).

The local delta is intentionally limited to:

- preferring the TinyUSB runtime-DFU interface when both DFU modes are enabled;
- rebooting with the TinyUF2 application reset hint `0x11F2` after DFU detach.

`USBCDC.cpp` is not overridden. The implementation from the pinned core is
used unchanged.

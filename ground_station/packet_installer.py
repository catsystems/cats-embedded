# Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Additional notice:
# This file was adapted from Florian Baumgartner's ESP32 IoT Framework 
# (https://github.com/FlorianBaumgartner/ESP32_IoT_Framework), released under MIT License.

def requirePackages():
    """Fail with an actionable message when optional upload packages are absent."""
    missing = []

    try:
        import serial  # noqa: F401
        import serial.tools.list_ports  # noqa: F401
    except ModuleNotFoundError:
        missing.append("pyserial==3.5")

    try:
        from tendo.singleton import SingleInstance  # noqa: F401
    except ModuleNotFoundError:
        missing.append("tendo==0.3.0")

    try:
        import usb.core  # noqa: F401
        import usb.backend.libusb1  # noqa: F401
    except ModuleNotFoundError:
        missing.append("pyusb==1.3.1")

    try:
        from libusb._platform import DLL_PATH  # noqa: F401
    except (ModuleNotFoundError, OSError):
        missing.append("libusb==1.0.29.post7")

    if missing:
        packages = ", ".join(missing)
        raise RuntimeError(
            "Ground Station upload dependencies are missing: "
            f"{packages}. Install them with "
            "'python -m pip install -r ground_station/requirements-upload.txt'."
        )

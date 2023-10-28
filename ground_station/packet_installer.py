# Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Additional notice:
# This file was adapted from Florian Baumgartner's ESP32 IoT Framework 
# (https://github.com/FlorianBaumgartner/ESP32_IoT_Framework), released under MIT License.

def installPackages():
    import sys
    import time
    import threading
    try:
        import serial
        import serial.tools.list_ports
    except ModuleNotFoundError:
        print("USB Modules not found, try to install them...")
        import pip
        def install(package):
            if hasattr(pip, 'main'):
                pip.main(['install', package])
            else:
                pip._internal.main(['install', package])

        install("serial")
        install("pyserial")
        import serial
        import serial.tools.list_ports
        print("USB Modules successfully installed and imported!")
        
    try:
        from tendo.singleton import SingleInstance, SingleInstanceException
    except ModuleNotFoundError:
        print("Singleton Module not found, try to install it...")
        import pip
        def install(package):
            if hasattr(pip, 'main'):
                pip.main(['install', package])
            else:
                pip._internal.main(['install', package])

        install("tendo")
        try:
            from tendo.singleton import SingleInstance, SingleInstanceException
            print("Singleton Module successfully installed and imported!")
        except ModuleNotFoundError:
            print("Please restart upload script...")
            
    try:
        import usb.core
        import usb.backend.libusb1
    except ModuleNotFoundError:
        print("USB Modules not found, try to install them...")
        import pip
        def install(package):
            if hasattr(pip, 'main'):
                pip.main(['install', package])
            else:
                pip._internal.main(['install', package])

        install("libusb")
        install("pyusb")
        import usb.core
        import usb.backend.libusb1
        print("USB Modules successfully installed and imported!")

    print("All needed packages are installed")

installPackages()

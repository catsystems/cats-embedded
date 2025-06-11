# Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Additional notice:
# This file was adapted from Florian Baumgartner's ESP32 IoT Framework 
# (https://github.com/FlorianBaumgartner/ESP32_IoT_Framework), released under MIT License.

###############################################################################
# file    dfu_reboot.py
###############################################################################
# brief   Sends DFU command to specified devices over USB interface
###############################################################################

# Use Zadig 2.7 (or later) to install drivers:
# Install driver for "TinyUSB DFU_RT (Interface x)": libusb-win32 (v1.2.6.0)

import usb.core
import usb.backend.libusb1


class DFU_Reboot:
    def __init__(self):
        pass
    
    def listDevices(self):
        deviceList = []
        backend = usb.backend.libusb1.get_backend()
        if backend is None:
            raise RuntimeError("libusb backend not found")

        devices = usb.core.find(find_all=True, backend=backend)
        for dev in devices:
            if dev is not None:
                try:
                    deviceInfo = {
                        "dev": dev,
                        "vid": dev.idVendor,
                        "pid": dev.idProduct,
                        "ser": usb.util.get_string(dev, dev.iSerialNumber) if dev.iSerialNumber else None,
                        "manufacturer": usb.util.get_string(dev, dev.iManufacturer) if dev.iManufacturer else None,
                        "product": usb.util.get_string(dev, dev.iProduct) if dev.iProduct else None,
                    }
                    deviceList.append(deviceInfo)
                except Exception:
                    pass

        deviceList = sorted(deviceList, key=lambda x: x['ser'] or "")
        return deviceList

    def reboot(self, devices):
        history = []
        for dev in devices:
            interface = 0
            status = False
            while(interface < 256 and not status):
                try:
                    dev["dev"].ctrl_transfer(bmRequestType=0x21, bRequest=0, wValue=0, wIndex=interface)
                    status = True   # Correct Interface number found
                    history.append(dev['ser'])
                    print(f"Sent DFU Command to: {dev['ser']}")
                except Exception:
                    interface += 1

        serial = [i['ser'] for i in devices]
        dif = set(serial) - set(history)
        #if(dif):
            #return [f"Could not set devices into DFU mode: {dif}"]     
        return False




if __name__ == "__main__":
    dfu = DFU_Reboot()
    devices = dfu.listDevices()
    print("devices:")
    print(devices)
    # print(dfu.reboot(devices))

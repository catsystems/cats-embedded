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
    
    def listDeviced(self):
        deviceList = []
        busses = usb.busses()
        for bus in busses:
            devices = bus.devices
            for dev in devices:
                if dev != None:
                    try:
                        deviceInfo = {"dev":dev.dev,
                                      "vid":dev.idVendor,
                                      "pid":dev.idProduct,
                                      "ser":dev.dev.serial_number,
                                      "manufacturer":dev.dev.manufacturer,
                                      "product":dev.dev.product}
                        deviceList.append(deviceInfo)
                    except:
                        pass
        deviceList = sorted(deviceList, key=lambda x: x['ser'])   # Sort list by increasing Serial number
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
    devices = dfu.listDeviced()
    print(devices)
    # print(dfu.reboot(devices))

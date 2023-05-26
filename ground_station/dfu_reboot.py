###############################################################################
# file    dfu_reboot.py
###############################################################################
# brief   Sends DFU command to specified devices over USB interface
###############################################################################
# author  Florian Baumgartner
# version 1.0
# date    2022-08-02
###############################################################################
# MIT License
#
# Copyright (c) 2022 Crelin - Florian Baumgartner
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell          
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
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

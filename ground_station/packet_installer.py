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

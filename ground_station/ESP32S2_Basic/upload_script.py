###############################################################################
# file    upload_script.py
###############################################################################
# brief   Main python script that is executed by Platform IO on firmware upload
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

Import("env")

import ast
import time
import glob
import shutil
import subprocess
from packet_installer import installPackages
from uf2_loader import UF2Loader
from dfu_reboot import DFU_Reboot

DEBUG = False

# please keep $SOURCE variable, it will be replaced with a path to firmware

# Generic
env.Replace(
    UPLOADER="executable or path to executable",
    UPLOADCMD="$UPLOADER $UPLOADERFLAGS $SOURCE"
)

# In-line command with arguments
env.Replace(
    UPLOADCMD="executable -arg1 -arg2 $SOURCE"
)

# Python callback
def on_upload(source, target, env):
    print("\n************************** Custom Upload Script ********************************")
    installPackages()
    arguments = env.GetProjectOption("upload_flags")
    firmware_path = str(source[0])
    loader = UF2Loader()
    dfu = DFU_Reboot()
    
    def getParameterString(keyword):
        paramStr = ";" + ";".join(arguments)
        index = paramStr.find(";" + keyword)
        if(index == -1):
            index = paramStr.find(keyword)
        return paramStr[index + len(keyword)::].split(";")[0].split("=")[-1]
        
    usb_serial = [getParameterString("USB_SERIAL").strip('"').strip("'")]
    usb_vid = int(getParameterString("USB_VID"), base=16)
    usb_pid = int(getParameterString("USB_PID"), base=16)
    compare_Serial = getParameterString("COMPARE_SERIAL_NUMBER").lower() == "true"
    use_serial_number_list = getParameterString("USE_SERIAL_NUMBER_LIST").lower() == "true"
    enable_automatic_console = getParameterString("ENABLE_AUTOMATIC_CONSOLE").lower() == "true"
    compare_vid_pid_console = getParameterString("COMPARE_VID_PID_CONSOLE").lower() == "true"
    use_tabs_console = getParameterString("USE_TABS_CONSOLE").lower() == "true"
    serial_number_list = ast.literal_eval(getParameterString("SERIAL_NUMBER_LIST"))
     
    if DEBUG:
        print(f"USB_SERIAL: {usb_serial}, USB_VID: {usb_vid:04X}, USB_PID: {usb_pid:04X}, COMPARE_SERIAL_NUMBER: {compare_Serial}")
        print(f"USE_SERIAL_NUMBER_LIST: {use_serial_number_list}, SERIAL_NUMBER_LIST: {serial_number_list}, ENABLE_AUTOMATIC_CONSOLE: {enable_automatic_console}, COMPARE_VID_PID_CONSOLE: {compare_vid_pid_console}, USE_TABS_CONSOLE: {use_tabs_console}")
        print(f"firmware_path: {firmware_path}")
        print(firmware_path)
        print(firmware_path.rsplit('.', 1)[0] + ".UF2")
    
    if(use_serial_number_list):
        usb_serial = serial_number_list
    
    if(enable_automatic_console):
        command = ["python", "serial_terminal.py"]
        command += [str(use_tabs_console), str(compare_vid_pid_console), str(compare_Serial), str(usb_vid), str(usb_pid), str(usb_serial)]
        CREATE_NO_WINDOW = 0x08000000
        subprocess.Popen(command, close_fds=True, creationflags=CREATE_NO_WINDOW, shell = True)
        
    firmwareFilePath = firmware_path.rsplit('.', 1)[0] + ".UF2"
    loader.save(firmware_path, firmwareFilePath)

    availableDrives = loader.get_drives()
    if not availableDrives:
        devices = dfu.listDeviced()
        for d in devices:
            if(d['ser'] == "0000000000000001"):
                devices.remove(d)
                print("Removed fake device: ['0000000000000001']")
        if not devices:
            return ['No devices found for entering bootloader, check if "libusb-win32" driver has been installed for "TinyUSB DFU_RT (Interface 1)"']
        if(compare_Serial):
            devicesFiltered = [d for d in devices if d["ser"] in usb_serial]
            if not devicesFiltered:
                return [f"No device with matching serial number {usb_serial} found, available devices: {[d['ser'] for d in devices]}"]  # Bug in python (scons) when string has more than 000 in row, then not red output?
            devices = devicesFiltered
        print(f"{len(devices)} Device{'s' if len(devices) > 1 else ''} found: {[d['ser'] for d in devices]}")
        status = dfu.reboot(devices)
        if status:
            print()
            return [status]
    else:
        devices = loader.get_drives()
        print("There is already a UF2-Drive available, skip entering bootloader (serial number cannot be compared)")

    TIMEOUT = 15        # [s]
    uploadCount = 0
    t = time.time()
    print("Start Download:", end = '')
    while(time.time() - t < TIMEOUT):
        drives = loader.get_drives()
        if(drives):
            print("\nFlashing %s (%s)" % (drives[0], loader.board_id(drives[0])), end = "")
            shutil.copyfile(firmwareFilePath, drives[0] + "/NEW.UF2")
            print(" -> OK", end = "")
            uploadCount += 1
            t = time.time()
        if uploadCount == len(devices):
            print("\nDownload was successful!")
            return False
        if(uploadCount == 0):
            print(".", end = '')
        time.sleep(0.3)   
    print()
    if not drives:
        return ["Timeout: No drive to deploy found"]
    return [f"Timeout: Only {uploadCount} of {len(devices)} have been programmed"]         # Error: Timout
    
    # return "Test"         # Warning
    # return False          # OK
    # return ["My error"]   # Error
    

env.Replace(UPLOADCMD=on_upload)

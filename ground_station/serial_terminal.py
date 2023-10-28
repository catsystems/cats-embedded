# Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Additional notice:
# This file was adapted from Florian Baumgartner's ESP32 IoT Framework 
# (https://github.com/FlorianBaumgartner/ESP32_IoT_Framework), released under MIT License.

###############################################################################
# file    serial_terminal.py
###############################################################################
# brief   Uses Windows 10 Terminal to open specified COM-Ports in multiple tabs
###############################################################################

# Disabling Warning of closing multiple tabs in Windows Terminal
# https://github.com/microsoft/terminal/issues/5125


import ast
import sys
import threading
import subprocess
import win32gui
from pathlib import Path
from serial_console import Console
from tendo.singleton import SingleInstance, SingleInstanceException


DEBUG = False

class Terminal:
    def __init__(self, useTabs, enableCompUsb, enableCompSer, vid, pid, ser, closeWidthVscode=True):
        self.runThread = True
        self.vsCode = closeWidthVscode
        
        self.USE_TABS = useTabs
        self.ENABLE_COMP_USB = enableCompUsb
        self.ENABLE_COMP_SER = enableCompSer
        self.COMPARE_VID = vid
        self.COMPARE_PID = pid
        self.COMPARE_SER = ser
        
        self.portsBeingOpened = []


    def getOpenPorts(self):
        def winEnumHandler(hwnd, ctx):
            if win32gui.IsWindowVisible(hwnd):
                apps.append(win32gui.GetWindowText(hwnd))
        apps = []
        win32gui.EnumWindows(winEnumHandler, apps)

        
        ports = []
        visualStudioRunning = False
        for i in apps:
            if i.startswith("COM"):
                ports.append(i.split()[0])
            if i.strip().endswith("Visual Studio Code"):
                visualStudioRunning = True
        if(not visualStudioRunning):
            print(apps)
        return ports, visualStudioRunning
    
    def start(self):
        self.thread = threading.Thread(target=self.run, name='MyThread')
        self.thread.start()
        
    def stop(self):
        self.runThread = False

    def run(self):
        try:
            me = SingleInstance()
        except SingleInstanceException:
            self.runThread = False
            return       
        
        while(self.runThread):
            availablePorts = Console.getPorts()
            openPorts, vsCodeRunning = self.getOpenPorts()
            portsToOpen = []
            for p in availablePorts:
                if(self.ENABLE_COMP_USB and (p.vid != self.COMPARE_VID or p.pid != self.COMPARE_PID)):
                    continue
                if(self.ENABLE_COMP_SER and (p.ser not in self.COMPARE_SER)):
                    continue
                if(p.port in openPorts):
                    continue
                if(p.port in self.portsBeingOpened):
                    continue
                self.portsBeingOpened.append(p.port)
                portsToOpen.append(p)
            availablePorts.sort(key=lambda x: x.ser)
            portsToOpen.sort(key=lambda x: x.ser)
            self.portsBeingOpened = [p for p in self.portsBeingOpened if p not in openPorts]

            if(self.vsCode and not vsCodeRunning):
                if DEBUG:
                    print("Virtual Studio Code is not running -> terminating")
                self.runThread = False
            
            if(not self.runThread):
                break
            
            if("COM..." in availablePorts):
                if DEBUG:
                    print("COM... detected -> Skip")
                continue
            
            if DEBUG:
                print(f"Already open Ports: {openPorts}")
                print(f"Available ports: {[i.port for i in portsToOpen]}")
                print(f"New ports to open: {[i.port for i in portsToOpen]}") 
                print(f"Ports that are being opened: {self.portsBeingOpened}")    
                
            
            scriptPath = Path.cwd() / "serial_console.py"
            processResult = []
            try:
                if self.USE_TABS:
                    command = ["wt.exe"]
                    for i, port in enumerate(portsToOpen):
                        tabName = f"{port.port} (SER: {port.ser})"
                        command.extend(["--title", tabName, "python", scriptPath, port.port])
                        if(i < len(portsToOpen) - 1):
                            command.append(";")
                    processResult = subprocess.run(command)
                else:
                    for i, port in enumerate(portsToOpen):
                        tabName = f"{port.port} (SER: {port.ser})"
                        command = ["wt.exe", "--title", tabName, "python", scriptPath, port.port]
                        processResult.append(subprocess.run(command))
            
            except FileNotFoundError:
                print("Please Install Windows Terminal: https://docs.microsoft.com/en-us/windows/terminal")
                self.runThread = False



if __name__ == '__main__':
    use_tabs_console = False
    compare_vid_pid_console = True
    compare_Serial = False
    usb_vid = 0x239A
    usb_pid = 0x80AB
    usb_serial = ["0", "1", "2"]
    
    if(len(sys.argv) >= 7):
        use_tabs_console = sys.argv[1].lower() == "true"
        compare_vid_pid_console = sys.argv[2].lower() == "true"
        compare_Serial = sys.argv[3].lower() == "true"
        usb_vid = int(sys.argv[4])
        usb_pid = int(sys.argv[5])
        usb_serial = ast.literal_eval(sys.argv[6])
        
        if DEBUG:
            print(f"USB_SERIAL: {usb_serial}, USB_VID: {usb_vid:04X}, USB_PID: {usb_pid:04X}")
            print(f"COMPARE_SERIAL_NUMBER: {compare_Serial}, COMPARE_VID_PID_CONSOLE: {compare_vid_pid_console}, USE_TABS_CONSOLE: {use_tabs_console}")
    
    terminal = Terminal(closeWidthVscode=True, useTabs=use_tabs_console,
                        enableCompUsb=compare_vid_pid_console,
                        enableCompSer=compare_Serial,
                        vid=usb_vid, pid=usb_pid, ser=usb_serial)
    terminal.start()
    
    import time
    try:
        while terminal.runThread:
            time.sleep(0.1)
    except KeyboardInterrupt:
        terminal.stop()
    
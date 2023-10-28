# Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Additional notice:
# This file was adapted from Florian Baumgartner's ESP32 IoT Framework 
# (https://github.com/FlorianBaumgartner/ESP32_IoT_Framework), released under MIT License.

###############################################################################
# file    serial_console.py
###############################################################################
# brief   Handles physical communication to specified COM-Port (USB CDC)
###############################################################################

import sys
import time
import threading
import serial
import serial.tools.list_ports


class Port:
    def __init__(self, port, vid, pid, ser):
        self.port = port
        self.vid = vid
        self.pid = pid
        self.ser = ser
    
    def __str__(self):
        return f"{self.port} (VID: {self.vid:04X}, PID: {self.pid:04X}, SER: {self.ser})"


class Console(threading.Thread):
    def __init__(self):
        self.UPDATE_FREQ = 10    # [Hz]
        self.TIMEOUT     = 6     # [s]
        
        threading.Thread.__init__(self)
        self._data = ""
        self.ser = None
        self.runThread = True
        self.processing = False


    def getPorts():
        ports = []
        for port, desc, hwid in sorted(serial.tools.list_ports.comports()):
            try:
                vid = int(hwid.split("PID=")[1].split(":")[0], base=16)
                pid = int(hwid.split("PID=")[1].split(":")[1].split(" ")[0], base=16)
                ser = hwid.split("SER=")[1].split(" ")[0]
                ports.append(Port(port, vid, pid, ser))
            except:
                pass
        return ports

            
    def run(self):
        while(self.runThread):
            try:
                time.sleep(1 / self.UPDATE_FREQ)
                ser = serial.Serial(comPort, 115200, timeout=self.TIMEOUT)         

            except serial.SerialException:
                try:
                    ser.close()
                except UnboundLocalError:
                    pass
                continue
            except KeyboardInterrupt:
                self.runThread = False
                
            while self.runThread:
                try:
                    self._data = ser.read().decode('UTF-8');
                    if self._data:
                        sys.stdout.write(self._data)
                        sys.stdout.flush()
                    
                except serial.SerialException:
                    ser.close()
                    break
                except UnicodeDecodeError:  # Ignore characters that cannot be printed
                    pass
                except KeyboardInterrupt:
                    self.runThread = False
                    self.processing = False
        try:
            ser.close()
        except UnboundLocalError:
            pass

    def getData(self):
        return self._data
    
    def terminate(self):
        self.runThread = False
    


if __name__ == '__main__':
    if(len(sys.argv) <= 1):
        print("Please specifiy a COM-Port, for example: COM34")
    else:
        comPort = sys.argv[1]
        # print(f"Start Console on port {comPort}\n")
    
    # print(f"Process ID: {os.getpid()}")
    # for p in Console.getPorts():
    #     print(p)

    console = Console()
    console.start()
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        console.terminate()
    
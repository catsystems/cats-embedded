###############################################################################
# file    serial_console.py
###############################################################################
# brief   Handles physical communication to specified COM-Port (USB CDC)
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
    
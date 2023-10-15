###############################################################################
# file    uf2_loader.py
###############################################################################
# brief   Binary executable to UF2-Format converter and automatic download tool
###############################################################################
# author  Microsoft & Florian Baumgartner
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
import struct
import shutil
import subprocess
import re
import os
import os.path
import glob

class Namespace:
            def __init__(self, **kwargs):
                self.__dict__.update(kwargs)

class UF2Loader:
    def __init__(self, family ="ESP32S2", startAddr=0):
        self.UF2_MAGIC_START0 = 0x0A324655   # "UF2\n"
        self.UF2_MAGIC_START1 = 0x9E5D5157   # Randomly selected
        self.UF2_MAGIC_END    = 0x0AB16F30   # Ditto
        
        self.SEARCH_PATH = "esp32-firmware-update/*.bin"
        self.INFO_FILE = "/INFO_UF2.TXT"
        self.appstartaddr = startAddr
        self.familyid = 0x0
        
        self.families = {
            'SAMD21': 0x68ed2b88,
            'SAML21': 0x1851780a,
            'SAMD51': 0x55114460,
            'NRF52': 0x1b57745f,
            'STM32F1': 0x5ee21072,
            'STM32F4': 0x57755a57,
            'ATMEGA32': 0x16573617,
            'MIMXRT10XX': 0x4FB2D5BD,
            'ESP32S2': 0xBFDD4EEE,
        }

        self.args = Namespace(
            base=startAddr,
            carray=False,
            convert=False,
            deploy=False,
            device_path=None,
            family=family,
            input=None,
            list=False,
            output=None
        )
        
        
    def is_uf2(self, buf):
        w = struct.unpack("<II", buf[0:8])
        return w[0] == self.UF2_MAGIC_START0 and w[1] == self.UF2_MAGIC_START1
    
    def is_hex(self, buf):
        try:
            w = buf[0:30].decode("utf-8")
        except UnicodeDecodeError:
            return False
        if w[0] == ':' and re.match(b"^[:0-9a-fA-F\r\n]+$", buf):
            return True
        return False
    
    def convert_from_uf2(self, buf):
        numblocks = len(buf) // 512
        curraddr = None
        outp = b""
        for blockno in range(numblocks):
            ptr = blockno * 512
            block = buf[ptr:ptr + 512]
            hd = struct.unpack(b"<IIIIIIII", block[0:32])
            if hd[0] != self.UF2_MAGIC_START0 or hd[1] != self.UF2_MAGIC_START1:
                print("Skipping block at " + ptr + "; bad magic")
                continue
            if hd[2] & 1:
                # NO-flash flag set; skip block
                continue
            datalen = hd[4]
            if datalen > 476:
                assert False, "Invalid UF2 data size at " + ptr
            newaddr = hd[3]
            if curraddr == None:
                self.appstartaddr = newaddr
                curraddr = newaddr
            padding = newaddr - curraddr
            if padding < 0:
                assert False, "Block out of order at " + ptr
            if padding > 10*1024*1024:
                assert False, "More than 10M of padding needed at " + ptr
            if padding % 4 != 0:
                assert False, "Non-word padding size at " + ptr
            while padding > 0:
                padding -= 4
                outp += b"\x00\x00\x00\x00"
            outp += block[32 : 32 + datalen]
            curraddr = newaddr + datalen
        return outp
    
    def convert_to_carray(self, file_content):
        outp = "const unsigned char bindata[] __attribute__((aligned(16))) = {"
        for i in range(len(file_content)):
            if i % 16 == 0:
                outp += "\n"
            outp += "0x%02x, " % ord(file_content[i])
        outp += "\n};\n"
        return outp
    
    def convert_to_uf2(self, file_content):
        datapadding = b""
        while len(datapadding) < 512 - 256 - 32 - 4:
            datapadding += b"\x00\x00\x00\x00"
        numblocks = (len(file_content) + 255) // 256
        outp = b""
        for blockno in range(numblocks):
            ptr = 256 * blockno
            chunk = file_content[ptr:ptr + 256]
            flags = 0x0
            if self.familyid:
                flags |= 0x2000
            hd = struct.pack(b"<IIIIIIII",
                self.UF2_MAGIC_START0, self.UF2_MAGIC_START1,
                flags, ptr + self.appstartaddr, 256, blockno, numblocks, self.familyid)
            while len(chunk) < 256:
                chunk += b"\x00"
            block = hd + chunk + datapadding + struct.pack(b"<I", self.UF2_MAGIC_END)
            assert len(block) == 512
            outp += block
        return outp
    
    def convert_from_hex_to_uf2(self, buf):
        self.appstartaddr = None
        upper = 0
        currblock = None
        blocks = []
        for line in buf.split('\n'):
            if line[0] != ":":
                continue
            i = 1
            rec = []
            while i < len(line) - 1:
                rec.append(int(line[i:i+2], 16))
                i += 2
            tp = rec[3]
            if tp == 4:
                upper = ((rec[4] << 8) | rec[5]) << 16
            elif tp == 2:
                upper = ((rec[4] << 8) | rec[5]) << 4
                assert (upper & 0xffff) == 0
            elif tp == 1:
                break
            elif tp == 0:
                addr = upper | (rec[1] << 8) | rec[2]
                if self.appstartaddr == None:
                    self.appstartaddr = addr
                i = 4
                while i < len(rec) - 1:
                    if not currblock or currblock.addr & ~0xff != addr & ~0xff:
                        currblock = self.Block(addr & ~0xff)
                        blocks.append(currblock)
                    currblock.bytes[addr & 0xff] = rec[i]
                    addr += 1
                    i += 1
        numblocks = len(blocks)
        resfile = b""
        for i in range(0, numblocks):
            resfile += blocks[i].encode(i, numblocks)
        return resfile
    
    def to_str(self, b):
        return b.decode("utf-8")
    
    def get_drives(self):
        drives = []
        if sys.platform == "win32":
            r = subprocess.check_output(["wmic", "PATH", "Win32_LogicalDisk",
                                         "get", "DeviceID,", "VolumeName,",
                                         "FileSystem,", "DriveType"])
            for line in self.to_str(r).split('\n'):
                words = re.split('\s+', line)
                if len(words) >= 3 and words[1] == "2" and words[2] == "FAT":
                    drives.append(words[0])
        else:
            rootpath = "/media"
            if sys.platform == "darwin":
                rootpath = "/Volumes"
            elif sys.platform == "linux":
                tmp = rootpath + "/" + os.environ["USER"]
                if os.path.isdir(tmp):
                    rootpath = tmp
            for d in os.listdir(rootpath):
                drives.append(os.path.join(rootpath, d))
    
    
        def has_info(d):
            try:
                return os.path.isfile(d + self.INFO_FILE)
            except:
                return False
    
        return list(filter(has_info, drives))
    
    
    def board_id(self, path):
        with open(path + self.INFO_FILE, mode='r') as file:
            file_content = file.read()
        return re.search("Board-ID: ([^\r\n]*)", file_content).group(1)
    
    
    def list_drives(self):
        for d in self.get_drives():
            print(d, self.board_id(d))
    
    
    def write_file(self, name, buf):
        with open(name, "wb") as f:
            f.write(buf)
        print("Wrote %d bytes to %s" % (len(buf), name))

    def save(self, inputPath, savePath):
        #self.appstartaddr = int(self.args.base, 16)
        self.familyid = self.families[self.args.family]
        self.args.input = glob.glob(inputPath)[0]
        self.args.output = savePath

        print("Input: ", self.args.input)
        print("Output: ", self.args.output)
            
        with open(self.args.input, mode='rb') as f:
            inpbuf = f.read()
        from_uf2 = self.is_uf2(inpbuf)
        ext = "uf2"
        if self.args.deploy:
            outbuf = inpbuf
        elif from_uf2:
            outbuf = self.convert_from_uf2(inpbuf)
            ext = "bin"
        elif self.is_hex(inpbuf):
            outbuf = self.convert_from_hex_to_uf2(inpbuf.decode("utf-8"))
        elif self.args.carray:
            outbuf = self.convert_to_carray(inpbuf)
            ext = "h"
        else:
            outbuf = self.convert_to_uf2(inpbuf)

        print("Converting to %s, output size: %d, start address: 0x%x" % (ext, len(outbuf), self.appstartaddr))
        self.write_file(self.args.output, outbuf)
    
    
    def download(self, inputPath):
        drives = self.get_drives()
        if len(drives) == 0:
            return("No drive to deploy.")
        print()
        tempFilePath = os.getcwd() + "/TMP.UF2"
        self.save(inputPath, tempFilePath)
        for d in drives:
            print("Flashing %s (%s)" % (d, self.board_id(d)))
            shutil.copyfile(tempFilePath, d + "/NEW.UF2")
        if os.path.exists(tempFilePath):
            os.remove(tempFilePath)
        return None

# If this script is called directly, it converts the .bin binary into the .UF2 binary
if __name__ == "__main__":
    loader = UF2Loader("ESP32S2", 0)
    binary_path = ".pio/build/ground_station/firmware"
    # if an argument is passed to the script, use it as the path to the binary
    if len(sys.argv) > 1:
        binary_path = sys.argv[1]
    input = f"{binary_path}.bin"
    output = f"{binary_path}.UF2"
    status = loader.save(input, output)
    print(f"Status: {status}")

from argparse import ArgumentParser
import serial.tools.list_ports
from serial import Serial
import json
import time
import re
import os
from datetime import datetime
from pathlib import Path

BOOT_STATE_MAP = {
  'CATS_INVALID': 0,
  'CATS_IDLE': 1,
  'CATS_CONFIG': 2,
  'CATS_TIMER': 3,
  'CATS_DROP': 4,
  'CATS_FLIGHT': 5
}

BOOL_MAP = {
    False: 0,
    True: 1
}

def prepare_config_msg(config):
    boot_state = BOOT_STATE_MAP[config['boot_state']]
    clear_flash = BOOL_MAP[config['clear_flash']]
    clear_log_info = BOOL_MAP[config['clear_log_info']]
    return f"CFG:{boot_state};{clear_flash};{clear_log_info}"



if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument("-c", "--cfg", type=str, help='configuration file')
    args = parser.parse_args()
    print(args.cfg)
    if args.cfg is None:
        load_new_config = False
    else:
        load_new_config = True
        with open(args.cfg) as f:
            config = json.load(f)
            print(f'Config found:\n{config}')
            print(f'Format to be sent to C.A.T.S.: {prepare_config_msg(config)}')
    
    #wait for connection
    while True:
        for port in serial.tools.list_ports.comports():
            if(port.description.startswith('STMicroelectronics Virtual COM Port')):
                break
        else:
            print("No C.A.T.S. found, sleeping for 1 second")
            time.sleep(1)
            continue
        break
    print(f'C.A.T.S. found at {port}')

    #load config
    if load_new_config:
        start_time = time.time()
        current_time = start_time
        with serial.Serial(port.name, 115200, timeout=10) as ser:
            #wait for 10 seconds to receive the commands
            while (current_time - start_time) < 10:
                current_time = time.time()
                line = ser.readline().decode().rstrip()
                print(f'C.A.T.S. says: "{line}"')
                if line == 'What is my purpose?':
                    print('PC says: "Wait for instructions!"')
                    ser.write(b'Wait for instructions!')
                    ack_line = ser.readline().decode().rstrip()
                    print(f'C.A.T.S. says: "{ack_line}"')
                    if ack_line == 'OK!':
                        ser.write(b'Hello from PC!')
                        time.sleep(0.1)
                        cfg_str = prepare_config_msg(config)
                        ser.write(cfg_str.encode())
                        read_cfg_response_len = 0
                        while read_cfg_response_len < 10:
                            cfg_response_line = ser.readline().decode().rstrip()
                            print(f'C.A.T.S. says: "{cfg_response_line}"')
                            read_cfg_response_len += 1
                    break

    #create logging dir
    now = datetime.now()
    current_flight_dir = str(now.strftime("%d_%m_%Y_%H_%M_%S"))
    full_path = os.path.join(os.getcwd(), 'flight_logs', current_flight_dir)
    Path(full_path).mkdir(parents=True, exist_ok=True)

    print('Reading the logs..')
    #save logs
    with serial.Serial(port.name, 115200, timeout=30) as ser:
        first_flight_found = False
        current_flight = '0'
        while first_flight_found == False:
            line = ser.readline().decode().rstrip()
            if f_rx := re.search(r'Recording of flight #(\d+):', line):
                print("First flight found!")
                current_flight = f_rx.groups(0)[0]
                first_flight_found = True
                break
            else:
                print(line)
        while True:
            filename = f'flight_{current_flight}.log'
            log_path = os.path.join(full_path, filename)
            with open(log_path, 'w') as log_file:
                while (line := ser.readline().decode().rstrip()) != 'Flight reader done':
                    if f_rx := re.search(r'Recording of flight #(\d+):', line):
                        current_flight = f_rx.groups(0)[0]
                        print(f'Log saved to {log_path}')
                        print(f"Current flight: {current_flight}")
                        break
                    else:
                        log_file.write(f'{line}\r\n') 
                else:
                    print(line)
                    print('All flights downloaded!')
                    break
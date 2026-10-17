# Copyright (C) 2026 Control and Telemetry Systems GmbH
# SPDX-License-Identifier: GPL-3.0-or-later

"""Run production recorder/SdFat/FlashCache and selected production HMI bodies on Linux.

Usage from the repo: python3 ground_station/tests/recorder_host/run.py
Only generated files under ground_station/.pio/recorder-host are written.
"""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
OUT = ROOT / '.pio' / 'recorder-host'
SD = ROOT / '.pio/libdeps/ground_station/SdFat - Adafruit Fork/src'
FLASH = ROOT / '.pio/libdeps/ground_station/Adafruit SPIFlash/src'


def run(args, **kwargs):
    subprocess.run([str(arg) for arg in args], check=True, timeout=180, **kwargs)


def body(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'


OUT.mkdir(parents=True, exist_ok=True)
# Same FatFs formatter as the compatibility test, with the GS's 960 KiB volume.
formatter = (ROOT / 'tests/fatfs_compatibility.c').read_text().replace(
    '#define TEST_SECTOR_COUNT 512U', '#define TEST_SECTOR_COUNT 1920U')
(OUT / 'format.c').write_text(formatter)
fat = ROOT / 'lib/FatFs'
run(['gcc', OUT / 'format.c', fat / 'ff.c', fat / 'ffunicode.c', '-I' + str(fat), '-o', OUT / 'format'])
run([OUT / 'format', 'create', OUT / 'blank.img'])

hmi = (ROOT / 'src/hmi/hmi.cpp').read_text()
utils = (ROOT / 'src/utils.cpp').read_text()
extracted = '#include "hmi_host.hpp"\n#include <cmath>\nextern SemaphoreHandle_t storageAccessMutex;\n'
for signature in ('void Hmi::initLive()', 'void Hmi::live()', 'void Hmi::update(void *pvParameter)'):
    extracted += body(hmi, signature)
extracted += body(utils, 'int32_t Utils::getFlashMemoryUsage()')
(OUT / 'hmi.cpp').write_text(extracted)

flags = ['-std=c++17', '-O1', '-g', '-pthread', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
         '-DENABLE_ARDUINO_FEATURES=0', '-DENABLE_ARDUINO_SERIAL=0', '-DENABLE_ARDUINO_STRING=0',
         '-DSPI_DRIVER_SELECT=3', '-D__FlashStringHelper=void', '-include', str(HERE / 'Arduino.h'),
         '-I' + str(HERE), '-I' + str(ROOT / 'src'), '-I' + str(ROOT / 'src/logging'),
         '-I' + str(SD), '-I' + str(FLASH)]


def compile_one(source, name):
    target = OUT / (name + '.o')
    run(['g++', *flags, '-c', source, '-o', target])
    return target


libraries = [compile_one(source, 'lib-' + source.stem)
             for folder in (SD / 'FatLib', SD / 'common') for source in sorted(folder.glob('*.cpp'))]
libraries += [compile_one(FLASH / 'Adafruit_FlashCache.cpp', 'flash-cache'),
              compile_one(HERE / 'test.cpp', 'test')]
recorder_source = (ROOT / 'src/logging/recorder.cpp').read_text()
recorder = compile_one(ROOT / 'src/logging/recorder.cpp', 'recorder')
hmi_object = compile_one(OUT / 'hmi.cpp', 'hmi')


def link(name, recorder_object, hmi_part):
    binary = OUT / name
    run(['g++', *flags, *libraries, recorder_object, hmi_part, '-o', binary])
    return binary


binary = link('recorder-test', recorder, hmi_object)
run([binary, OUT / 'blank.img', 'all'])

# Negative controls: each regression test must reject the corresponding old code.
mutations = [
    ('down', 'recorder', recorder_source.replace(
        '(ref->fileCreated || command.data.state < kTouchdownState)', 'true')),
    ('exit', 'hmi', extracted.replace('else if (needsConfirmation)',
                                    "else if (recorder.getStatus().activeFilename[0] != '\\0')")),
    ('scan', 'hmi', extracted.replace('ref->state != LIVE && !ref->isLogging', '!ref->isLogging')),
]
for name, part, source in mutations:
    original = recorder_source if part == 'recorder' else extracted
    assert source != original, f'{name}: mutation no longer matches production source'
    path = OUT / (name + '-old.cpp')
    path.write_text(source)
    obj = compile_one(path, name + '-old')
    mutant = link(name + '-old', obj if part == 'recorder' else recorder,
                  obj if part == 'hmi' else hmi_object)
    result = subprocess.run([str(mutant), str(OUT / 'blank.img'), name], timeout=30,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (OUT / (name + '-old.log')).write_text(result.stdout)
    if result.returncode == 0 or 'FAIL:' not in result.stdout:
        sys.exit(f'Negative control {name} did not fail its assertion: {result.stdout}')
    print(f'PASS: negative control {name} rejected the old behavior', flush=True)
print('PASS: production recorder/filesystem tests and all three negative controls', flush=True)

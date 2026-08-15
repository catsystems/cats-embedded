"""Verify modernization pins and protected Ground Station artifacts."""

from __future__ import annotations

import hashlib
import struct
from pathlib import Path


GROUND_STATION = Path(__file__).resolve().parent
REPOSITORY = GROUND_STATION.parent


def require_text(path: Path, fragments: tuple[str, ...]) -> None:
    content = path.read_text(encoding="utf-8")
    missing = [fragment for fragment in fragments if fragment not in content]
    if missing:
        raise RuntimeError(f"{path}: missing required contract entries: {missing}")


def git_blob_id(path: Path) -> str:
    data = path.read_bytes().replace(b"\r\n", b"\n")
    header = f"blob {len(data)}\0".encode()
    return hashlib.sha1(header + data).hexdigest()  # noqa: S324 - Git uses SHA-1 blob IDs.


def normalized_text_sha256(path: Path) -> str:
    """Hash a text file independently of the checkout's CRLF policy."""
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def verify_qmc5883_compass() -> None:
    manifest = GROUND_STATION / "dependency-locks" / "QMC5883Compass.git-blobs"
    expected = {}
    for line in manifest.read_text(encoding="utf-8").splitlines():
        digest, relative_path = line.split(maxsplit=1)
        expected[relative_path] = digest

    library_root = GROUND_STATION / "lib" / "QMC5883Compass"
    actual_paths = {
        path.relative_to(REPOSITORY).as_posix()
        for path in library_root.rglob("*")
        if path.is_file()
    }
    if actual_paths != set(expected):
        raise RuntimeError("QMC5883Compass file set differs from the protected manifest")

    for relative_path, expected_digest in expected.items():
        actual_digest = git_blob_id(REPOSITORY / relative_path)
        if actual_digest != expected_digest:
            raise RuntimeError(f"Protected QMC5883Compass file changed: {relative_path}")

    require_text(
        library_root / "library.properties",
        ("name=QMC5883Compass", "version=1.2.3"),
    )
    require_text(
        library_root / "CATS-VENDORING.md",
        (
            "b6ff8bfab322db558e9eeed516ac617eec0b6af5",
            "runtime selection between QMC5883L",
            "no call to `Wire.end()`",
        ),
    )
    require_text(
        library_root / "src" / "QMC5883Compass.hpp",
        ("kQMC5883L", "kQMC5883P", "readRaw", "readCalibrated", "createSensor"),
    )


def verify_uf2_update(path: Path) -> None:
    data = path.read_bytes()
    if len(data) == 0 or len(data) % 512 != 0:
        raise RuntimeError(f"Invalid UF2 update size: {path}")

    block_count = len(data) // 512
    for block_index in range(block_count):
        offset = block_index * 512
        header = struct.unpack_from("<8I", data, offset)
        trailer = struct.unpack_from("<I", data, offset + 508)[0]
        if header[0:2] != (0x0A324655, 0x9E5D5157) or trailer != 0x0AB16F30:
            raise RuntimeError(f"Invalid UF2 block framing: {path}")
        if (header[2] & 0x2000) == 0 or header[4] != 256 or header[5] != block_index:
            raise RuntimeError(f"Unexpected UF2 block layout: {path}")
        if header[6] != block_count or header[7] != 0xBFDD4EEE:
            raise RuntimeError(f"Unexpected ESP32-S2 UF2 metadata: {path}")


def verify_bootloaders() -> None:
    expected = {
        "tinyuf2-espressif_saola_1_wroom-0.10.2_combined.bin":
            "f63f414562e66e5fa4305815a56eec1d2e8118dae03e18825fa76bd00b0492ca",
        "tinyuf2-espressif_saola_1_wrover-0.10.2_combined.bin":
            "86a7f31c4015a10744a931fc541824975bf95319f21fd64d83aaf5c42a1047e7",
        "tinyuf2-espressif_saola_1_wroom-0.35.0_combined-ota.bin":
            "2c61ed6ed4545cc79f27f2d4c356a0a6567a281675fdb7285dea3b266444337f",
        "tinyuf2-espressif_saola_1_wrover-0.35.0_combined-ota.bin":
            "12f95fd2814589422b94d943d1a8218da1887fff2b74987d96143d6dd145e51a",
        "update-tinyuf2-espressif_saola_1_wroom-0.35.0.uf2":
            "8d760c32b4c35eb9c23a61e102d43ab22a694e933a4c1e2c5728ff7bd847d23a",
        "update-tinyuf2-espressif_saola_1_wrover-0.35.0.uf2":
            "1bb601567b78adad890b68c4f99a6d572e57b3ada608988e11569798b6c0a9bb",
    }
    bootloader_root = GROUND_STATION / "bootloader"
    for filename, expected_digest in expected.items():
        path = bootloader_root / filename
        actual_digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual_digest != expected_digest:
            raise RuntimeError(f"Protected TinyUF2 image changed: {filename}")

    expected_partition_digest = "48558625a6e1830373113b0afec542752420284730eae278c743a85ebdb804bd"
    for variant in ("wroom", "wrover"):
        combined = bootloader_root / f"tinyuf2-espressif_saola_1_{variant}-0.35.0_combined-ota.bin"
        combined_data = combined.read_bytes()
        partition_digest = hashlib.sha256(combined_data[0x8000:0x8C00]).hexdigest()
        if partition_digest != expected_partition_digest:
            raise RuntimeError(f"TinyUF2 {variant} image does not contain the Ground Station OTA partition table")
        if b"0.35.0" not in combined_data:
            raise RuntimeError(f"TinyUF2 {variant} image does not identify itself as 0.35.0")
        verify_uf2_update(bootloader_root / f"update-tinyuf2-espressif_saola_1_{variant}-0.35.0.uf2")


def main() -> None:
    platformio = GROUND_STATION / "platformio.ini"
    require_text(
        platformio,
        (
            "platform-espressif32/releases/download/2026.05.50/platform-espressif32.zip",
            "adafruit/Adafruit SPIFlash@5.1.1",
            "adafruit/SdFat - Adafruit Fork@2.3.103",
            "bblanchon/ArduinoJson@7.4.3",
            "jchristensen/JC_Button@2.1.6",
            "QMC5883Compass ; This library was modified by us",
            "paulstoffregen/Time@1.6.1",
            "-D USB_VID=0x239A",
            "-D USB_PID=0x80AB",
            "-DUSB_MANUFACTURER='\"CATS\"'",
            "-DUSB_PRODUCT='\"CATS Ground Station\"'",
        ),
    )
    platformio_text = platformio.read_text(encoding="utf-8")
    forbidden = (
        "Adafruit TinyUSB Library",
        "CFG_TUSB_",
        "CFG_TUD_",
        "toolchain-xtensa-esp32s2",
    )
    if any(fragment in platformio_text for fragment in forbidden):
        raise RuntimeError("External TinyUSB, obsolete TinyUSB flags, or toolchain overrides are present")
    if (GROUND_STATION / "src" / "USBCDC.cpp").exists():
        raise RuntimeError("USBCDC.cpp must come from the pinned Arduino-ESP32 core")
    require_text(
        GROUND_STATION / "src" / "USB.cpp",
        (
            "#if CFG_TUD_DFU_RUNTIME",
            "TUD_DFU_RT_DESCRIPTOR",
            "APP_REQUEST_UF2_RESET_HINT = 0x11F2",
        ),
    )
    require_text(
        GROUND_STATION / "src" / "utils.cpp",
        (
            '#include "USBMSC.h"',
            "USBMSC usb_msc;",
            "tud_msc_write10_complete_cb",
        ),
    )

    require_text(GROUND_STATION / "lib" / "Adafruit_GFX_Library" / "library.properties", ("version=1.12.6",))
    require_text(GROUND_STATION / "lib" / "Adafruit_BusIO" / "library.properties", ("version=1.17.4",))
    require_text(
        GROUND_STATION / "lib" / "Adafruit_SHARP_Memory_Display" / "library.properties",
        ("version=1.1.4",),
    )
    require_text(GROUND_STATION / "lib" / "LSM6DS3" / "library.properties", ("version=1.0.3",))
    require_text(GROUND_STATION / "lib" / "MadgwickAHRS" / "library.properties", ("version=1.2.0",))
    require_text(
        GROUND_STATION / "lib" / "FatFs" / "ff.c",
        (
            "R0.16 w/patch 2",
            "#define MIN_EXFAT",
            "fasize >= 0x200000",
            "XDIR_NumLabel] && si < 11",
        ),
    )
    require_text(
        GROUND_STATION / "lib" / "FatFs" / "ffconf.h",
        (
            "#define FFCONF_DEF\t80386",
            "#define FF_USE_MKFS\t\t1",
            "#define FF_USE_LABEL\t1",
            "#define FF_CODE_PAGE\t437",
            "#define FF_USE_LFN\t\t1",
            "#define FF_MULTI_PARTITION\t1",
            "#define FF_MIN_SS\t\t512",
            "#define FF_MAX_SS\t\t512",
            "#define FF_LBA64\t\t0",
            "#define FF_FS_EXFAT\t\t1",
            "#define FF_FS_NORTC\t\t1",
            "#define FF_FS_REENTRANT\t0",
        ),
    )
    fatfs_hashes = {
        "ff.c": "1a6a5cd8bc17e82a6d1eeeefdecfb0cb1672eb82d0ac254eab3ce3877c59aa5a",
        "ff.h": "86099100a6ef3c3623dfbeaacb3a2613ff266df76aad1386513042b831c5b733",
        "ffconf.h": "33be28306d67491d439a1c4b625a123de366c2f13e69bf5c5535e1db2db46b93",
        "diskio.h": "2279ac777bc7a8a317b1b44519df8604e4cdd0cd3b1608861eed3c78045d89c6",
        "ffunicode.c": "6cd4dc4a664386ebd108e56d367582c35f005fc20381486366b5d77ac58119bd",
    }
    fatfs_root = GROUND_STATION / "lib" / "FatFs"
    for filename, expected_digest in fatfs_hashes.items():
        actual_digest = normalized_text_sha256(fatfs_root / filename)
        if actual_digest != expected_digest:
            raise RuntimeError(f"Vendored FatFs source changed unexpectedly: {filename}")
    fixture = GROUND_STATION / "tests" / "fixtures" / "fatfs-r013c-sfd.img.gz"
    if hashlib.sha256(fixture.read_bytes()).hexdigest() != (
        "c743085a59696f2627c4e0459137af7b6536f73a580ded0689ffdddcb1b22bbd"
    ):
        raise RuntimeError("FatFs R0.13c compatibility fixture changed unexpectedly")
    require_text(
        GROUND_STATION / "src" / "utils.cpp",
        ("PARTITION VolToPart[FF_VOLUMES] = {{0, 0}};", "f_mkfs(\"\", &formatOptions"),
    )
    if not (GROUND_STATION / "lib" / "FatFs" / "ffunicode.c").is_file():
        raise RuntimeError("FatFs LFN support requires the vendored ffunicode.c")

    verify_qmc5883_compass()
    verify_bootloaders()
    print("Ground Station upgrade contract verified")


if __name__ == "__main__":
    main()

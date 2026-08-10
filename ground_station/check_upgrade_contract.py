"""Verify modernization pins and protected Ground Station artifacts."""

from __future__ import annotations

import hashlib
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


def verify_bootloaders() -> None:
    expected = {
        "tinyuf2-espressif_saola_1_wroom-0.10.2_combined.bin":
            "f63f414562e66e5fa4305815a56eec1d2e8118dae03e18825fa76bd00b0492ca",
        "tinyuf2-espressif_saola_1_wrover-0.10.2_combined.bin":
            "86a7f31c4015a10744a931fc541824975bf95319f21fd64d83aaf5c42a1047e7",
    }
    bootloader_root = GROUND_STATION / "bootloader"
    for filename, expected_digest in expected.items():
        actual_digest = hashlib.sha256((bootloader_root / filename).read_bytes()).hexdigest()
        if actual_digest != expected_digest:
            raise RuntimeError(f"Protected TinyUF2 image changed: {filename}")


def main() -> None:
    platformio = GROUND_STATION / "platformio.ini"
    require_text(
        platformio,
        (
            "platform-espressif32/releases/download/2026.05.50/platform-espressif32.zip",
            "adafruit/Adafruit TinyUSB Library@3.7.7",
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
    forbidden = ("CFG_TUSB_", "CFG_TUD_", "toolchain-xtensa-esp32s2")
    if any(fragment in platformio_text for fragment in forbidden):
        raise RuntimeError("Obsolete TinyUSB or toolchain overrides are present")
    if (GROUND_STATION / "src" / "USBCDC.cpp").exists():
        raise RuntimeError("USBCDC.cpp must come from the pinned Arduino-ESP32 core")

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
        actual_digest = hashlib.sha256((fatfs_root / filename).read_bytes()).hexdigest()
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

#!/usr/bin/env python3
"""Deterministic Ground Station simulator CLI.

The WebAssembly controller contract is mirrored here for headless scenarios.
This small launcher is intentionally dependency-free so `gs-sim.ps1 test` also works
on a clean Windows checkout before Emscripten has been installed.  It is the
headless scenario runner used by the browser fallback and by CI diagnostics.
"""

from __future__ import annotations

import argparse
import base64
import copy
import csv
import hashlib
import json
import math
import os
import re
import struct
import sys
import zlib
from pathlib import Path
from typing import Any

WIDTH = 400
HEIGHT = 240
INTRO_FRAME_MS = 50
ROCKET_FLIGHT_END_MS = 1200
LOGO_DESCENT_START_MS = 1400
LOGO_DESCENT_END_MS = 2850
LOGO_SETTLE_END_MS = 3150
BOOT_MS = 3500
STATIC_LOGO_MS = 2000
MAXIMUM_BOOT_MS = 5000
LONG_PRESS_MS = 500
REPEAT_MS = 100
BUTTONS = ("up", "down", "left", "right", "center", "ok", "back")
MENU_SCREENS = ("live", "recovery", "testing", "data", "sensors", "settings")
SETTING_COUNTS = (4, 4, 3)

assert BOOT_MS <= MAXIMUM_BOOT_MS
assert BOOT_MS % INTRO_FRAME_MS == 0


def startup_phase(elapsed_ms: int) -> str:
    if elapsed_ms < ROCKET_FLIGHT_END_MS:
        return "rocket_flight"
    if elapsed_ms < LOGO_DESCENT_START_MS:
        return "cloud_transition"
    if elapsed_ms < LOGO_DESCENT_END_MS:
        return "logo_descent"
    if elapsed_ms < LOGO_SETTLE_END_MS:
        return "logo_settle"
    if elapsed_ms < BOOT_MS:
        return "logo_hold"
    return "complete"


class ScenarioError(ValueError):
    pass


def fail(message: str) -> None:
    raise ScenarioError(message)


def valid_location(latitude: float, longitude: float) -> bool:
    return (math.isfinite(latitude) and math.isfinite(longitude) and (latitude != 0.0 or longitude != 0.0)
            and -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0)


def google_maps_url(latitude: float, longitude: float) -> str:
    if not valid_location(latitude, longitude):
        return ""
    return f"https://www.google.com/maps/search/?api=1&query={latitude:.4f}%2C{longitude:.4f}"


def log_location(log: dict[str, Any], link_source: int) -> tuple[float, float]:
    last = (0.0, 0.0)
    for row in csv.reader(str(log.get("csv", "")).splitlines()):
        if len(row) < 6 or not row[0].strip().isdigit():
            continue
        try:
            if int(row[0]) != link_source:
                continue
            latitude = int(row[4]) / 10000.0
            longitude = int(row[5]) / 10000.0
        except ValueError:
            continue
        if valid_location(latitude, longitude):
            last = (latitude, longitude)
    return last


class Framebuffer:
    """One-bit logical framebuffer with deterministic 1-bit PNG output."""

    def __init__(self) -> None:
        self.pixels = bytearray(WIDTH * HEIGHT)  # 0 = white, 1 = black
        self.revision = 0

    def clear(self, black: bool = False) -> None:
        self.pixels[:] = bytes([1 if black else 0]) * len(self.pixels)

    def pixel(self, x: int, y: int, black: bool = True) -> None:
        if 0 <= x < WIDTH and 0 <= y < HEIGHT:
            self.pixels[y * WIDTH + x] = 1 if black else 0

    def rect(self, x: int, y: int, w: int, h: int, black: bool = True, filled: bool = True) -> None:
        if filled:
            for yy in range(max(0, y), min(HEIGHT, y + h)):
                start = yy * WIDTH + max(0, x)
                end = yy * WIDTH + min(WIDTH, x + w)
                if end > start:
                    self.pixels[start:end] = bytes([1 if black else 0]) * (end - start)
        else:
            for xx in range(x, x + w):
                self.pixel(xx, y, black)
                self.pixel(xx, y + h - 1, black)
            for yy in range(y, y + h):
                self.pixel(x, yy, black)
                self.pixel(x + w - 1, yy, black)

    def line(self, x0: int, y0: int, x1: int, y1: int, black: bool = True) -> None:
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            self.pixel(x0, y0, black)
            if x0 == x1 and y0 == y1:
                break
            twice = 2 * err
            if twice >= dy:
                err += dy
                x0 += sx
            if twice <= dx:
                err += dx
                y0 += sy

    def text(self, x: int, y: int, value: str, scale: int = 1, black: bool = True) -> None:
        cursor = x
        for char in str(value):
            glyph = FONT.get(char.upper(), FONT["?"])
            for row, bits in enumerate(glyph):
                for col in range(5):
                    if bits & (1 << (4 - col)):
                        for sy in range(scale):
                            for sx in range(scale):
                                self.pixel(cursor + col * scale + sx, y + row * scale + sy, black)
            cursor += 6 * scale

    def present(self) -> None:
        self.revision += 1

    def packed(self) -> bytes:
        result = bytearray((WIDTH * HEIGHT + 7) // 8)
        for index, black in enumerate(self.pixels):
            if not black:  # PNG/API bit 1 means white, matching GFXcanvas1.
                result[index // 8] |= 0x80 >> (index & 7)
        return bytes(result)

    def png(self) -> bytes:
        rows = bytearray()
        for y in range(HEIGHT):
            rows.append(0)
            row = self.pixels[y * WIDTH : (y + 1) * WIDTH]
            packed = bytearray((WIDTH + 7) // 8)
            for x, black in enumerate(row):
                if not black:
                    packed[x // 8] |= 0x80 >> (x & 7)
            rows.extend(packed)

        def chunk(kind: bytes, data: bytes) -> bytes:
            return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

        header = struct.pack(">IIBBBBB", WIDTH, HEIGHT, 1, 0, 0, 0, 0)
        return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(bytes(rows), 9)) + chunk(b"IEND", b"")


# Five-by-seven glyphs.  Keeping labels in the same framebuffer pipeline makes
# PNGs and revision numbers deterministic without a host-font dependency.
FONT = {
    " ": (0, 0, 0, 0, 0, 0, 0),
    "?": (14, 17, 1, 2, 4, 0, 4),
    "-": (0, 0, 0, 31, 0, 0, 0),
    ":": (0, 4, 0, 0, 0, 4, 0),
    "/": (1, 2, 4, 8, 16, 0, 0),
    ".": (0, 0, 0, 0, 0, 6, 6),
    "0": (14, 17, 19, 21, 25, 17, 14),
    "1": (4, 12, 4, 4, 4, 4, 14),
    "2": (14, 17, 1, 2, 4, 8, 31),
    "3": (30, 1, 1, 14, 1, 1, 30),
    "4": (2, 6, 10, 18, 31, 2, 2),
    "5": (31, 16, 16, 30, 1, 1, 30),
    "6": (6, 8, 16, 30, 17, 17, 14),
    "7": (31, 1, 2, 4, 8, 8, 8),
    "8": (14, 17, 17, 14, 17, 17, 14),
    "9": (14, 17, 17, 15, 1, 2, 12),
}
for _char, _bits in zip("ABCDEFGHIJKLMNOPQRSTUVWXYZ", (
    (14, 17, 17, 31, 17, 17, 17), (30, 17, 17, 30, 17, 17, 30),
    (14, 17, 16, 16, 16, 17, 14), (30, 17, 17, 17, 17, 17, 30),
    (31, 16, 16, 30, 16, 16, 31), (31, 16, 16, 30, 16, 16, 16),
    (14, 17, 16, 23, 17, 17, 14), (17, 17, 17, 31, 17, 17, 17),
    (14, 4, 4, 4, 4, 4, 14), (7, 2, 2, 2, 18, 18, 12),
    (17, 18, 20, 24, 20, 18, 17), (16, 16, 16, 16, 16, 16, 31),
    (17, 27, 21, 21, 17, 17, 17), (17, 25, 21, 19, 17, 17, 17),
    (14, 17, 17, 17, 17, 17, 14), (30, 17, 17, 30, 16, 16, 16),
    (14, 17, 17, 17, 21, 18, 13), (30, 17, 17, 30, 20, 18, 17),
    (15, 16, 16, 14, 1, 1, 30), (31, 4, 4, 4, 4, 4, 4),
    (17, 17, 17, 17, 17, 17, 14), (17, 17, 17, 17, 17, 10, 4),
    (17, 17, 17, 21, 21, 21, 10), (17, 17, 10, 4, 10, 17, 17),
    (17, 17, 10, 4, 4, 4, 4), (31, 1, 2, 4, 8, 16, 31),
)):
    FONT[_char] = _bits


def defaults() -> dict[str, Any]:
    return {
        "configuration": {
            "timeZoneOffset": 0, "neverStopLogging": False, "dualReceiver": False,
            "linkPhrase1": "", "linkPhrase2": "", "testingPhrase": "", "imperialUnits": False,
            "startupAnimation": True,
        },
        "links": [
            {"enabled": True, "connected": False, "telemetry": {"state": 2, "errors": 0, "altitudeM": 0,
             "velocityMps": 0, "latitude": 0.0, "longitude": 0.0, "voltage": 0.0, "pyroContinuity": 0,
             "testingMode": False, "timestampDs": 0, "lastUpdateMs": 0, "updated": False},
             "info": {"linkQuality": 0, "rssi": 0, "snr": 0, "lastUpdateMs": 0, "updated": False}},
            {"enabled": True, "connected": False, "telemetry": {"state": 2, "errors": 0, "altitudeM": 0,
             "velocityMps": 0, "latitude": 0.0, "longitude": 0.0, "voltage": 0.0, "pyroContinuity": 0,
             "testingMode": False, "timestampDs": 0, "lastUpdateMs": 0, "updated": False},
             "info": {"linkQuality": 0, "rssi": 0, "snr": 0, "lastUpdateMs": 0, "updated": False}},
        ],
        "navigation": {"homeLatitude": 0.0, "homeLongitude": 0.0, "rocketLatitude": 0.0,
                        "rocketLongitude": 0.0, "northRad": 0.0, "azimuthRad": 0.0, "distanceM": 0.0,
                        "elevationRad": 0.0, "ax": 0.0, "ay": 0.0, "az": 1.0, "gx": 0.0, "gy": 0.0,
                        "gz": 0.0, "mx": 0.0, "my": 0.0, "mz": 0.0, "calibrationPercentage": 0.0,
                        "calibrationState": 0, "updated": False},
        "deviceStatus": {"batteryVoltage": 0.0, "usb": False, "freeStoragePercent": 100, "gnss": False,
                          "clockValid": False, "hour": 0, "minute": 0, "logging": False,
                          "recorderWriteFailure": False, "deleteFailure": False, "finalizeFailure": False,
                          "usbStorageState": "firmware"},
        "logs": [],
    }


def merge(base: Any, patch: Any) -> Any:
    if isinstance(base, dict) and isinstance(patch, dict):
        for key, value in patch.items():
            base[key] = merge(base.get(key), value) if key in base else copy.deepcopy(value)
        return base
    return copy.deepcopy(patch)


def canonical_path(value: str) -> str:
    return value.replace("device_status", "deviceStatus").replace("free_storage_percent", "freeStoragePercent")


class Simulator:
    def __init__(self, initial: dict[str, Any] | None = None, ready: bool = True, base_dir: Path | None = None) -> None:
        self.state = merge(defaults(), initial or {})
        self.state["logs"] = [
            log for log in self.state.get("logs", [])
            if str(log.get("name", "")).lower().endswith(".csv")
        ]
        self.state["logs"].sort(
            key=lambda log: (
                1 if re.fullmatch(r"log_(\d+)\.csv", str(log.get("name", ""))) else 0,
                int(re.fullmatch(r"log_(\d+)\.csv", str(log.get("name", ""))).group(1))
                if re.fullmatch(r"log_(\d+)\.csv", str(log.get("name", ""))) else -1,
            ),
            reverse=True,
        )
        self.base_dir = base_dir or Path.cwd()
        default_links = defaults()["links"]
        supplied_links = self.state.get("links", [])
        self.state["links"] = [
            merge(copy.deepcopy(default_links[index]), supplied_links[index] if index < len(supplied_links) else {})
            for index in range(2)
        ]
        self.frame = Framebuffer()
        self.now_ms = 0
        self.screen = "logo"
        self.testing_state = "disclaimer"
        self.calibration_state = "idle"
        self.settings_state = "list"
        self.menu_selection = 0
        self.settings_page = 0
        self.settings_selection = -1
        self.data_selection = 0
        self.data_statistics = False
        self.data_subview = "list"
        self.usb_storage_session = False
        self.usb_storage_message = ""
        self.usb_previously_connected = False
        self.automatic_usb_share_pending = False
        self.previous_recorder_state = "idle"
        self.log_scroll_offset = 0
        self.qr_view = "none"
        self.qr_url = ""
        self.recovery_locations = [(0.0, 0.0), (0.0, 0.0)]
        self.selected_recovery_link = 0
        self.recorder_state = "idle"
        self.active_filename = ""
        self.recorded_rows = 0
        self.dropped_rows = 0
        self.participant_mask = 0
        self.touchdown_mask = 0
        self.recorder_armed = True
        self.completed_participants = 0
        self.rearm_mask = 0
        self.keyboard_index = 0
        self.held = {name: False for name in BUTTONS}
        self.held_since = {name: 0 for name in BUTTONS}
        self.last_repeat = {name: 0 for name in BUTTONS}
        self.actions: list[dict[str, Any]] = []
        self.replay_rows: list[dict[str, Any]] = []
        self.replay_index = 0
        self.replay_speed = 1.0
        self.replay_start = 0
        self.testing_start = 0
        self.intro_frame_rendered = -1
        if ready:
            self.advance(self.startup_duration_ms())
        else:
            self.render_intro()

    def config(self) -> dict[str, Any]:
        return self.state["configuration"]

    def startup_duration_ms(self) -> int:
        return BOOT_MS if self.config().get("startupAnimation", True) else STATIC_LOGO_MS

    def link(self, index: int) -> dict[str, Any]:
        return self.state["links"][index]

    def emit(self, action: str, link: int = 0, value: int | float | None = None, text: str | None = None) -> None:
        item: dict[str, Any] = {"type": action}
        if link:
            item["link"] = link
        if value is not None:
            item["value"] = value
        if text is not None:
            item["text"] = text
        self.actions.append(item)

    def render(self, title: str, subtitle: str = "") -> None:
        self.frame.clear()
        self.frame.rect(0, 0, WIDTH, 18, True)
        self.frame.text(8, 5, "CATS GS", 1, False)
        self.frame.text(12, 32, title, 2)
        if subtitle:
            self.frame.text(12, 64, subtitle[:60], 1)
        self.frame.rect(8, 95, 384, 125, True, filled=False)
        self.frame.present()

    def draw_intro_cloud(self, x: int, y: int, scale: int) -> None:
        outline = ((0, 11), (6, 11), (7, 7), (10, 4), (14, 3), (18, 6), (19, 8),
                   (23, 8), (25, 5), (29, 4), (33, 6), (35, 10), (40, 11), (0, 11))
        for first, second in zip(outline, outline[1:]):
            self.frame.line(x + first[0] * scale, y + first[1] * scale,
                            x + second[0] * scale, y + second[1] * scale)

    def draw_intro_rocket(self, center_x: int, center_y: int, frame_number: int) -> None:
        def point(forward: int, sideways: int) -> tuple[int, int]:
            return (center_x + (5 * forward + 9 * sideways) // 10,
                    center_y + (-9 * forward + 5 * sideways) // 10)

        tip, left, right = point(28, 0), point(-15, -8), point(-15, 8)
        nose_left, nose_right = point(12, -8), point(12, 8)
        for start, end in ((tip, nose_left), (tip, nose_right), (nose_left, left),
                           (nose_right, right), (left, right)):
            self.frame.line(*start, *end)
        window = point(5, 0)
        self.frame.rect(window[0] - 3, window[1] - 3, 7, 7)
        self.frame.line(*point(-3, -5), *point(-3, 5))
        for start, end in ((point(-11, -7), point(-21, -16)), (point(-21, -16), point(-2, -8)),
                           (point(-11, 7), point(-21, 16)), (point(-21, 16), point(-2, 8))):
            self.frame.line(*start, *end)
        wobble = 2 if frame_number % 2 == 0 else -2
        self.frame.line(*point(-16, 0), *point(-34, wobble))
        self.frame.line(*point(-16, 0), *point(-28, -5 - wobble))
        self.frame.line(*point(-16, 0), *point(-27, 5 + wobble))

    def render_intro(self) -> None:
        if not self.config().get("startupAnimation", True):
            if self.intro_frame_rendered == 0:
                return
            self.intro_frame_rendered = 0
            self.frame.clear()
            self.frame.rect(162, 45, 90, 150, filled=False)
            self.frame.text(171, 109, "CATS", 3)
            self.frame.present()
            return

        elapsed = min(self.now_ms, BOOT_MS)
        elapsed -= elapsed % INTRO_FRAME_MS
        frame_number = elapsed // INTRO_FRAME_MS
        if frame_number == self.intro_frame_rendered:
            return
        self.intro_frame_rendered = frame_number
        phase = startup_phase(elapsed)
        self.frame.clear()

        if phase in ("rocket_flight", "cloud_transition"):
            drift = elapsed // 85
            self.draw_intro_cloud(258 - drift, 35, 2)
            self.draw_intro_cloud(35 - drift // 2, 136, 1)
            if phase == "rocket_flight":
                rocket_x = 20 + 172 * elapsed // ROCKET_FLIGHT_END_MS
                rocket_y = 265 - 310 * elapsed // ROCKET_FLIGHT_END_MS
                self.draw_intro_rocket(rocket_x, rocket_y, frame_number)
            self.draw_intro_cloud(90 - drift, 83, 2)
        else:
            cloud_elapsed = elapsed - LOGO_DESCENT_START_MS
            cloud_duration = LOGO_SETTLE_END_MS - LOGO_DESCENT_START_MS
            cloud_travel = 140 * min(cloud_elapsed, cloud_duration) // cloud_duration
            self.draw_intro_cloud(52 - cloud_travel, 45, 2)
            self.draw_intro_cloud(292 + cloud_travel, 139, 1)
            logo_y = 45
            if phase == "logo_descent":
                progress = elapsed - LOGO_DESCENT_START_MS
                duration = LOGO_DESCENT_END_MS - LOGO_DESCENT_START_MS
                logo_y = -155 + 202 * progress * (2 * duration - progress) // (duration * duration)
            elif phase == "logo_settle":
                progress = elapsed - LOGO_DESCENT_END_MS
                duration = LOGO_SETTLE_END_MS - LOGO_DESCENT_END_MS
                logo_y = 47 - 2 * progress * progress // (duration * duration)
            self.frame.rect(162, logo_y, 90, 150, filled=False)
            self.frame.text(171, logo_y + 64, "CATS", 3)
            if phase == "logo_descent":
                foreground_x = 170 - 220 * cloud_elapsed // cloud_duration
                self.draw_intro_cloud(foreground_x, 82, 2)
        self.frame.present()

    def render_screen(self) -> None:
        if self.screen == "logo":
            self.render_intro()
        elif self.screen == "menu":
            self.render("Main Menu", " ".join(MENU_SCREENS))
            self.frame.text(12, 82, f"Selection: {self.menu_selection}")
        elif self.screen == "live":
            self.render("Live", "Left GNSS  Right downrange  B back")
            for index in range(2):
                link = self.link(index)
                y = 110 + index * 48
                self.frame.text(16, y, f"L{index + 1} {link['telemetry'].get('state', 0)}")
                self.frame.text(130, y, f"AGE {max(0, self.now_ms - link['telemetry'].get('lastUpdateMs', 0))}ms")
        elif self.screen == "recovery":
            if self.qr_view.startswith("recovery_link_"):
                self.render(f"[Link {self.qr_view[-1]}] Last Location", self.qr_url)
            else:
                has_last_location = any(valid_location(*location) for location in self.recovery_locations)
                self.render("Recovery", "Press > for last locations" if has_last_location else "Navigation / downrange")
        elif self.screen == "testing":
            self.render("Testing", self.testing_state)
        elif self.screen == "data":
            if self.qr_view.startswith("log_link_"):
                self.render(f"[Link {self.qr_view[-1]}] Last Location", self.qr_url)
            else:
                logs = self.state.get("logs", [])
                if self.data_statistics and logs:
                    log_name = str(logs[self.data_selection].get("name", ""))
                    title = log_name
                    if len(title) > 32:
                        title = f"{log_name[:29]}..."
                    self.render(title, "Flight statistics")
                else:
                    self.render("Data", "Flight logs")
            self.frame.text(12, 82, f"Selection: {self.data_selection}")
        elif self.screen == "sensors":
            self.render("Sensors", self.calibration_state)
        elif self.screen == "settings":
            title = f"Settings page {self.settings_page}"
            self.render(title, self.settings_state)
            self.frame.text(12, 82, f"Selection: {self.settings_selection}")
        elif self.screen == "bootloader":
            self.render("Bootloader", "Bootloader request emitted")
        elif self.screen == "usb_storage":
            storage_state = self.state["deviceStatus"].get("usbStorageState", "firmware")
            if self.usb_storage_message:
                self.render("USB Drive", self.usb_storage_message)
            elif storage_state == "host":
                self.render("USB Drive", "Logs available on PC. Back (B). Disconnect (A).")
            else:
                self.render("USB Drive", "Preparing USB drive...")

    def press(self, button: str) -> None:
        button = self.normalize_button(button)
        if button not in self.held:
            fail(f"unknown button '{button}'")
        self.held[button] = True
        self.held_since[button] = self.now_ms
        self.last_repeat[button] = self.now_ms
        self.tick({button})

    def release(self, button: str) -> None:
        button = self.normalize_button(button)
        if button not in self.held:
            fail(f"unknown button '{button}'")
        self.held[button] = False
        self.held_since[button] = 0
        self.last_repeat[button] = 0
        self.tick(set())

    def hold(self, button: str, milliseconds: int) -> None:
        if milliseconds < 0:
            fail("hold duration must be non-negative")
        self.press(button)
        self.advance(milliseconds)
        self.release(button)

    def advance(self, milliseconds: int) -> None:
        if milliseconds < 0:
            fail("advance duration must be non-negative")
        target = self.now_ms + milliseconds
        while self.now_ms < target:
            next_tick = min(target, self.now_ms + 20)
            self.now_ms = next_tick
            self.apply_replay()
            repeats = {name for name in BUTTONS if self.held[name] and self.now_ms - self.held_since[name] >= LONG_PRESS_MS
                       and self.now_ms - self.last_repeat[name] >= REPEAT_MS}
            self.tick(repeats)
        if milliseconds == 0:
            self.apply_replay()
            self.tick(set())

    def tick(self, explicit_pressed: set[str]) -> None:
        for index in range(2):
            telemetry = self.link(index)["telemetry"]
            location = (float(telemetry.get("latitude", 0.0)), float(telemetry.get("longitude", 0.0)))
            if valid_location(*location):
                self.recovery_locations[index] = location
        self.ingest_recording()
        self.update_automatic_usb_storage()
        if self.screen == "logo" and self.now_ms >= self.startup_duration_ms():
            self.screen = "menu"
            self.render_screen()
        if self.screen == "menu":
            self.menu_step(explicit_pressed)
        elif self.screen == "live":
            self.live_step(explicit_pressed)
        elif self.screen == "testing":
            self.testing_step(explicit_pressed)
        elif self.screen == "data":
            self.data_step(explicit_pressed)
        elif self.screen == "sensors":
            self.sensors_step(explicit_pressed)
        elif self.screen == "settings":
            self.settings_step(explicit_pressed)
        elif self.screen == "recovery":
            self.recovery_step(explicit_pressed)
        elif self.screen == "bootloader" and "back" in explicit_pressed:
            self.screen = "settings"
        elif self.screen == "usb_storage":
            self.usb_storage_step(explicit_pressed)
        for name in explicit_pressed:
            if self.held.get(name):
                self.last_repeat[name] = self.now_ms
        self.state["deviceStatus"]["logging"] = self.recorder_state == "recording"
        self.render_screen()

    def ingest_recording(self) -> None:
        for index in range(2):
            telemetry = self.link(index)["telemetry"]
            if not telemetry.get("updated", False):
                continue
            telemetry["updated"] = False
            source_bit = 1 << index
            state = int(telemetry.get("state", 0))
            if state > 2 and self.state["deviceStatus"].get("usbStorageState", "firmware") != "firmware":
                self.state["deviceStatus"]["usbStorageState"] = "firmware"
                self.emit("usb_storage_reclaimed_for_logging")
            if not self.recorder_armed:
                if state <= 2:
                    self.rearm_mask |= source_bit
                    if (not self.config().get("dualReceiver", False) or
                            self.completed_participants and
                            self.rearm_mask & self.completed_participants == self.completed_participants):
                        self.recorder_armed = True
                        self.participant_mask = 0
                        self.completed_participants = 0
                        self.recorded_rows = 0
                continue
            if state <= 2:
                continue
            if self.state["deviceStatus"].get("recorderWriteFailure", False):
                self.recorder_state = "fault"
                continue
            logs = self.state.setdefault("logs", [])
            active = next((log for log in logs if log.get("active")), None)
            if active is None:
                numbers = []
                for log in logs:
                    match = re.fullmatch(r"log_(\d+)\.csv", str(log.get("name", "")))
                    if match:
                        numbers.append(int(match.group(1)))
                self.active_filename = f"log_{max(numbers, default=-1) + 1:03d}.csv"
                active = {"name": self.active_filename, "active": True,
                          "csv": "link,ts[deciseconds],state,errors,lat[deg/10000],lon[deg/10000],altitude[m],velocity[m/s],battery[decivolts],pyro1,pyro2\n"}
                logs.insert(0, active)
                self.recorder_state = "recording"
            row = [index + 1, int(telemetry.get("timestampDs", 0)), state, int(telemetry.get("errors", 0)),
                   round(float(telemetry.get("latitude", 0)) * 10000), round(float(telemetry.get("longitude", 0)) * 10000),
                   int(telemetry.get("altitudeM", 0)), int(telemetry.get("velocityMps", 0)),
                   round(float(telemetry.get("voltage", 0)) * 10), int(telemetry.get("pyroContinuity", 0)) & 1,
                   1 if int(telemetry.get("pyroContinuity", 0)) & 2 else 0]
            active["csv"] += ",".join(str(value) for value in row) + "\n"
            active["sizeBytes"] = len(active["csv"])
            self.participant_mask |= source_bit
            self.recorded_rows += 1
            if state == 7:
                self.touchdown_mask |= source_bit
                complete = (not self.config().get("dualReceiver", False) or
                            self.touchdown_mask & self.participant_mask == self.participant_mask)
                if complete and not self.config().get("neverStopLogging", False):
                    self.finalize_recording()

    def update_automatic_usb_storage(self) -> None:
        device = self.state["deviceStatus"]
        connected = bool(device.get("usb", False))
        if not connected:
            self.usb_previously_connected = False
            self.automatic_usb_share_pending = False
            self.previous_recorder_state = self.recorder_state
            return
        if not self.usb_previously_connected:
            self.usb_previously_connected = True
            self.automatic_usb_share_pending = self.recorder_state == "idle"
        if self.previous_recorder_state != "idle" and self.recorder_state == "idle":
            self.automatic_usb_share_pending = True
        if self.recorder_state != "idle":
            self.automatic_usb_share_pending = False
        self.previous_recorder_state = self.recorder_state

        if (self.automatic_usb_share_pending and self.screen != "data" and
                device.get("usbStorageState", "firmware") == "firmware"):
            device["usbStorageState"] = "host"
            self.automatic_usb_share_pending = False
            self.emit("usb_storage_shared_automatically")

    def finalize_recording(self) -> bool:
        if self.recorder_state not in ("recording", "fault") or not self.active_filename:
            return False
        if self.state["deviceStatus"].get("finalizeFailure", False):
            self.recorder_state = "fault"
            return False
        for log in self.state.get("logs", []):
            if log.get("name") == self.active_filename:
                log["active"] = False
        self.completed_participants = self.participant_mask
        self.recorder_state = "idle"
        self.active_filename = ""
        self.recorder_armed = False
        self.touchdown_mask = 0
        self.rearm_mask = 0
        return True

    @staticmethod
    def normalize_button(button: str) -> str:
        aliases = {"a": "ok", "b": "back", "space": "center", "enter": "ok", "escape": "back",
                   "up": "up", "down": "down", "left": "left", "right": "right", "center": "center", "ok": "ok", "back": "back"}
        return aliases.get(str(button).lower(), str(button).lower())

    def to_menu(self) -> None:
        self.screen = "menu"
        self.testing_state = "disclaimer"
        self.calibration_state = "idle"
        self.data_selection = 0
        self.data_statistics = False
        self.data_subview = "list"
        self.log_scroll_offset = 0
        self.qr_view = "none"
        self.qr_url = ""

    def menu_step(self, pressed: set[str]) -> None:
        old = self.menu_selection
        if "right" in pressed and self.menu_selection % 3 < 2:
            self.menu_selection += 1
        if "left" in pressed and self.menu_selection % 3 > 0:
            self.menu_selection -= 1
        if "down" in pressed and self.menu_selection < 3:
            self.menu_selection += 3
        if "up" in pressed and self.menu_selection > 2:
            self.menu_selection -= 3
        if old != self.menu_selection:
            self.emit("menu_selection", value=self.menu_selection)
        if "ok" in pressed or "center" in pressed:
            self.screen = MENU_SCREENS[self.menu_selection]
            if self.screen == "data":
                self.automatic_usb_share_pending = False
                self.state["deviceStatus"]["usbStorageState"] = "firmware"
            self.qr_view = "none"
            self.qr_url = ""
            self.settings_page = 0
            self.settings_selection = -1
            self.settings_state = "list"
            if self.screen == "recovery" and self.config().get("dualReceiver", False):
                self.selected_recovery_link = 0 if valid_location(*self.recovery_locations[0]) else 1 if valid_location(*self.recovery_locations[1]) else 0
            self.render_screen()

    def live_step(self, pressed: set[str]) -> None:
        if "left" in pressed:
            self.emit("live_view", text="gnss")
        if "right" in pressed:
            self.emit("live_view", text="downrange")
        if "back" in pressed:
            self.to_menu()

    def recovery_step(self, pressed: set[str]) -> None:
        if self.config().get("dualReceiver", False):
            if self.qr_view == "none" and ({"up", "down"} & pressed):
                self.selected_recovery_link = 1 - self.selected_recovery_link
            if "right" in pressed:
                if self.qr_view == "none":
                    if valid_location(*self.recovery_locations[self.selected_recovery_link]):
                        self.qr_view = f"recovery_link_{self.selected_recovery_link + 1}"
                else:
                    other = 1 if self.qr_view == "recovery_link_1" else 0
                    if valid_location(*self.recovery_locations[other]):
                        self.selected_recovery_link = other
                        self.qr_view = f"recovery_link_{other + 1}"
            if "left" in pressed and self.qr_view != "none":
                self.qr_view = "none"
            if self.qr_view.startswith("recovery_link_"):
                link_index = int(self.qr_view[-1]) - 1
                self.qr_url = google_maps_url(*self.recovery_locations[link_index])
            else:
                self.qr_url = ""
            if "back" in pressed:
                self.to_menu()
            return
        if "right" in pressed:
            if self.qr_view == "none":
                link_index = 0 if valid_location(*self.recovery_locations[0]) else 1
                if valid_location(*self.recovery_locations[link_index]):
                    self.qr_view = f"recovery_link_{link_index + 1}"
            elif self.qr_view == "recovery_link_1" and valid_location(*self.recovery_locations[1]):
                self.qr_view = "recovery_link_2"
        if "left" in pressed and self.qr_view != "none":
            if self.qr_view == "recovery_link_2" and valid_location(*self.recovery_locations[0]):
                self.qr_view = "recovery_link_1"
            else:
                self.qr_view = "none"
        if self.qr_view.startswith("recovery_link_"):
            link_index = int(self.qr_view[-1]) - 1
            self.qr_url = google_maps_url(*self.recovery_locations[link_index])
        else:
            self.qr_url = ""
        if "back" in pressed:
            self.to_menu()

    def testing_step(self, pressed: set[str]) -> None:
        link1 = self.link(0)
        if self.testing_state == "confirm_event":
            if "ok" in pressed:
                event = self.keyboard_index + 1
                self.emit("event_triggered", 1, event)
                self.testing_state = "started"
            if "back" in pressed:
                self.testing_state = "started"
            return
        if "back" in pressed:
            if self.testing_state in ("waiting", "started", "confirm_event"):
                self.emit("testing_exit", 1)
            self.to_menu()
            return
        if self.testing_state == "disclaimer" and "ok" in pressed:
            if link1.get("connected", False):
                self.testing_state = "can_start"
                self.emit("testing_connection", 1, 1)
            else:
                self.testing_state = "cannot_start"
                self.emit("testing_connection", 1, 0)
        elif self.testing_state == "can_start" and "ok" in pressed:
            self.link(1)["enabled"] = False
            self.link(0)["enabled"] = True
            self.testing_start = self.now_ms
            self.testing_state = "waiting"
            self.emit("link_disabled", 2)
            self.emit("testing_enter", 1)
        elif self.testing_state == "waiting":
            tel = link1["telemetry"]
            if tel.get("testingMode") and tel.get("state") == 1:
                self.testing_state = "started"
                self.emit("testing_started", 1)
            elif self.now_ms - self.testing_start > 10000:
                link1["enabled"] = False
                self.testing_state = "failed"
                self.emit("testing_timeout", 1)
        elif self.testing_state == "started":
            if "up" in pressed and self.keyboard_index % 4 > 0:
                self.keyboard_index -= 1
            if "down" in pressed and self.keyboard_index % 4 < 3:
                self.keyboard_index += 1
            if "left" in pressed and self.keyboard_index > 3:
                self.keyboard_index -= 4
            if "right" in pressed and self.keyboard_index < 4:
                self.keyboard_index += 4
            if "ok" in pressed:
                self.testing_state = "confirm_event"
            if not link1.get("connected", False) or link1["telemetry"].get("state") != 1:
                self.testing_state = "failed"
                self.emit("testing_connection_lost", 1)

    def data_step(self, pressed: set[str]) -> None:
        logs = self.state.get("logs", [])
        maximum = max(0, len(logs) - 1)
        if self.data_subview == "details":
            if "back" in pressed or ("up" in pressed and self.qr_view == "none"):
                self.data_statistics = False
                self.data_subview = "list"
                self.qr_view = "none"
                self.qr_url = ""
                return
            if "down" in pressed and self.qr_view == "none":
                self.data_statistics = False
                self.data_subview = "options"
                return
            if ({"left", "right"} & pressed) and logs:
                log = logs[self.data_selection]
                link1 = log_location(log, 1)
                link2 = log_location(log, 2)
                if "right" in pressed:
                    if self.qr_view == "none":
                        if valid_location(*link1):
                            self.qr_view = "log_link_1"
                            self.qr_url = google_maps_url(*link1)
                        elif valid_location(*link2):
                            self.qr_view = "log_link_2"
                            self.qr_url = google_maps_url(*link2)
                    elif self.qr_view == "log_link_1" and valid_location(*link2):
                        self.qr_view = "log_link_2"
                        self.qr_url = google_maps_url(*link2)
                elif "left" in pressed:
                    if self.qr_view == "log_link_2" and valid_location(*link1):
                        self.qr_view = "log_link_1"
                        self.qr_url = google_maps_url(*link1)
                    elif self.qr_view != "none":
                        self.qr_view = "none"
                        self.qr_url = ""
            return
        if self.data_subview == "options":
            if "back" in pressed or "up" in pressed:
                self.data_statistics = True
                self.data_subview = "details"
            elif "ok" in pressed and logs:
                self.data_subview = "confirm_finalize" if logs[self.data_selection].get("active") else "confirm_delete"
            return
        if self.data_subview in ("confirm_finalize", "confirm_delete"):
            if "back" in pressed:
                self.data_subview = "options"
            elif "ok" in pressed:
                if self.data_subview == "confirm_finalize":
                    success = self.finalize_recording()
                elif self.state["deviceStatus"].get("usbStorageState", "firmware") == "host":
                    self.data_subview = "usb_blocked"
                    return
                elif self.state["deviceStatus"].get("deleteFailure", False):
                    success = False
                else:
                    del logs[self.data_selection]
                    success = True
                self.data_subview = "list" if success else "failure"
                self.data_selection = min(self.data_selection, max(0, len(logs) - 1))
            return
        if self.data_subview in ("failure", "usb_blocked"):
            if "back" in pressed:
                self.data_subview = "options"
            return
        if "down" in pressed:
            self.data_selection = min(maximum, self.data_selection + 1)
            if self.data_selection >= self.log_scroll_offset + 11:
                self.log_scroll_offset = self.data_selection - 10
        if "up" in pressed:
            self.data_selection = max(0, self.data_selection - 1)
            if self.data_selection < self.log_scroll_offset:
                self.log_scroll_offset = self.data_selection
        if "ok" in pressed and logs:
            self.data_statistics = True
            self.data_subview = "details"
            self.qr_view = "none"
            self.qr_url = ""
            self.emit("flight_statistics", value=self.data_selection)
        if "back" in pressed:
            self.automatic_usb_share_pending = bool(self.state["deviceStatus"].get("usb", False))
            self.to_menu()

    def sensors_step(self, pressed: set[str]) -> None:
        if self.calibration_state == "idle":
            if "ok" in pressed:
                self.calibration_state = "prepare"
            if "back" in pressed:
                self.to_menu()
        elif self.calibration_state == "prepare":
            if "ok" in pressed:
                self.calibration_state = "calibrating"
                self.state["navigation"]["calibrationState"] = 1
                self.emit("calibration_started")
            if "back" in pressed:
                self.calibration_state = "idle"
        elif self.calibration_state == "calibrating":
            if "back" in pressed:
                self.calibration_state = "idle"
                self.state["navigation"]["calibrationState"] = 2
                self.emit("calibration_cancelled")
            elif self.state["navigation"].get("calibrationState") == 3 or self.state["navigation"].get("calibrationPercentage", 0) >= 100:
                self.calibration_state = "concluded"
                self.emit("calibration_completed")
        elif self.calibration_state == "concluded" and ({"ok", "back"} & pressed):
            self.calibration_state = "idle"
            self.state["deviceStatus"]["usbStorageState"] = "firmware"
            self.automatic_usb_share_pending = bool(self.state["deviceStatus"].get("usb", False))
            self.emit("configuration_saved")

    def settings_step(self, pressed: set[str]) -> None:
        if self.settings_state == "keyboard":
            if "right" in pressed:
                self.keyboard_index = min(37, self.keyboard_index + 1)
            if "left" in pressed:
                self.keyboard_index = max(0, self.keyboard_index - 1)
            if "ok" in pressed:
                alphabet = "1234567890QWERTYUIOPASDFGHJKL ZXCVBNM_"
                config = self.config()
                key = ("linkPhrase1", "linkPhrase2", "testingPhrase")[max(0, min(2, self.settings_selection - 1))]
                config[key] = (config.get(key, "") + alphabet[self.keyboard_index])[:16]
            if "back" in pressed:
                self.settings_state = "list"
                self.emit("settings_string_done")
            return

        if self.settings_selection < 0:
            if "right" in pressed:
                self.settings_page = min(2, self.settings_page + 1)
            if "left" in pressed:
                self.settings_page = max(0, self.settings_page - 1)
        else:
            config = self.config()
            inc = "right" in pressed
            dec = "left" in pressed
            if self.settings_page == 0 and self.settings_selection == 0:
                if inc: config["neverStopLogging"] = True
                if dec: config["neverStopLogging"] = False
            elif self.settings_page == 0 and self.settings_selection == 2 and "ok" in pressed:
                self.usb_storage_message = ""
                device = self.state["deviceStatus"]
                if not device.get("usb", False):
                    self.usb_storage_message = "Connect USB cable first."
                elif self.recorder_state != "idle":
                    self.usb_storage_message = "Finalize the active log first."
                elif device.get("usbStorageState", "firmware") in ("host", "preparing"):
                    self.usb_storage_session = True
                elif device.get("usbStorageState", "firmware") != "firmware":
                    self.usb_storage_message = "USB storage is unavailable."
                else:
                    device["usbStorageState"] = "host"
                    self.usb_storage_session = True
                    self.emit("usb_storage_shared")
                self.screen = "usb_storage"
                return
            elif self.settings_page == 0 and self.settings_selection == 3 and "ok" in pressed:
                self.screen = "bootloader"
                self.emit("bootloader_requested")
            elif self.settings_page == 1 and self.settings_selection == 0:
                if inc: config["dualReceiver"] = True
                if dec: config["dualReceiver"] = False
            elif self.settings_page == 1 and 1 <= self.settings_selection <= 3 and "ok" in pressed:
                self.settings_state = "keyboard"
                self.keyboard_index = 0
            elif self.settings_page == 2 and self.settings_selection == 0:
                config["timeZoneOffset"] = max(-12, min(12, config.get("timeZoneOffset", 0) + (1 if inc else -1 if dec else 0)))
            elif self.settings_page == 2 and self.settings_selection == 1 and (inc or dec):
                config["imperialUnits"] = not config.get("imperialUnits", False)
            elif self.settings_page == 2 and self.settings_selection == 2:
                if inc: config["startupAnimation"] = True
                if dec: config["startupAnimation"] = False

        if "down" in pressed:
            self.settings_selection = min(SETTING_COUNTS[self.settings_page] - 1, self.settings_selection + 1)
        if "up" in pressed:
            self.settings_selection = max(-1, self.settings_selection - 1)
        if "back" in pressed:
            if self.settings_selection >= 0:
                self.settings_selection = -1
            else:
                self.state["deviceStatus"]["usbStorageState"] = "firmware"
                self.automatic_usb_share_pending = bool(self.state["deviceStatus"].get("usb", False))
                self.emit("configuration_saved")
                self.to_menu()

    def usb_storage_step(self, pressed: set[str]) -> None:
        storage_state = self.state["deviceStatus"].get("usbStorageState", "firmware")
        if self.usb_storage_session and storage_state == "firmware":
            self.usb_storage_session = False
            self.emit("usb_storage_reclaimed")
            self.screen = "settings"
            self.settings_page = 0
            self.settings_selection = 2
        elif self.usb_storage_session and storage_state == "fault":
            self.usb_storage_session = False
            self.usb_storage_message = "Storage could not be remounted."
        elif self.usb_storage_session and storage_state == "host" and "back" in pressed:
            self.usb_storage_session = False
            self.screen = "settings"
            self.settings_page = 0
            self.settings_selection = 2
        elif self.usb_storage_session and ((storage_state == "host" and "ok" in pressed) or
                                           (storage_state == "preparing" and "back" in pressed)):
            self.state["deviceStatus"]["usbStorageState"] = "firmware"
        elif not self.usb_storage_session and "back" in pressed:
            self.screen = "settings"
            self.settings_page = 0
            self.settings_selection = 2

    def set_value(self, path: str, value: Any) -> None:
        path = canonical_path(path)
        aliases = {"config": "configuration", "device": "deviceStatus", "nav": "navigation"}
        parts = path.split(".")
        if parts[0] in aliases:
            parts[0] = aliases[parts[0]]
        target: Any = self.state
        for part in parts[:-1]:
            if part.startswith("link") and part[4:].isdigit():
                target = self.link(int(part[4:]) - 1)
            elif part.isdigit():
                target = target[int(part)]
            else:
                if part not in target:
                    fail(f"set path '{path}' has no field '{part}'")
                target = target[part]
        key = parts[-1]
        if isinstance(target, list) and key.isdigit():
            target[int(key)] = value
        elif isinstance(target, dict) and key in target:
            target[key] = value
        else:
            fail(f"set path '{path}' has no field '{key}'")
        if path in ("deviceStatus.usb", "device.usb") and not bool(value):
            self.state["deviceStatus"]["usbStorageState"] = "firmware"
        if path.startswith("links") or path.startswith("link"):
            # Injected peripheral values are observable updates, like a radio commit.
            link_index = None
            if path.startswith("link") and path[4:5].isdigit():
                link_index = int(path[4]) - 1
            elif path.startswith("links.") and path[6:7].isdigit():
                link_index = int(path[6])
            if link_index in (0, 1):
                self.link(link_index)["telemetry"]["updated"] = True
                self.link(link_index)["info"]["updated"] = True

    def set_initial(self, values: dict[str, Any]) -> None:
        if not isinstance(values, dict):
            fail("set step must be an object")
        if any(isinstance(key, str) and "." in key for key in values):
            for key, value in values.items():
                if "." in key:
                    self.set_value(key, value)
                else:
                    self.set_value(key, value)
        else:
            merge(self.state, values)
        default_links = defaults()["links"]
        supplied_links = self.state.get("links", [])
        self.state["links"] = [
            merge(copy.deepcopy(default_links[index]), supplied_links[index] if index < len(supplied_links) else {})
            for index in range(2)
        ]
        self.render_screen()

    def replay(self, spec: Any) -> None:
        if isinstance(spec, str):
            path = spec
            speed = 1.0
        elif isinstance(spec, dict):
            path = spec.get("file", spec.get("path"))
            speed = float(spec.get("speed", 1.0))
        else:
            fail("replay step must be a CSV path or object")
        if not path:
            fail("replay step is missing file")
        if speed <= 0:
            fail("replay speed must be positive")
        csv_path = Path(path)
        if not csv_path.is_absolute():
            csv_path = self.base_dir / csv_path
        try:
            raw_lines = csv_path.read_text(encoding="utf-8").splitlines()
        except OSError as error:
            fail(f"cannot read replay '{csv_path}': {error}")
        rows: list[dict[str, Any]] = []
        for line_number, line in enumerate(raw_lines, 1):
            if not line.strip() or line.lower().startswith("link,"):
                continue
            fields = next(csv.reader([line]))
            if len(fields) < 9:
                fail(f"replay {csv_path}:{line_number}: expected at least 9 fields, got {len(fields)}")
            try:
                values = [int(item.strip()) for item in fields[:9]]
            except ValueError as error:
                fail(f"replay {csv_path}:{line_number}: non-integer field: {error}")
            link, timestamp, state, errors, lat, lon, altitude, velocity, voltage = values
            if link not in (1, 2):
                fail(f"replay {csv_path}:{line_number}: link must be 1 or 2")
            try:
                pyro = (int(fields[9]) if len(fields) > 9 else 0) & 1
                if len(fields) > 10 and int(fields[10]):
                    pyro |= 2
            except ValueError as error:
                fail(f"replay {csv_path}:{line_number}: invalid continuity field: {error}")
            row = {"link": link, "ts": timestamp, "state": state, "errors": errors, "lat": lat, "lon": lon,
                   "altitude": altitude, "velocity": velocity, "voltage": voltage, "pyro": pyro, "line": line_number}
            if len(fields) >= 14:
                try:
                    row["lq"], row["rssi"], row["snr"] = int(fields[11]), int(fields[12]), int(fields[13])
                except (ValueError, IndexError) as error:
                    fail(f"replay {csv_path}:{line_number}: invalid metric field: {error}")
            else:
                row["lq"], row["rssi"], row["snr"] = 100, -50, 10
            rows.append(row)
        rows.sort(key=lambda item: (item["ts"], item["line"]))
        self.replay_rows = rows
        self.replay_index = 0
        self.replay_speed = speed
        self.replay_start = self.now_ms
        self.apply_replay()

    def apply_replay(self) -> None:
        while self.replay_index < len(self.replay_rows):
            row = self.replay_rows[self.replay_index]
            if self.now_ms < self.replay_start + int((row["ts"] - self.replay_rows[0]["ts"]) * 100 / self.replay_speed):
                break
            link = self.link(row["link"] - 1)
            tel = link["telemetry"]
            tel.update({"state": row["state"], "errors": row["errors"], "latitude": row["lat"] / 10000.0,
                        "longitude": row["lon"] / 10000.0, "altitudeM": row["altitude"], "velocityMps": row["velocity"],
                        "voltage": row["voltage"] / 10.0, "pyroContinuity": row["pyro"], "timestampDs": row["ts"],
                        "lastUpdateMs": self.now_ms, "updated": True})
            info = link["info"]
            info.update({"linkQuality": row["lq"], "rssi": row["rssi"], "snr": row["snr"], "lastUpdateMs": self.now_ms, "updated": True})
            link["connected"] = True
            self.replay_index += 1

    def snapshot(self) -> dict[str, Any]:
        logs = self.state.get("logs", [])
        selected_health = "none"
        if logs and self.data_subview != "list":
            selected_health = "active" if logs[self.data_selection].get("active") else "complete"
        selected_link = self.selected_recovery_link if self.config().get("dualReceiver", False) else -1
        target = (self.recovery_locations[selected_link] if selected_link >= 0 else
                  (float(self.state["navigation"].get("rocketLatitude", 0.0)),
                   float(self.state["navigation"].get("rocketLongitude", 0.0))))
        home = (float(self.state["navigation"].get("homeLatitude", 0.0)),
                float(self.state["navigation"].get("homeLongitude", 0.0)))
        recovery_valid = valid_location(*target) and valid_location(*home)
        distance = 0.0
        azimuth = 0.0
        if recovery_valid:
            dy = math.radians(target[0] - home[0]) * 6378100.0
            dx = math.radians(target[1] - home[1]) * math.cos(math.radians(home[0])) * 6378100.0
            distance = math.sqrt(dx * dx + dy * dy)
            azimuth = math.atan2(dx, dy)
        return {
            "activeScreen": self.screen,
            "testingState": self.testing_state,
            "calibrationState": self.calibration_state,
            "settingsState": self.settings_state,
            "inputState": "held" if any(self.held.values()) else "idle",
            "menuSelection": self.menu_selection,
            "settingsPage": self.settings_page,
            "settingsSelection": self.settings_selection,
            "dataSelection": self.data_selection,
            "dataStatistics": self.data_statistics,
            "currentDataSubview": self.data_subview,
            "logCount": len(logs),
            "logScrollOffset": self.log_scroll_offset,
            "selectedLogHealth": selected_health,
            "recorderState": self.recorder_state,
            "activeFilename": self.active_filename,
            "recordedRowCount": self.recorded_rows,
            "droppedRowCount": self.dropped_rows,
            "usbStorageState": self.state["deviceStatus"].get("usbStorageState", "firmware"),
            "usbStorageMessage": self.usb_storage_message,
            "selectedRecoveryLink": selected_link,
            "recoverySolution": {"valid": recovery_valid, "latitude": target[0], "longitude": target[1],
                                 "distanceM": distance, "azimuthRad": azimuth},
            "qrView": self.qr_view,
            "qrUrl": self.qr_url,
            "recoveryLocations": [
                {"latitude": location[0], "longitude": location[1]} for location in self.recovery_locations
            ],
            "virtualTimeMs": self.now_ms,
            "startupElapsedMs": min(self.now_ms, self.startup_duration_ms()),
            "startupPhase": ("static_logo" if not self.config().get("startupAnimation", True)
                             else startup_phase(self.now_ms)) if self.screen == "logo" else "complete",
            "configuration": copy.deepcopy(self.config()),
            "links": copy.deepcopy(self.state["links"]),
            "navigation": copy.deepcopy(self.state["navigation"]),
            "deviceStatus": copy.deepcopy(self.state["deviceStatus"]),
            "actions": copy.deepcopy(self.actions),
            "framebufferRevision": self.frame.revision,
            "framebufferBytes": base64.b64encode(self.frame.packed()).decode("ascii"),
            "framebufferSha256": hashlib.sha256(self.frame.packed()).hexdigest(),
        }

    def assert_value(self, assertion: dict[str, Any]) -> None:
        if not isinstance(assertion, dict):
            fail("assert step must be an object")
        snap = self.snapshot()
        for key, expected in assertion.items():
            if key in ("action", "emittedAction", "emittedActions"):
                names = {item.get("type") for item in snap["actions"]}
                wanted = expected if isinstance(expected, list) else [expected]
                missing = [name for name in wanted if name not in names]
                if missing:
                    fail(f"assert actions missing {missing}; emitted {sorted(names)}")
                continue
            actual: Any = snap
            for part in str(key).replace(".", "/").split("/"):
                if isinstance(actual, dict) and part in actual:
                    actual = actual[part]
                elif isinstance(actual, list) and part.isdigit() and int(part) < len(actual):
                    actual = actual[int(part)]
                else:
                    fail(f"assert field '{key}' does not exist")
            if actual != expected:
                fail(f"assertion failed for '{key}': expected {expected!r}, got {actual!r}")

    def run(self, steps: list[Any]) -> list[dict[str, Any]]:
        snapshots: list[dict[str, Any]] = []
        if not isinstance(steps, list):
            fail("scenario steps must be an array")
        for index, step in enumerate(steps, 1):
            if not isinstance(step, dict) or len(step) != 1:
                fail(f"step {index}: expected an object with one operation")
            operation, value = next(iter(step.items()))
            try:
                if operation == "set":
                    self.set_initial(value)
                elif operation == "press":
                    self.press(value)
                elif operation == "release":
                    self.release(value)
                elif operation == "hold":
                    if not isinstance(value, dict): fail(f"step {index}: hold expects object")
                    self.hold(value.get("button"), int(value.get("ms", value.get("durationMs", 0))))
                elif operation == "advance":
                    self.advance(int(value))
                elif operation == "replay":
                    self.replay(value)
                elif operation == "assert":
                    self.assert_value(value)
                elif operation == "snapshot":
                    snapshot = self.snapshot()
                    snapshot["snapshotName"] = str(value)
                    snapshots.append(snapshot)
                else:
                    fail(f"step {index}: unknown operation '{operation}'")
            except ScenarioError:
                raise
            except Exception as error:
                fail(f"step {index} ({operation}): {error}")
        return snapshots


def run_scenario(path: Path, write_snapshots: bool = False) -> dict[str, Any]:
    try:
        scenario = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        fail(f"scenario {path}:{error.lineno}:{error.colno}: invalid JSON: {error.msg}")
    except OSError as error:
        fail(f"cannot read scenario '{path}': {error}")
    if not isinstance(scenario, dict):
        fail(f"scenario {path}: root must be an object")
    initial = scenario.get("initial", {})
    if not isinstance(initial, dict):
        fail("initial must be an object")
    ready = bool(initial.pop("ready", scenario.get("ready", True)))
    sim = Simulator(initial, ready=ready, base_dir=path.parent)
    snapshots = sim.run(scenario.get("steps", []))
    result = {"scenario": str(path), "snapshot": sim.snapshot(), "snapshots": snapshots, "ok": True}
    if write_snapshots:
        output = path.with_suffix(".actual.json")
        output.write_text(json.dumps(result, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        output.with_suffix(".png").write_bytes(sim.frame.png())
    return result


def deterministic_test(root: Path) -> int:
    cases = []
    scenario_dir = root / "ground_station" / "simulator" / "scenarios"
    if not scenario_dir.exists():
        scenario_dir = root / "simulator" / "scenarios"
    golden_path = scenario_dir.parent / "golden" / "snapshots.json"
    try:
        goldens = json.loads(golden_path.read_text(encoding="utf-8")) if golden_path.exists() else {}
    except (OSError, json.JSONDecodeError) as error:
        print(f"cannot load golden snapshot manifest {golden_path}: {error}", file=sys.stderr)
        return 1
    for scenario_name in ("startup-intro.json", "startup-static.json", "menu.json", "testing-timeout.json",
                          "settings.json", "replay.json",
                          "qr-recovery.json", "qr-data.json", "qr-no-fix.json", "qr-zero-coordinate.json",
                          "recording-independent.json", "recording-modes.json", "log-management.json",
                          "legacy-log-names.json", "usb-storage.json"):
        scenario_path = scenario_dir / scenario_name
        if scenario_path.exists():
            try:
                cases.append((scenario_name, json.loads(scenario_path.read_text(encoding="utf-8"))))
            except (OSError, json.JSONDecodeError) as error:
                print(f"cannot load simulator fixture {scenario_path}: {error}", file=sys.stderr)
                return 1

    for name, case in cases:
        first_initial = copy.deepcopy(case["initial"])
        first_ready = bool(first_initial.pop("ready", case.get("ready", True)))
        first = Simulator(first_initial, ready=first_ready, base_dir=scenario_dir)
        first_snapshots = first.run(copy.deepcopy(case["steps"]))
        second_initial = copy.deepcopy(case["initial"])
        second_ready = bool(second_initial.pop("ready", case.get("ready", True)))
        second = Simulator(second_initial, ready=second_ready, base_dir=scenario_dir)
        second_snapshots = second.run(copy.deepcopy(case["steps"]))
        a, b = first.snapshot(), second.snapshot()
        if (json.dumps(a, sort_keys=True) != json.dumps(b, sort_keys=True)
                or json.dumps(first_snapshots, sort_keys=True) != json.dumps(second_snapshots, sort_keys=True)
                or first.frame.png() != second.frame.png()):
            print(f"determinism test failed: {name}", file=sys.stderr)
            return 1
        expected_hash = goldens.get(name)
        if isinstance(expected_hash, str) and a["framebufferSha256"] != expected_hash:
            print(f"golden framebuffer failed: {name}: expected {expected_hash}, got {a['framebufferSha256']}",
                  file=sys.stderr)
            return 1
        if isinstance(expected_hash, dict):
            expected_final = expected_hash.get("final")
            if expected_final is not None and a["framebufferSha256"] != expected_final:
                print(f"golden framebuffer failed: {name}: expected {expected_final}, got {a['framebufferSha256']}",
                      file=sys.stderr)
                return 1
            actual_snapshots = {item["snapshotName"]: item["framebufferSha256"] for item in first_snapshots}
            for snapshot_name, snapshot_hash in expected_hash.get("snapshots", {}).items():
                if actual_snapshots.get(snapshot_name) != snapshot_hash:
                    print(f"golden snapshot failed: {name}/{snapshot_name}: expected {snapshot_hash}, "
                          f"got {actual_snapshots.get(snapshot_name)}", file=sys.stderr)
                    return 1
    print(f"gs-sim: {len(cases)} deterministic controller/scenario tests passed")
    print(f"framebuffer: {len(first.frame.packed())} bytes, sha256={a['framebufferSha256']}")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="gs-sim")
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run")
    run.add_argument("scenario", type=Path)
    run.add_argument("--write-snapshots", action="store_true")
    test = sub.add_parser("test")
    test.add_argument("--root", type=Path, default=Path.cwd())
    args = parser.parse_args(argv)
    try:
        if args.command == "test":
            return deterministic_test(args.root)
        result = run_scenario(args.scenario.resolve(), args.write_snapshots)
        print(json.dumps(result, sort_keys=True, indent=2))
        return 0
    except ScenarioError as error:
        print(f"gs-sim: error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

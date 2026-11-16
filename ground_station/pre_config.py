# Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
#
# SPDX-License-Identifier: GPL-3.0-or-later

import os
Import("env")

# pioarduino's unified Xtensa archive contains the binaries one directory
# below the PlatformIO package root. A shared PlatformIO home can retain the
# package metadata while losing the installer-added PATH adjustment, leaving
# an otherwise valid compiler undiscoverable on Windows.
platform = env.PioPlatform()
toolchain_root = platform.get_package_dir("toolchain-xtensa-esp-elf")
if toolchain_root:
    nested_toolchain_root = os.path.join(toolchain_root, "xtensa-esp-elf")
    nested_toolchain_bin = os.path.join(nested_toolchain_root, "bin")
    compiler_name = "xtensa-esp32s2-elf-g++.exe" if os.name == "nt" else "xtensa-esp32s2-elf-g++"
    if os.path.isfile(os.path.join(nested_toolchain_bin, compiler_name)):
        env.PrependENVPath("PATH", nested_toolchain_bin)
        toolchain_root = nested_toolchain_root

# Keep source paths in diagnostics and debug information independent of the build machine.
project_root = env.subst("$PROJECT_DIR").replace(os.sep, "/")
core_root = platform.config.get("platformio", "core_dir").replace(os.sep, "/")
prefix_map_flags = [
    f"-ffile-prefix-map={project_root}=.",
    f"-ffile-prefix-map={core_root}=platformio",
]
if toolchain_root:
    # Use the same DWARF paths for flat and nested toolchain installations.
    compiler_root = toolchain_root.replace(os.sep, "/")
    prefix_map_flags.append(f"-fdebug-prefix-map={compiler_root}=platformio/packages/toolchain-xtensa-esp-elf")
env.Append(CCFLAGS=prefix_map_flags, LINKFLAGS=prefix_map_flags)

env.Append(
    # Unfortunately we can't use the standard standard,
    # we have to use the GNU standard as the arduino library requires it..
    CFLAGS=["-std=gnu17"],
    CCFLAGS=[
        #TODO: check if thumb should be enabled
        # cpu
        # "-mthumb",
        # "-mthumb-interwork",

        "-ffunction-sections",
        "-fdata-sections",
        "-fno-common",
        "-fmessage-length=0",
        "-fdiagnostics-color=always",
        "-fstack-usage",

        "-Wno-packed-bitfield-compat",
        "-Wno-attributes"
    ],
    CXXFLAGS=[
        "-std=gnu++20",
        # "-frtti",
        # Disable volatile warnings of type "compound assignment with 'volatile'-qualified left operand is deprecated [-Wvolatile]"
        # This is heavily used by STM libraries and creates too much noise when compiling
        # Eventually this flag should be set only for library files
        "-Wno-volatile"
        ],
    LINKFLAGS=[
        # cpu
        # "-mthumb",
        # "-mthumb-interwork",

        "-Wl,-gc-sections,--print-memory-usage"
    ]
)

# Configure src files
def src_file_config(env, node):
    """
    `node.name` - a name of File System Node
    `node.get_path()` - a relative path
    `node.get_abspath()` - an absolute path
    """

    if 'lib' in node.get_path() or 'FrameworkArduino' in node.get_path():
        return node
    else:
        # print(f'Modifying flags for {node.name} ({node.get_path()})')
        return env.Object(
        node,
        CCFLAGS=env["CCFLAGS"] + [
            "-Wall",
            "-Wimplicit-fallthrough",
            # "-Wshadow",
            "-Wdouble-promotion",
            # "-Wundef",
            "-Wformat=2",
            "-Wformat-truncation=2",
            "-Wformat-overflow",
            "-Wformat-signedness",
            "-Wattributes",
            "-Wextra",

            "-Werror",
            ]
        )

env.AddBuildMiddleware(src_file_config)

# override compilation DB path
env.Replace(COMPILATIONDB_PATH=os.path.join(
    "$BUILD_DIR", "compile_commands.json"))

print(env["CFLAGS"])
print(env["CCFLAGS"])
print(env["CXXFLAGS"])

# Dump build environment (for debug)
# print(env.Dump())

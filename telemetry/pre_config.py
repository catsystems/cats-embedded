# Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
#
# SPDX-License-Identifier: GPL-3.0-or-later

import os
Import("env")

env.Append(
    CFLAGS=["-std=c17"],
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

        "-Wall",
        "-Wimplicit-fallthrough",
        "-Wshadow",
        "-Wdouble-promotion",
        "-Wundef",
        "-Wformat=2",
        "-Wformat-truncation=2",
        "-Wformat-overflow",
        "-Wformat-signedness",

        "-Werror",

        "-Wno-packed-bitfield-compat"
    ],
    CXXFLAGS=[
        "-std=c++20",
        "-frtti",
        # Disable volatile warnings of type "compound assignment with 'volatile'-qualified left operand is deprecated [-Wvolatile]"
        # This is heavily used by STM libraries and creates too much noise when compiling
        # Eventually this flag should be set only for library files
        "-Wno-volatile"],
    LINKFLAGS=[
        # cpu
        # "-mthumb",
        # "-mthumb-interwork",

        "-Wl,-gc-sections,--print-memory-usage"
    ]
)

# include toolchain paths
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)
# override compilation DB path
env.Replace(COMPILATIONDB_PATH=os.path.join(
    "$BUILD_DIR", "compile_commands.json"))

print(env["CFLAGS"])
print(env["CCFLAGS"])
print(env["CXXFLAGS"])

# Dump build environment (for debug)
# print(env.Dump())

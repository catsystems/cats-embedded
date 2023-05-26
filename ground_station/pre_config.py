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

        "-Wno-packed-bitfield-compat",
        "-Wno-attributes"
    ],
    CXXFLAGS=[
        "-std=c++20",
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

            "-Werror",
            ]
        )

env.AddBuildMiddleware(src_file_config)

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

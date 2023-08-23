Import("projenv")

include_flags = []
for path in projenv["CPPPATH"]:
    if path != projenv["PROJECT_INCLUDE_DIR"]:
        include_flags.append(["-isystem", path])

projenv.Append(
    CXXFLAGS = include_flags
)

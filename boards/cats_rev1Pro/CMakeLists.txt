#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.16)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(cats_rev1Pro C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

if (CMAKE_BUILD_TYPE)
    define_property(
            SOURCE
            PROPERTY COMPILE_FLAGS
            INHERITED
            BRIEF_DOCS "brief-doc"
            FULL_DOCS "full-doc"
    )
    string(TOUPPER ${CMAKE_BUILD_TYPE} _build_type)
    set_directory_properties(PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS_${_build_type}}")
    set(CMAKE_CXX_FLAGS_${_build_type} "")
endif ()

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
link_directories(lib/CMSIS/DSP/lib/)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0 -fdiagnostics-color=always -fstack-usage
        -Wall -Wimplicit-fallthrough -Wshadow -Wdouble-promotion -Wundef -Wformat=2 -Wformat-truncation=2 -Wformat-overflow)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(
        lib/STM/STM32L4xx_HAL_Driver/Inc
        lib/STM/STM32L4xx_HAL_Driver/Inc/Legacy
        lib/STM/USB/STM32_USB_Device_Library/Core/Inc
        lib/STM/USB/STM32_USB_Device_Library/Class/CDC/Inc
        lib/STM/USB/USB_DEVICE/App
        lib/STM/USB/USB_DEVICE/Target
        lib/STM/EEPROM
        lib/CMSIS/Device/ST/STM32L4xx/Include
        lib/CMSIS/Include
        lib/CMSIS/DSP/Inc
        lib/FreeRTOS/Source/include
        lib/FreeRTOS/Source/CMSIS_RTOS_V2
        lib/FreeRTOS/Source/portable/GCC/ARM_CM4F
        lib/Tracing/inc
        lib/Tracing/cfg
        lib/LittleFS
        src/)

add_definitions(-DUSE_HAL_DRIVER -DDEBUG -DSTM32L433xx)

file(GLOB_RECURSE LIB_FILES "lib/*.*")
set_source_files_properties(
        ${LIB_FILES}
        PROPERTIES
        COMPILE_FLAGS -O3
)


file(GLOB_RECURSE SOURCES "lib/*.*" "src/*.*")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32L433RCTX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

target_link_libraries(${PROJECT_NAME}.elf libarm_cortexM4lf_math.a)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")

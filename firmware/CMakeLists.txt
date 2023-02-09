#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.18)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATS_TARGET VEGA)

if ("${CATS_TARGET}" STREQUAL "VEGA")
    set(MCU_TYPE STM32F411xE)
    set(MCU_FAMILY STM32F4)
    set(FLASH_LD STM32F411CCUX_FLASH.ld)
endif ()

# project settings
project(${CATS_TARGET} C CXX ASM)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 17)

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
link_directories(lib/CMSIS/DSP/lib)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0 -fdiagnostics-color=always -fstack-usage
        -Wall -Wimplicit-fallthrough -Wshadow -Wdouble-promotion -Wundef -Wformat=2 -Wformat-truncation=2 -Wformat-overflow -Wformat-signedness -Werror)

# Disable volatile warnings of type "compound assignment with 'volatile'-qualified left operand is deprecated [-Wvolatile]"
# This is heavily used by STM libraries and creates too much noise when compiling
# Eventually this flag should be set only for library files
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-volatile")

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
    add_definitions(-DCATS_DEBUG)
endif ()

include_directories(
        lib/STM/${MCU_FAMILY}/${MCU_FAMILY}xx_HAL_Driver/Inc
        lib/STM/${MCU_FAMILY}/${MCU_FAMILY}xx_HAL_Driver/Inc/Legacy
        lib/STM/${MCU_FAMILY}/${MCU_FAMILY}xx/Include
        lib/STM/${MCU_FAMILY}/USB_DEVICE
        lib/STM/USB/STM32_USB_Device_Library/Core/Inc
        lib/STM/USB/STM32_USB_Device_Library/Class/CDC/Inc
        lib/CMSIS/Include
        lib/CMSIS/DSP/Inc
        lib/FreeRTOS/Source/include
        lib/FreeRTOS/Source/CMSIS_RTOS_V2
        lib/FreeRTOS/Source/portable/GCC/ARM_CM4F
        lib/Tracing/inc
        lib/Tracing/cfg
        lib/LittleFS
        target/${CATS_TARGET}
        src
        src/drivers/usb)

add_definitions(-DUSE_HAL_DRIVER -D${MCU_TYPE} -DCATS_${CATS_TARGET})

file(GLOB_RECURSE LIB_FILES
        "lib/STM/${MCU_FAMILY}/*.*"
        "lib/STM/USB/*.*"
        "lib/CMSIS/*.*"
        "lib/FreeRTOS/*.*"
        "lib/LittleFS/*.*"
        "lib/Tracing/*.*")

set_source_files_properties(
        ${LIB_FILES}
        PROPERTIES
        COMPILE_FLAGS -O3
)

file(GLOB_RECURSE TARGET_FILES "target/*.*")
set_source_files_properties(
        ${TARGET_FILES}
        PROPERTIES
        COMPILE_FLAGS -O3
)

file(GLOB_RECURSE SOURCES
        "lib/STM/USB/*.*"
        "lib/STM/${MCU_FAMILY}/*.*"
        "lib/CMSIS/*.*"
        "lib/FreeRTOS/*.*"
        "lib/LittleFS/*.*"
        "target/${CATS_TARGET}/*.*"
        "src/*.*")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/target/${CATS_TARGET}/${FLASH_LD})

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${CATS_TARGET}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

target_link_libraries(${CATS_TARGET}.elf libarm_cortexM4lf_math.a)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${CATS_TARGET}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CATS_TARGET}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${CATS_TARGET}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${CATS_TARGET}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")

cmake_minimum_required(VERSION 3.20)
project(gs_simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(gs-sim-wasm
  wasm_api.cpp
  window_hmi_renderer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/Adafruit_GFX_Library/Adafruit_GFX.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/NayukiQRCode/src/qrcodegen.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/hmi_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/hmi/location_qr.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/hmi/settings.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/hmi/window.cpp)
set_target_properties(gs-sim-wasm PROPERTIES OUTPUT_NAME "gs-sim")
target_compile_definitions(gs-sim-wasm PRIVATE GS_SIMULATOR_WASM=1 ARDUINO=100)
target_include_directories(gs-sim-wasm PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/compat
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/Adafruit_GFX_Library
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/Adafruit_BusIO
  ${CMAKE_CURRENT_SOURCE_DIR}/../lib/NayukiQRCode/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/hmi)
if(EMSCRIPTEN)
  set_target_properties(gs-sim-wasm PROPERTIES SUFFIX ".js")
  target_link_options(gs-sim-wasm PRIVATE
    "-sWASM=1"
    "-sMODULARIZE=1"
    "-sEXPORT_ES6=1"
    "-sENVIRONMENT=web"
    "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPU8']"
    "-sEXPORTED_FUNCTIONS=['_gs_reset','_gs_press','_gs_release','_gs_hold','_gs_advance','_gs_set_link_json','_gs_set_navigation_json','_gs_set_sensor_json','_gs_set_device_status_json','_gs_set_configuration_json','_gs_set_logs_json','_gs_load_replay_json','_gs_snapshot_json','_gs_framebuffer','_gs_framebuffer_size','_gs_framebuffer_revision']")
endif()

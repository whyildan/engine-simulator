cmake_minimum_required(VERSION 3.16)
project(enginesim C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------- dependencies
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.90.9-docking
  GIT_SHALLOW TRUE)
FetchContent_Declare(implot
  GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG v0.16
  GIT_SHALLOW TRUE)
FetchContent_Declare(miniaudio
  GIT_REPOSITORY https://github.com/mackron/miniaudio.git
  GIT_TAG 0.11.21
  GIT_SHALLOW TRUE)
FetchContent_Declare(implot3d
  GIT_REPOSITORY https://github.com/brenocq/implot3d.git
  GIT_TAG v0.2
  GIT_SHALLOW TRUE)
foreach(dep imgui implot implot3d miniaudio)
  FetchContent_GetProperties(${dep})
  if(NOT ${dep}_POPULATED)
    FetchContent_Populate(${dep})
  endif()
endforeach()

if(MSVC)
  add_compile_options(/W3 /MP /utf-8)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
  # physics core is hot: always optimise it, even in Debug
  set(SIM_OPT_FLAGS /O2 /fp:fast)
else()
  set(SIM_OPT_FLAGS -O2 -ffast-math)
endif()

# ---------------------------------------------------------------- simulation core (no UI, no device audio)
add_library(enginesim_core STATIC
  src/sim/Presets.cpp
  src/sim/Waveguide.cpp
  src/sim/Exhaust.cpp
  src/sim/Intake.cpp
  src/sim/Cylinder.cpp
  src/sim/Engine.cpp
  src/sim/Dyno.cpp
  src/sim/Boost.cpp
  src/sim/Components.cpp
  src/sim/Thermal.cpp
  src/ecu/Sensors.cpp
  src/ecu/Ecu.cpp
  src/ecu/Calibration.cpp
  src/ecu/BaseMap.cpp
  src/ecu/CalibrationIO.cpp
  src/sim/Vehicle.cpp
  src/sim/Simulator.cpp
  src/audio/Listener.cpp
  src/audio/Convolver.cpp
  src/audio/ImpulseResponses.cpp
)
target_include_directories(enginesim_core PUBLIC src)
target_compile_options(enginesim_core PRIVATE ${SIM_OPT_FLAGS})

# ---------------------------------------------------------------- GUI app
option(ES_BUILD_APP "Build the GUI application" ON)
if(ES_BUILD_APP)
add_library(imgui_lib STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_win32.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_dx11.cpp
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp
  ${implot_SOURCE_DIR}/implot_demo.cpp
  ${implot3d_SOURCE_DIR}/implot3d.cpp
  ${implot3d_SOURCE_DIR}/implot3d_items.cpp
)
target_include_directories(imgui_lib PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends ${implot_SOURCE_DIR} ${implot3d_SOURCE_DIR})

add_executable(enginesim WIN32
  src/main.cpp
  src/audio/AudioOutput.cpp
  src/ui/App.cpp
  src/ui/Panels.cpp
  src/ui/Layout.cpp
  src/ui/TableEditor.cpp
  src/ui/Tuning.cpp
  src/ui/CalManager.cpp
  src/ui/Widgets.cpp
)
target_include_directories(enginesim PRIVATE src ${miniaudio_SOURCE_DIR})
target_link_libraries(enginesim PRIVATE enginesim_core imgui_lib d3d11 dxgi d3dcompiler winmm comdlg32)
endif()

# ---------------------------------------------------------------- headless tools
add_executable(render_wav tools/render_wav.cpp)
target_link_libraries(render_wav PRIVATE enginesim_core)

add_executable(sim_tests tests/sim_tests.cpp)
target_link_libraries(sim_tests PRIVATE enginesim_core)

add_executable(silence_probe tests/silence_probe.cpp)
target_link_libraries(silence_probe PRIVATE enginesim_core)

add_executable(boost_sound_probe tests/boost_sound_probe.cpp)
target_link_libraries(boost_sound_probe PRIVATE enginesim_core)

add_executable(drive_test tests/drive_test.cpp)
target_link_libraries(drive_test PRIVATE enginesim_core)

add_executable(boost_trace tests/boost_trace.cpp)
target_link_libraries(boost_trace PRIVATE enginesim_core)

add_executable(freerev_check tests/freerev_check.cpp)
target_link_libraries(freerev_check PRIVATE enginesim_core)

add_executable(ir_test tests/ir_test.cpp)
target_link_libraries(ir_test PRIVATE enginesim_core)
add_executable(parts_test tests/parts_test.cpp)
target_link_libraries(parts_test PRIVATE enginesim_core)
add_executable(ecu_test tests/ecu_test.cpp)
target_link_libraries(ecu_test PRIVATE enginesim_core)

add_executable(ve_probe tests/ve_probe.cpp)
target_link_libraries(ve_probe PRIVATE enginesim_core)

add_executable(map_dump tests/map_dump.cpp)
target_link_libraries(map_dump PRIVATE enginesim_core)

add_executable(cal_test tests/cal_test.cpp)
target_link_libraries(cal_test PRIVATE enginesim_core)

add_executable(driveline_probe tests/driveline_probe.cpp)
target_link_libraries(driveline_probe PRIVATE enginesim_core)

add_executable(ve_settle tests/ve_settle.cpp)
target_link_libraries(ve_settle PRIVATE enginesim_core)

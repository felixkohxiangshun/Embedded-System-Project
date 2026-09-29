# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(robo_car C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# One executable, one .uf2 - every subsystem's .c file below compiles into it.
# Per NFR-02: one header + one source file per subsystem, pins centralised
# in config.h, no circular #include dependencies between subsystem files.
add_executable(robo_car
        main.c
        motor/motor.c
        line_tracking/line_tracking.c
        barcode/barcode.c
        imu_terrain/imu_terrain.c
        obstacle_scan/obstacle_scan.c
        mission_state/mission_state.c
        telemetry/telemetry.c
        wifi_mqtt/wifi_mqtt.c
        )

pico_set_program_name(robo_car "robo_car")
pico_set_program_version(robo_car "0.1")

# Set to 1 if you want printf() debug output over USB serial
pico_enable_stdio_uart(robo_car 0)
pico_enable_stdio_usb(robo_car 1)

# Root (for config.h) plus every subsystem folder, so any file can
# #include "motor.h", "config.h", etc. without a relative path.
target_include_directories(robo_car PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/motor
        ${CMAKE_CURRENT_LIST_DIR}/line_tracking
        ${CMAKE_CURRENT_LIST_DIR}/barcode
        ${CMAKE_CURRENT_LIST_DIR}/imu_terrain
        ${CMAKE_CURRENT_LIST_DIR}/obstacle_scan
        ${CMAKE_CURRENT_LIST_DIR}/mission_state
        ${CMAKE_CURRENT_LIST_DIR}/telemetry
        ${CMAKE_CURRENT_LIST_DIR}/wifi_mqtt
        )

target_link_libraries(robo_car
        pico_stdlib
        pico_cyw43_arch_none
        hardware_pwm
        hardware_adc
        hardware_i2c
        hardware_pio
        )

pico_add_extra_outputs(robo_car)

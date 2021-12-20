# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/pi/pico/pico-sdk/")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(NAU7802 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(NAU7802 NAU7802.c )

pico_set_program_name(NAU7802 "NAU7802")
pico_set_program_version(NAU7802 "0.1")

pico_enable_stdio_uart(NAU7802 1)
pico_enable_stdio_usb(NAU7802 0)

# Add the standard library to the build
target_link_libraries(NAU7802 pico_stdlib)

# Add any user requested libraries
target_link_libraries(NAU7802
        hardware_dma
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_i2c
        pico_stdlib
        )

pico_add_extra_outputs(NAU7802)


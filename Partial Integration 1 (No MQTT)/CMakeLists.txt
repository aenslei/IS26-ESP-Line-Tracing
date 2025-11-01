cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(../example_auto_set_url.cmake)

project(roboDriverV2)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(roboDriverV2
        roboDriverV2.c
        WithPID.c
        IMUDriver.c
        )

# pull in common dependencies
target_link_libraries(roboDriverV2     
    pico_stdlib 
    hardware_gpio
    hardware_timer
    hardware_pwm
    hardware_i2c
)

# create map/bin/hex/uf2 file etc.
pico_enable_stdio_usb(roboDriverV2 1)
pico_enable_stdio_uart(roboDriverV2 1)
pico_add_extra_outputs(roboDriverV2)

cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(../example_auto_set_url.cmake)

project(roboDriverV3)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(roboDriverV3
        roboDriverV3.c
        WithPID.c
        IMUDriver.c
        IR_track.c
        decode_barcode.c
        )

# pull in common dependencies
target_link_libraries(roboDriverV3     
    pico_stdlib 
    hardware_gpio
    hardware_timer
    hardware_pwm
    hardware_i2c
    hardware_adc
)

# create map/bin/hex/uf2 file etc.
pico_enable_stdio_usb(roboDriverV3 1)
pico_enable_stdio_uart(roboDriverV3 1)
pico_add_extra_outputs(roboDriverV3)

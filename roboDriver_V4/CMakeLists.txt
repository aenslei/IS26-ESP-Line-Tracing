cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(../example_auto_set_url.cmake)

project(roboDriverV4)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(roboDriverV4
        roboDriverV4_FSM.c
        WithPID.c
        IMUDriver.c
        IR_track.c
        decode_barcode.c
        )

# pull in common dependencies
target_link_libraries(roboDriverV4     
    pico_stdlib 
    hardware_gpio
    hardware_timer
    hardware_pwm
    hardware_i2c
    hardware_adc
)

# Line Following Test Target
add_executable(roboDriverV4_LineTest
        roboDriverV4_FSM_LineTest.c
        WithPID.c
        IMUDriver.c
        IR_track.c
        decode_barcode.c
        )

target_link_libraries(roboDriverV4_LineTest     
    pico_stdlib 
    hardware_gpio
    hardware_timer
    hardware_pwm
    hardware_i2c
    hardware_adc
)

# create map/bin/hex/uf2 file etc.
pico_enable_stdio_usb(roboDriverV4 1)
pico_enable_stdio_uart(roboDriverV4 1)
pico_add_extra_outputs(roboDriverV4)

pico_enable_stdio_usb(roboDriverV4_LineTest 1)
pico_enable_stdio_uart(roboDriverV4_LineTest 1)
pico_add_extra_outputs(roboDriverV4_LineTest)


# Line Following + Correction FSM Test
add_executable(roboDriverV4_LineCorrection
        roboDriverV4_FSM_LineCorrection.c
        WithPID.c
        IMUDriver.c
        IR_track.c
        decode_barcode.c
        )

target_link_libraries(roboDriverV4_LineCorrection     
    pico_stdlib 
    hardware_gpio
    hardware_timer
    hardware_pwm
    hardware_i2c
    hardware_adc
)

pico_enable_stdio_usb(roboDriverV4_LineCorrection 1)
pico_enable_stdio_uart(roboDriverV4_LineCorrection 1)
pico_add_extra_outputs(roboDriverV4_LineCorrection)


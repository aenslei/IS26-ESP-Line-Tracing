cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(servo_uss_integration C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(servo_uss_integration
    main.c
    servo.c
    ultrasonic.c
)

pico_enable_stdio_usb(servo_uss_integration 1)
pico_enable_stdio_uart(servo_uss_integration 0)

target_link_libraries(servo_uss_integration
    pico_stdlib
    hardware_pwm
)

pico_add_extra_outputs(servo_uss_integration)

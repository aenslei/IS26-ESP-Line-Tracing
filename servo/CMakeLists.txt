cmake_minimum_required(VERSION 3.13)

# Initialize Pico SDK before project()
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(servo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(servo.elf
    main.c
)

pico_enable_stdio_usb(servo.elf 1)
pico_enable_stdio_uart(servo.elf 0)

target_link_libraries(servo.elf
    pico_stdlib
    hardware_pwm
)

pico_add_extra_outputs(servo.elf)



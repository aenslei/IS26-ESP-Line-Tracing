cmake_minimum_required(VERSION 3.13)
project(servo C CXX ASM)

add_compile_options(-mcpu=cortex-m0plus -mthumb)

# Pull in Pico SDK
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
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


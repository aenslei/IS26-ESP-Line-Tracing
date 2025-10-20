cmake_minimum_required(VERSION 3.13)

# Import Pico SDK before project()
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ultrasonic_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize Pico SDK
pico_sdk_init()

add_executable(ultrasonic_test
    main.c
    ultrasonic.c
)

pico_enable_stdio_usb(ultrasonic_test 1)
pico_enable_stdio_uart(ultrasonic_test 0)

target_link_libraries(ultrasonic_test
    pico_stdlib
)

pico_add_extra_outputs(ultrasonic_test)


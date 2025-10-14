cmake_minimum_required(VERSION 3.13)

# Corrected path to pico_sdk_import.cmake
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ultrasonic_interrupt_demo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize SDK
pico_sdk_init()

# Add your files
add_executable(ultrasonic_interrupt_demo
    main.c
    ultrasonic_interrupt.c
)

# Link libraries
target_link_libraries(ultrasonic_interrupt_demo
    pico_stdlib
    hardware_gpio
    hardware_timer
)

# Enable USB output
pico_enable_stdio_usb(ultrasonic_interrupt_demo 1)
pico_enable_stdio_uart(ultrasonic_interrupt_demo 0)

# Generate .uf2 and .bin
pico_add_extra_outputs(ultrasonic_interrupt_demo)


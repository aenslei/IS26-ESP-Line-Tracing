cmake_minimum_required(VERSION 3.13)

# Initialize the Pico SDK before defining the project
include(/mnt/c/Users/lewta/Desktop/Pico_wsl/pico-sdk/external/pico_sdk_import.cmake)


project(servo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Define executable and source files
add_executable(servo
    main.c
    servo.c
)

# Enable USB serial (for Tera Term) and disable UART
pico_enable_stdio_usb(servo 1)
pico_enable_stdio_uart(servo 0)

# Link required libraries
target_link_libraries(servo
    pico_stdlib
    hardware_pwm
)

# Generate .uf2, .elf, and .bin files
pico_add_extra_outputs(servo)





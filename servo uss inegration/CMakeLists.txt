cmake_minimum_required(VERSION 3.13)
include(/mnt/c/Users/lewta/Desktop/Pico_wsl/pico-sdk/external/pico_sdk_import.cmake)

project(servo_uss C CXX ASM)
pico_sdk_init()

add_executable(servo_uss
    main.c
    servo.c
    ultrasonic.c
)

target_link_libraries(servo_uss pico_stdlib hardware_pwm)
pico_add_extra_outputs(servo_uss)
pico_enable_stdio_usb(servo_uss 1)
pico_enable_stdio_uart(servo_uss 0)

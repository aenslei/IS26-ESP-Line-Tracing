cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(imu_driver)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Main IMU driver
add_executable(imu_driver
    IMUDriver.c
)

target_link_libraries(imu_driver
    pico_stdlib
    hardware_i2c
)

pico_enable_stdio_usb(imu_driver 1)
pico_enable_stdio_uart(imu_driver 1)
pico_add_extra_outputs(imu_driver)
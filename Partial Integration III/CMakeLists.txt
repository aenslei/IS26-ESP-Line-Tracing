cmake_minimum_required(VERSION 3.13)

# Force the Wi-Fi board (uses CYW43439 on Pico W)
set(PICO_BOARD pico_w)

# Import Pico SDK
include(/mnt/c/Users/lewta/Desktop/Pico_wsl/pico_sdk_import.cmake)

# Project metadata
project(wifi_telemetry_servo_uss C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# ✅ Target Cortex-M0+ (RP2040)
add_compile_options(-mcpu=cortex-m0plus -mthumb)
add_link_options(-mcpu=cortex-m0plus -mthumb)

# Initialize the SDK
pico_sdk_init()

# Define your executable and source files
add_executable(wifi_telemetry_servo_uss
    main.c
    functions.c
    telemetry.c
    servo.c
    ultrasonic.c
)

# Include paths (current dir + lwipopts)
target_include_directories(wifi_telemetry_servo_uss PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/lwipopts
)

# Link against Pico libraries and other required libs
target_link_libraries(wifi_telemetry_servo_uss
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
    hardware_pwm
)

# Enable USB and UART stdio
pico_enable_stdio_usb(wifi_telemetry_servo_uss 1)
pico_enable_stdio_uart(wifi_telemetry_servo_uss 1)

# Generate .uf2 and .bin
pico_add_extra_outputs(wifi_telemetry_servo_uss)


# Main barcode scanner
add_executable(adc_console
    barcodedigitalver2.c
)

target_link_libraries(adc_console
    pico_stdlib
    hardware_adc
    hardware_pwm
)

pico_enable_stdio_usb(barcodedigitalver2 1)
pico_enable_stdio_uart(barcodedigitalver2 0)
pico_add_extra_outputs(barcodedigitalver2)

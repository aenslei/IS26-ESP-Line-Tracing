add_executable(roboDriver roboDriver.c)

# pull in common dependencies and additional hardware support
target_link_libraries(roboDriver 
    pico_stdlib 
    hardware_gpio
    hardware_timer
    hardware_pwm
)

# enable usb output, enable uart output
pico_enable_stdio_usb(roboDriver 1)
pico_enable_stdio_uart(roboDriver 1)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(roboDriver)

# add url via pico_set_program_url
example_auto_set_url(roboDriver)

add_executable(WithPID
        WithPID.c
        )

# pull in common dependencies
target_link_libraries(WithPID     
    pico_stdlib 
    hardware_gpio
    hardware_timer
    hardware_pwm
)

# create map/bin/hex/uf2 file etc.
pico_enable_stdio_usb(WithPID 1)
pico_enable_stdio_uart(WithPID 1)

pico_add_extra_outputs(WithPID)

# add url via pico_set_program_url
example_auto_set_url(WithPID)

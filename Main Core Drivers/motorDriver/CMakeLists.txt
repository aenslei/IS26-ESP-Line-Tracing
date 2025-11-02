add_executable(hello_pwm
        hello_pwm.c
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(hello_pwm pico_stdlib hardware_pwm)

# enable usb output, enable uart output
pico_enable_stdio_usb(hello_pwm 1)
pico_enable_stdio_uart(hello_pwm 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(hello_pwm)

# add url via pico_set_program_url
example_auto_set_url(hello_pwm)

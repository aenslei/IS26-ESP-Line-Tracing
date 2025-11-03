add_executable(pico_wifi_client
    main.c
    functions.c
)

# Make compiler aware of header files
target_include_directories(pico_wifi_client PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}         # your project folder
    ${CMAKE_CURRENT_LIST_DIR}/lwipopts # folder with lwipopts.h
)

target_link_libraries(pico_wifi_client
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
)

pico_add_extra_outputs(pico_wifi_client)

pico_enable_stdio_usb(pico_wifi_client 1)
pico_enable_stdio_uart(pico_wifi_client 1)
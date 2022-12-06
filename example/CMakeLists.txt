pico_sdk_init()

add_executable(example
        example.c
        )

target_link_libraries(example pico_stdlib pico_multicore iono_rp_d16)

# disable standard output on UART (used by RS-485)
pico_enable_stdio_uart(example 0)

# enable standard output on USB
pico_enable_stdio_usb(example 1)

pico_add_extra_outputs(example)

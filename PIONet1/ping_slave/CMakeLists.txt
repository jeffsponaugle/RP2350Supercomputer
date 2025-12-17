cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ping_slave C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# ============================================================================
# Ping Slave (Echo) Application
# ============================================================================

add_executable(ping_slave
    main.c
    pio_spi_dma.c
)

target_include_directories(ping_slave PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Generate PIO headers
pico_generate_pio_header(ping_slave ${CMAKE_CURRENT_LIST_DIR}/spi_tx_cs.pio)
pico_generate_pio_header(ping_slave ${CMAKE_CURRENT_LIST_DIR}/spi_rx_cs.pio)

target_link_libraries(ping_slave
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_clocks
    hardware_irq
    hardware_gpio
)

# Enable USB serial output
pico_enable_stdio_usb(ping_slave 1)
pico_enable_stdio_uart(ping_slave 0)

# Create UF2 file for easy flashing
pico_add_extra_outputs(ping_slave)

cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(edm C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(main
    main.cpp
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
pico_add_extra_outputs(main)
target_link_libraries(main pico_multicore pico_stdlib hardware_adc hardware_pwm hardware_dma hardware_pio hardware_i2c pico_i2c_slave pico_cyw43_arch_lwip_threadsafe_background)

pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/pulse_analyzer.pio)

# Add lwipopts.h to the include path
target_include_directories(main PRIVATE ${CMAKE_CURRENT_LIST_DIR})


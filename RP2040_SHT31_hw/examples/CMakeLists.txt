cmake_minimum_required(VERSION 3.12)
project (RP2040_SHT31 C CXX ASM)
set(PICO_DEV_SOURCE ${WORKING_DIRECTORY}pico-dev)
set(PICO_SDK_PATH ${WORKING_DIRECTORY}/pico-sdk/src/rp2_common/hardware_i2c)
add_executable(RP2040_SHT31 
    main.cpp)
add_directory($ENV{PICO_DEV_SOURCE}/RP2040_SHT31_hw)
add_directory($ENV{PICO_SDK_PATH}/RP2040_SHT31_hw)
target_include_directories(RP2040_SHT31 INTERFACE $ENV{PICO_DEV_SOURCE}/RP2040_SHT31_hw/include)
target_link_libraries(RP2040_SHT31_hw hardware_i2c pico_stdlib)
target_include_directories(RP2040_SHT31 $ENV{PICO_SDK_PATH}/pico-sdk/src/rp2_common/hardware_i2c)
target_link_libraries(RP2040_SHT31 hardware_i2c pico_stdlib RP2040_SHT31_hw)

pico_add_extra_outputs(RP2040_SHT31)
pico_enable_stdio_usb(RP2040_SHT31 1)
pico_enable_stdio_uart(RP2040_SHT31 0)
add_library(RP2040_SHT31_hw STATIC
    RP2040_SHT31_hw.cpp)
target_include_directories(RP2040_SHT31_hw PUBLIC
    .
    /include)
target_link_libraries(RP2040_SHT31_hw hardware_i2c pico_stdlib)
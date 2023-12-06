add_library(RP2040_HTU21DF_hw STATIC
    RP2040_HTU21DF_hw.cpp)
target_include_directories(RP2040_HTU21DF_hw PUBLIC
    .
    /include)
target_link_libraries(RP2040_HTU21DF_hw hardware_i2c pico_stdlib)
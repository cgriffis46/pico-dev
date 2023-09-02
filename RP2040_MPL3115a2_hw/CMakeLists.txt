add_library(RP2040_MPL3115a2_hw STATIC
    RP2040_MPL3115a2_hw.cpp)
target_include_directories(RP2040_MPL3115a2_hw PUBLIC
    .
    /include)
target_link_libraries(RP2040_MPL3115a2_hw hardware_i2c pico_stdlib)
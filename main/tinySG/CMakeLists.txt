cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(tinySG LANGUAGES C)
endif()

add_library(tinysg STATIC
    src/tinysg.c
    src/tinysg_math.c
    src/tinysg_fonts.c
    src/tinysg_style.c
    src/tinysg_shapes.c
    src/tinysg_damage.c
    src/tinysg_display.c
    src/tinysg_effects.c
    src/tinysg_layout.c
)
add_library(tinySG::tinySG ALIAS tinysg)
target_include_directories(tinysg PUBLIC include)
target_compile_features(tinysg PUBLIC c_std_99)

if(ESP_PLATFORM AND CONFIG_IDF_TARGET_ESP32P4)
    target_sources(tinysg PRIVATE port/tinysg_esp32p4.c)
    target_compile_definitions(tinysg PUBLIC TINYSG_ESP32P4=1)
    target_link_libraries(tinysg PRIVATE idf::esp_driver_ppa idf::esp_hw_support)
endif()

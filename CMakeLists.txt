# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
set(COMPONENTS main additional_components)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESPREZ)

# Dodanie niestandardowego celu, który wykona kopiowanie po zakończeniu budowy
add_custom_target(copy_usbd ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_SOURCE_DIR}/addinotial_compoments/usbd.c"  # Ścieżka do zmodyfikowanego pliku
    "${CMAKE_SOURCE_DIR}/managed_components/espressif__tinyusb/src/device/usbd.c"  # Miejsce docelowe
    COMMENT "Copying modified usbd.c to managed_components"
)

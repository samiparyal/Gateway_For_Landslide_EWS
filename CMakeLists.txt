# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# esp_lcd's RGB panel driver isn't used anywhere in this project, and its
# rgb/esp_lcd_panel_rgb.c reproducibly crashes this toolchain's GCC with an
# internal compiler error (segfault during register allocation) at -Og.
# Excluding the whole component avoids compiling that file at all.
set(EXCLUDE_COMPONENTS "esp_lcd")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(s3_gateway_for_landslide)

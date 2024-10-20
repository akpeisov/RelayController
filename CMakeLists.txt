# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(PROJECT_VER "1.0.0.1")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS ../ESP32/components ../../ESP32/esp-idf-lib/components)

project(RelayController)


#                    "${workspaceFolder}/../ESP32/components/**",

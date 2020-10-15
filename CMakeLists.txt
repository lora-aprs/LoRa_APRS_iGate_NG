
cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

list(APPEND EXTRA_COMPONENT_DIRS externals/smooth/smooth_component)

project(LoRa_APRS_iGate_NG)

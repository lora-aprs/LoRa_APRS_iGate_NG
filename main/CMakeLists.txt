
cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 17)

set(SOURCES main.cpp lora.cpp hw/sx1287.cpp)

idf_component_register(SRCS ${SOURCES}
        INCLUDE_DIRS
            ${CMAKE_CURRENT_LIST_DIR}
            $ENV{IDF_PATH}/components
        REQUIRES
            smooth_component
        )


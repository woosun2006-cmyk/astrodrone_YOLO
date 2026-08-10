cmake_minimum_required(VERSION 3.10)
project(astro_drone_gcs CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/sender
    ${CMAKE_CURRENT_SOURCE_DIR}/../control  # yaml_settings.hpp
)

find_package(Threads REQUIRED)

add_library(gcs_fec STATIC
    sender/crc32.cpp
    sender/fec_encoder.cpp
    sender/udp_sender.cpp
)

add_executable(telem_sender
    sender/telem_sender_main.cpp
    ../control/yaml_settings.cpp
    ../control/drone_lib.cpp
    ../control/mav_transport.cpp
    ../control/target_link.cpp
    ../control/pos_calculator.cpp
)
target_link_libraries(telem_sender PRIVATE gcs_fec Threads::Threads)

cmake_minimum_required(VERSION 3.16)
project(astrodrone_simulation_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED ASTRODRONE_REPO)
    message(FATAL_ERROR "ASTRODRONE_REPO must point to the repository root")
endif()

add_executable(read_only_telemetry read_only_telemetry.cpp)
target_include_directories(read_only_telemetry PRIVATE "${ASTRODRONE_REPO}/control")
target_compile_options(read_only_telemetry PRIVATE -Wall -Wextra)

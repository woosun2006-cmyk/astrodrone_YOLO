cmake_minimum_required(VERSION 3.10)
project(astro_drone_control CXX)

option(BUILD_TESTS "Build offline unit and characterization tests" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(drone_lib STATIC
    drone_lib.cpp
    mav_transport.cpp
    yaml_settings.cpp
    target_link.cpp
    pos_calculator.cpp
)
target_include_directories(drone_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(target_json STATIC target_json.cpp)
target_include_directories(target_json PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Threads REQUIRED)

add_executable(control control.cpp)
target_link_libraries(control PRIVATE drone_lib Threads::Threads)

add_executable(check_link check_link.cpp)
target_link_libraries(check_link PRIVATE drone_lib Threads::Threads)

add_executable(get_gps_location get_gps_location.cpp)
target_link_libraries(get_gps_location PRIVATE drone_lib Threads::Threads)

add_executable(test_arm test_arm.cpp)
target_link_libraries(test_arm PRIVATE drone_lib Threads::Threads)

add_executable(check_alt check_alt.cpp)
target_link_libraries(check_alt PRIVATE drone_lib Threads::Threads)

add_executable(emergency emergency.cpp)
target_link_libraries(emergency PRIVATE drone_lib Threads::Threads)

add_executable(target_distance target_distance.cpp)
target_link_libraries(target_distance PRIVATE drone_lib target_json Threads::Threads)

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

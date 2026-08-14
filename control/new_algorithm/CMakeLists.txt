cmake_minimum_required(VERSION 3.10)
project(astro_drone_hybrid_guidance CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra)

# Standalone project instead of add_subdirectory()-ing this into
# control/CMakeLists.txt, which stays untouched. Compiles control/'s
# existing sources fresh (reuse, not modification, not a copy pasted with
# changes) alongside this folder's new files.
set(CONTROL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CONTROL_DIR})

find_package(Threads REQUIRED)

add_executable(hybrid_guidance
    hybrid_guidance_main.cpp
    hybrid_guidance.cpp
    ${CONTROL_DIR}/drone_lib.cpp
    ${CONTROL_DIR}/mav_transport.cpp
    ${CONTROL_DIR}/yaml_settings.cpp
    ${CONTROL_DIR}/target_link.cpp
    ${CONTROL_DIR}/pos_calculator.cpp
)
target_link_libraries(hybrid_guidance PRIVATE Threads::Threads)

# Must land two levels below the repo root (astro-drone/control/build_new_algorithm/),
# the same depth as control/build/ - yaml_settings.cpp's load_setting_file()
# (unmodified) and this folder's load_hybrid_guidance_settings() both derive
# setting/'s path from "<this binary's own directory>/../../setting", so the
# binary has to sit at that same depth for either lookup to succeed.
set_target_properties(hybrid_guidance PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CONTROL_DIR}/build_new_algorithm)

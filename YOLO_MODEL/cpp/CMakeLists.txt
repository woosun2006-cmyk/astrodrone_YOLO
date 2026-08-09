cmake_minimum_required(VERSION 3.10)
project(astro_drone_yolo_live CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)          # cmake 3.10 predates the CUDAToolkit module
find_package(Threads REQUIRED)

find_library(NVINFER_LIB nvinfer)
find_library(NVONNXPARSER_LIB nvonnxparser)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin)
if(NOT NVINFER_LIB OR NOT NVONNXPARSER_LIB)
    message(FATAL_ERROR "TensorRT (libnvinfer/libnvonnxparser) not found - "
                         "this target only builds on a JetPack image with "
                         "libnvinfer-dev / libnvonnxparsers-dev installed.")
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../control  # yaml_settings.hpp
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
)

add_executable(yolo_live
    yolo_live.cpp
    trt_engine.cpp
    http_server.cpp
    sysinfo.cpp
    report.cpp
    ../../control/yaml_settings.cpp
)

target_link_libraries(yolo_live PRIVATE
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${NVINFER_LIB}
    ${NVONNXPARSER_LIB}
    ${NVINFER_PLUGIN_LIB}
    Threads::Threads
)

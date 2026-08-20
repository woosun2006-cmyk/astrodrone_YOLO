cmake_minimum_required(VERSION 3.10)
project(astro_drone_yolo_live CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra)

set(TENSORRT_ROOT "$ENV{TENSORRT_ROOT}" CACHE PATH "TensorRT installation root")
if(TENSORRT_ROOT)
    set(TENSORRT_INCLUDE_HINT "${TENSORRT_ROOT}/include")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
        set(TENSORRT_LIB_HINT "${TENSORRT_ROOT}/targets/aarch64-linux-gnu/lib")
    else()
        set(TENSORRT_LIB_HINT "${TENSORRT_ROOT}/targets/x86_64-linux-gnu/lib")
    endif()
endif()

find_package(OpenCV REQUIRED)
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.17")
    find_package(CUDAToolkit REQUIRED)
    set(CUDA_INCLUDE_DIRS ${CUDAToolkit_INCLUDE_DIRS})
    set(CUDA_RUNTIME_TARGET CUDA::cudart)
else()
    find_package(CUDA REQUIRED)
    set(CUDA_RUNTIME_TARGET ${CUDA_LIBRARIES})
endif()
find_package(Threads REQUIRED)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(GZ_FRAME_SOURCE QUIET gz-transport gz-msgs)
    # Some Gazebo package metadata on Ubuntu 24.04 advertises an old,
    # missing cmake5 include directory. Keep only directories that exist.
    foreach(GZ_INCLUDE_DIR IN LISTS GZ_FRAME_SOURCE_INCLUDE_DIRS)
        if(IS_DIRECTORY "${GZ_INCLUDE_DIR}")
            list(APPEND GZ_VALID_INCLUDE_DIRS "${GZ_INCLUDE_DIR}")
        endif()
    endforeach()
endif()

find_library(NVINFER_LIB NAMES nvinfer libnvinfer.so.8.6.1
    PATHS "${TENSORRT_LIB_HINT}")
find_library(NVONNXPARSER_LIB NAMES nvonnxparser libnvonnxparser.so.8.6.1
    PATHS "${TENSORRT_LIB_HINT}")
find_library(NVINFER_PLUGIN_LIB NAMES nvinfer_plugin libnvinfer_plugin.so.8.6.1
    PATHS "${TENSORRT_LIB_HINT}")
find_path(NVINFER_INCLUDE_DIR NvInfer.h PATHS "${TENSORRT_INCLUDE_HINT}")
find_path(NVONNXPARSER_INCLUDE_DIR NvOnnxParser.h PATHS "${TENSORRT_INCLUDE_HINT}")
if(NOT NVINFER_LIB OR NOT NVONNXPARSER_LIB)
    message(FATAL_ERROR "TensorRT (libnvinfer/libnvonnxparser) not found - "
                         "this target only builds on a JetPack image with "
                         "libnvinfer-dev / libnvonnxparsers-dev installed.")
endif()
if(NOT NVINFER_INCLUDE_DIR OR NOT NVONNXPARSER_INCLUDE_DIR)
    message(FATAL_ERROR "TensorRT headers not found; install TensorRT development headers.")
endif()
if(TENSORRT_LIB_HINT)
    # The TensorRT tarball ships versioned libraries without system-wide
    # symlinks. Help the linker resolve their cuDNN dependencies and keep the
    # user-space TensorRT path in the executable's build-time runtime path.
    link_directories("${TENSORRT_LIB_HINT}")
    set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath-link,${TENSORRT_LIB_HINT}")
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../control  # yaml_settings.hpp
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${NVINFER_INCLUDE_DIR}
    ${NVONNXPARSER_INCLUDE_DIR}
)

add_executable(yolo_live
    yolo_live.cpp
    gcs_video_publisher.cpp
    frame_source.cpp
    trt_engine.cpp
    http_server.cpp
    sysinfo.cpp
    report.cpp
    ../../control/yaml_settings.cpp
)

target_link_libraries(yolo_live PRIVATE
    ${OpenCV_LIBS}
    ${CUDA_RUNTIME_TARGET}
    ${NVINFER_LIB}
    ${NVONNXPARSER_LIB}
    ${NVINFER_PLUGIN_LIB}
    Threads::Threads
)
if(TENSORRT_LIB_HINT)
    set_target_properties(yolo_live PROPERTIES
        BUILD_RPATH "${TENSORRT_LIB_HINT};/usr/lib/x86_64-linux-gnu")
endif()
if(GZ_FRAME_SOURCE_FOUND)
    target_compile_definitions(yolo_live PRIVATE ASTRODRONE_HAVE_GZ_FRAME_SOURCE=1)
    target_include_directories(yolo_live PRIVATE ${GZ_VALID_INCLUDE_DIRS})
    target_link_directories(yolo_live PRIVATE ${GZ_FRAME_SOURCE_LIBRARY_DIRS})
    target_link_libraries(yolo_live PRIVATE ${GZ_FRAME_SOURCE_LIBRARIES})
endif()

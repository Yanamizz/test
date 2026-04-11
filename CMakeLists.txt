# cmake needs this line
cmake_minimum_required(VERSION 3.10)
 
# Define project name
project(radar_track)

# Release模式下的编译指令
# SET(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -s")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++17 -pthread -Wall -Wl")
 
# Debug模式下的编译指令
SET(CMAKE_BUILD_TYPE "Debug")
# Add -Wno-maybe-uninitialized to reduce false-positive warnings from Eigen internals
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++17 -pthread -O2 -Wall -Wno-maybe-uninitialized -DNDEBUG") 
 
set(OpenCV_LIBS opencv_videoio opencv_imgcodecs opencv_imgproc opencv_core opencv_dnn opencv_highgui opencv_calib3d)
find_package(Eigen3 REQUIRED)

# 允许优先使用仓库内 third_lib/openvino 的 CMake 包配置
set(_LOCAL_OPENVINO_DIR "${PROJECT_SOURCE_DIR}/third_lib/openvino/build")
if(EXISTS "${_LOCAL_OPENVINO_DIR}/OpenVINOConfig.cmake")
    list(APPEND CMAKE_PREFIX_PATH "${_LOCAL_OPENVINO_DIR}")
endif()
find_package(OpenVINO CONFIG QUIET)

include_directories(
    /usr/include
    /usr/include/opencv4
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/third_lib/serial/include
    ${PROJECT_SOURCE_DIR}/third_lib/kalman/include
    ${PROJECT_SOURCE_DIR}/third_lib/Galaxy_camera/inc
    ${EIGEN3_INCLUDE_DIR}
)

find_library(SERIAL_LIB serial)
if(SERIAL_LIB)
    message(STATUS "Using system serial library: ${SERIAL_LIB}")
else()
    if(EXISTS "${PROJECT_SOURCE_DIR}/third_lib/serial/CMakeLists.txt")
        add_subdirectory(${PROJECT_SOURCE_DIR}/third_lib/serial EXCLUDE_FROM_ALL)
        set(SERIAL_LIB serial)
        message(STATUS "System serial library not found, fallback to bundled third_lib/serial")
    else()
        message(FATAL_ERROR "serial library not found. Please install libserial or provide third_lib/serial.")
    endif()
endif()
 
# 设置可执行文件输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
 
link_directories(
    /usr/local/lib/
    ${PROJECT_SOURCE_DIR}/third_lib/Galaxy_camera/lib/x86_64
)
 
# 仅编译 src 目录下源码；第三方库使用系统/预编译库链接

# 递归检索指定目录下所有 .cc
set(SRC_ROOT ${PROJECT_SOURCE_DIR}/src)
file(GLOB_RECURSE SRCS ${SRC_ROOT}/*.cc)

# 为每个 .cc 生成对应文件名的可执行文件
foreach(src ${SRCS})
    get_filename_component(name_we ${src} NAME_WE)
    add_executable(${name_we} ${src})
    target_link_libraries(${name_we}
        ${OpenCV_LIBS}
        ${SERIAL_LIB}
    )

    if(name_we STREQUAL "ImagePredict")
        target_link_libraries(${name_we}
            gxiapi
        )

        if(OpenVINO_FOUND)
            target_link_libraries(${name_we}
                openvino::runtime
            )
            message(STATUS "ImagePredict: OpenVINO enabled via ${OpenVINO_DIR}")
        else()
            message(WARNING "ImagePredict: OpenVINO package not found. It will compile fallback path that throws at runtime.")
        endif()
    endif()
endforeach()
 

# Link moved into per-target loop above
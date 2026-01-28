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
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_DEBUG}")
# Add -Wno-maybe-uninitialized to reduce false-positive warnings from Eigen internals
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++17 -pthread -O2 -Wall -Wno-maybe-uninitialized -DNDEBUG") 
 
set(OpenCV_LIBS opencv_videoio opencv_imgcodecs opencv_imgproc opencv_core opencv_dnn opencv_highgui opencv_calib3d)
 
find_package(Eigen3 REQUIRED)

include_directories(
    /usr/local/include
    /usr/local/include/opencv4
    /usr/local/include/onnxruntime
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/third_lib/kalman/include
    ${PROJECT_SOURCE_DIR}/third_lib/Galaxy_camera/inc
    ${EIGEN3_INCLUDE_DIR}
)
# find_package(serial REQUIRED)

 
# 设置可执行文件输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
 
link_directories(
    /usr/local/lib/
    ${PROJECT_SOURCE_DIR}/third_lib/Galaxy_camera/lib/x86_64
)
 
# 递归检索指定目录下所有 .cc
set(SRC_ROOT ${PROJECT_SOURCE_DIR}/src)
file(GLOB_RECURSE SRCS ${SRC_ROOT}/*.cc)

# 为每个 .cc 生成对应文件名的可执行文件
foreach(src ${SRCS})
    get_filename_component(name_we ${src} NAME_WE)
    add_executable(${name_we} ${src})
    target_link_libraries(${name_we}
        -lonnxruntime
        -lgxiapi
        ${OpenCV_LIBS}
        serial  # 添加 serial 库
    )       
endforeach()
 
option(KALMAN_BUILD_EXAMPLES "Build the Kalman examples" OFF)  # 可选，关闭示例
add_subdirectory(${PROJECT_SOURCE_DIR}/third_lib/kalman)

# Link moved into per-target loop above
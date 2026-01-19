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
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++17 -pthread") 
 
set(OpenCV_LIBS opencv_videoio opencv_imgcodecs opencv_imgproc opencv_core opencv_dnn opencv_highgui)
 
include_directories(
    /usr/local/include/opencv4
    ${PROJECT_SOURCE_DIR}/include
    /usr/local/include/onnxruntime
    ${PROJECT_SOURCE_DIR}/third_lib/kalman/include
    ${EIGEN3_INCLUDE_DIR}
)
 
link_directories(
    /usr/local/lib/
)
 
#递归指定源码的路径
file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*.cc)
 
# Declare the executable target built from your sources
add_executable(radar_track ${SRCS})
 
option(KALMAN_BUILD_EXAMPLES "Build the Kalman examples" OFF)  # 可选，关闭示例
add_subdirectory(${PROJECT_SOURCE_DIR}/third_lib/kalman)

# Link your application with OpenCV libraries
target_link_libraries(radar_track 
-lonnxruntime 
${OpenCV_LIBS}  
)
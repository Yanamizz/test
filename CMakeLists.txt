cmake_minimum_required(VERSION 3.16)

project(Radar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(radar
    src/main.cc
    include/radar/libs/librm/src/librm/device/device.cc
    include/radar/libs/librm/src/librm/modules/crc.cc
)

target_include_directories(radar PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include/radar/libs/librm/src
    ${CMAKE_SOURCE_DIR}/include/radar/libs/librm/libs/etl/include
    ${CMAKE_SOURCE_DIR}/include/radar/libs/librm/libs/eternal/include
    ${CMAKE_SOURCE_DIR}/include/radar/libs/librm/libs/eigen
    ${CMAKE_SOURCE_DIR}/include/radar/libs/librm/libs/kalman/include
    ${CMAKE_SOURCE_DIR}/include/radar/libs/librm/libs/rpc_core/include
    ${CMAKE_SOURCE_DIR}/include/radar/libs/librm/libs/hipnuc_sdk/include
    ${CMAKE_SOURCE_DIR}/include/radar/libs/librm/libs/ardupilot_filter_lib
)

target_compile_definitions(radar PRIVATE
    RADAR_DEFAULT_LOG_DIR="${CMAKE_SOURCE_DIR}/test/logs"
)

add_subdirectory(test)

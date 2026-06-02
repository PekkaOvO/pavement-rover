cmake_minimum_required(VERSION 3.10)
project(KF-GINS)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands for clangd" FORCE)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_definitions( -D_USE_MATH_DEFINES=True -D_CRT_SECURE_NO_WARNINGS)
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-result")
    list(APPEND ABSL_GCC_FLAGS "-w")
endif()

file(GLOB_RECURSE SOURCE
        src/kf_gins.cpp
        src/fileio/fileloader.cc
        src/fileio/filesaver.cc
        src/kf-gins/gi_engine.cpp
        src/realtime/realtime.cpp
        src/kf-gins/insmech.cpp)

include_directories(${PROJECT_SOURCE_DIR}/src)
add_executable(${PROJECT_NAME} ${SOURCE})
add_executable(${PROJECT_NAME}-PathControl
        src/path_control/path_control_main.cpp
        src/path_control/path_controller.cpp)
add_executable(${PROJECT_NAME}-GnssPathControl
        src/path_control/gnss_path_control_main.cpp
        src/path_control/path_controller.cpp
        src/path_control/gps_publisher.cpp
        src/path_control/vehicle_cmd_listener.cpp)

# Standalone test control program (crack handling full-flow test)
add_executable(${PROJECT_NAME}-TestControl test_control.cpp)
target_link_libraries(${PROJECT_NAME}-TestControl pthread)
set_target_properties(${PROJECT_NAME}-TestControl PROPERTIES
    COMPILE_FLAGS "-pthread"
    LINK_FLAGS "-pthread")

# gd32_bridge: standalone daemon for GD32 image bridge
add_subdirectory(gd32_bridge)

# Eigen3
include_directories(ThirdParty/eigen-3.3.9)

# yaml-cpp-0.7.0
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory(ThirdParty/yaml-cpp-0.7.0)
if (TARGET yaml-cpp AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    target_compile_options(yaml-cpp PRIVATE -w)
endif()
target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_link_libraries(${PROJECT_NAME}-PathControl yaml-cpp)
target_link_libraries(${PROJECT_NAME}-GnssPathControl yaml-cpp)

# abseil
set(ABSL_PROPAGATE_CXX_STD true)
add_subdirectory(ThirdParty/abseil-cpp-20220623.1)
target_link_libraries(${PROJECT_NAME}
        absl::strings
        absl::str_format
        absl::time)

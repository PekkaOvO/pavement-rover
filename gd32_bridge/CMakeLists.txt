cmake_minimum_required(VERSION 3.10)

# gd32_bridge is cross-compiled for ARM, typically included
# from the parent KF-GINS CMakeLists.txt or built standalone.
# This file can be used standalone with -DCMAKE_TOOLCHAIN_FILE.

project(gd32_bridge CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(gd32_bridge
    main.cpp
    uart_reader.cpp
    gps_receiver.cpp
    tcp_sender.cpp
)

target_include_directories(gd32_bridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# No external dependencies beyond POSIX/libc
